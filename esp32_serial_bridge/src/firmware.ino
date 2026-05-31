/*
 * ESP32 Serial Bridge Firmware — Approach 1
 * -----------------------------------------
 * Replaces micro-ROS with a simple text-based serial protocol.
 * The RPi4 sends wheel velocity commands; this firmware drives the
 * TB6612FNG motor driver under PID control and reports encoder ticks.
 *
 * Serial protocol (USB, 115200 baud):
 *   RPi4  → ESP32:  "W <left_rpm> <right_rpm>\n"
 *   ESP32 → RPi4:   "E <left_ticks> <right_ticks> <dt_us>\n"
 *
 * Hardware:
 *   Left  motor: PWMA=25, AIN1=27, AIN2=26
 *   Right motor: PWMB=19, BIN1=5,  BIN2=18
 *   Left  encoder: A=34, B=35
 *   Right encoder: A=32, B=39
 *   STBY: connect to 3.3V (always enable)
 *
 * Author: Navid's project — Approach 1 custom serial bridge
 */

#include <Arduino.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
// Left motor (MOTOR1)
#define LEFT_PWM_PIN   25
#define LEFT_IN_A      27
#define LEFT_IN_B      26

// Right motor (MOTOR2)
#define RIGHT_PWM_PIN  19
#define RIGHT_IN_A     5
#define RIGHT_IN_B     18

// Encoder pins: 34, 35, 39 are input-only on ESP32 (no pull-up/down, interrupt-capable)
// GPIO32 is bidirectional and also interrupt-capable — used for RIGHT_ENC_A
#define LEFT_ENC_A     34
#define LEFT_ENC_B     35
#define RIGHT_ENC_A    32
#define RIGHT_ENC_B    39

// ── PWM / LEDC configuration ─────────────────────────────────────────────────
// ESP32 uses the LEDC peripheral for PWM — NOT analogWrite()
#define PWM_FREQ       20000   // 20 kHz (inaudible)
#define PWM_RESOLUTION 10      // 10-bit: 0..1023
#define LEFT_PWM_CH    0       // LEDC channel 0 for left motor
#define RIGHT_PWM_CH   1       // LEDC channel 1 for right motor

// ── Robot parameters ─────────────────────────────────────────────────────────
#define COUNTS_PER_REV   937   // 11 CPR × 21.3 gear ratio × 4 quadrature
#define MOTOR_MAX_RPM    259   //173   // 280 RPM × (7.4V / 12V)
#define CONTROL_HZ       50    // PID loop runs at 50 Hz
#define CONTROL_PERIOD_US (1000000UL / CONTROL_HZ)

// ── PID gains ────────────────────────────────────────────────────────────────
// Tune these once the robot is assembled and driving
#define KP  3.0   // Keep as it is - provides adequate initial kick for velocity control
#define KI  3.5   // increase from 0.8 -> 3.5 to reduce steady-state error 
#define KD  0.08  // increase from 0.05 -> 0.08 to reduce overshoot and oscillations  

// ── Encoder state (volatile — modified in ISR) ───────────────────────────────
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

// Last encoder B state for quadrature decoding
volatile int left_b_last  = 0;
volatile int right_b_last = 0;

// ── PID state ────────────────────────────────────────────────────────────────
float left_target_rpm  = 0.0;
float right_target_rpm = 0.0;

float left_integral  = 0.0;
float right_integral = 0.0;
float left_prev_err  = 0.0;
float right_prev_err = 0.0;

// ── Timing ───────────────────────────────────────────────────────────────────
unsigned long last_control_us = 0;
unsigned long last_cmd_ms     = 0;   // watchdog: stop if no command for 500ms

// Snapshot of ticks at last control cycle (for velocity calculation)
long left_ticks_prev  = 0;
long right_ticks_prev = 0;

// Ticks accumulated since last report to RPi4
long left_ticks_report  = 0;
long right_ticks_report = 0;
unsigned long last_report_us = 0;

// ── Encoder ISRs ─────────────────────────────────────────────────────────────
// Called on rising edge of ENC_A. Read ENC_B to determine direction.
void IRAM_ATTR left_encoder_isr() {
  if (digitalRead(LEFT_ENC_B) == LOW) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

void IRAM_ATTR right_encoder_isr() {
  if (digitalRead(RIGHT_ENC_B) == LOW) {
    right_ticks--;
  } else {
    right_ticks++;
  }
}

// ── Motor output ─────────────────────────────────────────────────────────────
// pwm: -1023 (full reverse) .. 0 (stop) .. +1023 (full forward)
void set_left_motor(int pwm) {
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) {
    digitalWrite(LEFT_IN_A, HIGH);
    digitalWrite(LEFT_IN_B, LOW);
    ledcWrite(LEFT_PWM_CH, pwm);
  } else {
    digitalWrite(LEFT_IN_A, LOW);
    digitalWrite(LEFT_IN_B, HIGH);
    ledcWrite(LEFT_PWM_CH, -pwm);
  }
}

void set_right_motor(int pwm) {
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) {
    digitalWrite(RIGHT_IN_A, HIGH);
    digitalWrite(RIGHT_IN_B, LOW);
    ledcWrite(RIGHT_PWM_CH, pwm);
  } else {
    digitalWrite(RIGHT_IN_A, LOW);
    digitalWrite(RIGHT_IN_B, HIGH);
    ledcWrite(RIGHT_PWM_CH, -pwm);
  }
}

void stop_motors() {
  set_left_motor(0);
  set_right_motor(0);
  left_integral  = 0.0;
  right_integral = 0.0;
  left_prev_err  = 0.0;
  right_prev_err = 0.0;
}

// ── PID step ─────────────────────────────────────────────────────────────────
// measured_rpm: actual wheel speed measured from encoders this cycle
// target_rpm:   desired wheel speed from RPi4 command
// Returns: PWM value to apply (-1023..+1023)
int pid_step(float target_rpm, float measured_rpm,
             float &integral, float &prev_err, float dt_s) {
  float err = target_rpm - measured_rpm;
  integral  += err * dt_s;
  integral   = constrain(integral, -500.0, 500.0);  // anti-windup
  float deriv = (err - prev_err) / dt_s;
  prev_err = err;

  // Scale: MOTOR_MAX_RPM corresponds to PWM = 1023
  float output = (KP * err + KI * integral + KD * deriv)
                 * (1023.0 / MOTOR_MAX_RPM);
  return (int)constrain(output, -1023.0, 1023.0);
}

// ── Serial command parser ─────────────────────────────────────────────────────
// Parses "W <left_rpm> <right_rpm>\n" from RPi4
void parse_command(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.charAt(0) == 'W') {
    // Wheel velocity command
    float l = 0.0, r = 0.0;
    int parsed = sscanf(line.c_str(), "W %f %f", &l, &r);
    if (parsed == 2) {
      left_target_rpm  = constrain(l, -(float)MOTOR_MAX_RPM, (float)MOTOR_MAX_RPM);
      right_target_rpm = constrain(r, -(float)MOTOR_MAX_RPM, (float)MOTOR_MAX_RPM);
      last_cmd_ms = millis();
    }
  } else if (line.charAt(0) == 'S') {
    // Stop command
    left_target_rpm  = 0.0;
    right_target_rpm = 0.0;
    stop_motors();
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);  // brief settle time

  // Motor direction pins
  pinMode(LEFT_IN_A,  OUTPUT);
  pinMode(LEFT_IN_B,  OUTPUT);
  pinMode(RIGHT_IN_A, OUTPUT);
  pinMode(RIGHT_IN_B, OUTPUT);

  // LEDC PWM setup
  ledcSetup(LEFT_PWM_CH,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM_PIN,  LEFT_PWM_CH);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CH);

  // Encoder pins
  pinMode(LEFT_ENC_A,  INPUT);
  pinMode(LEFT_ENC_B,  INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  // Attach encoder interrupts (RISING edge of channel A)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  left_encoder_isr,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), right_encoder_isr, RISING);

  stop_motors();

  last_control_us = micros();
  last_report_us  = micros();
  last_cmd_ms     = millis();

  Serial.println("ESP32 serial bridge ready");
}

// ── Loop ─────────────────────────────────────────────────────────────────────
String serial_buf = "";

void loop() {
  // ── Read serial commands from RPi4 ──
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parse_command(serial_buf);
      serial_buf = "";
    } else {
      serial_buf += c;
      if (serial_buf.length() > 64) serial_buf = "";  // overflow guard
    }
  }

  // ── Watchdog: stop motors if no command for 500ms ──
  if (millis() - last_cmd_ms > 500) {
    left_target_rpm  = 0.0;
    right_target_rpm = 0.0;
    stop_motors();
  }

  // ── 50 Hz PID control loop ──
  unsigned long now_us = micros();
  long dt_us = (long)(now_us - last_control_us);

  if (dt_us >= (long)CONTROL_PERIOD_US) {
    float dt_s = dt_us * 1e-6f;
    last_control_us = now_us;

    // Snapshot encoder counts atomically
    noInterrupts();
    long l_ticks = left_ticks;
    long r_ticks = right_ticks;
    interrupts();

    // Ticks since last control cycle → RPM
    long dl = l_ticks - left_ticks_prev;
    long dr = r_ticks - right_ticks_prev;
    left_ticks_prev  = l_ticks;
    right_ticks_prev = r_ticks;

    float left_rpm  = (dl / (float)COUNTS_PER_REV) / dt_s * 60.0f;
    float right_rpm = (dr / (float)COUNTS_PER_REV) / dt_s * 60.0f;

    // Accumulate ticks for reporting
    left_ticks_report  += dl;
    right_ticks_report += dr;

    // Run PID
    if (abs(left_target_rpm) < 0.5f && abs(right_target_rpm) < 0.5f) {
      stop_motors();
    } else {
      int left_pwm  = pid_step(left_target_rpm,  left_rpm,
                               left_integral,  left_prev_err,  dt_s);
      int right_pwm = pid_step(right_target_rpm, right_rpm,
                               right_integral, right_prev_err, dt_s);
      set_left_motor(left_pwm);
      set_right_motor(right_pwm);
    }
  }

  // ── Report encoder ticks to RPi4 at 20 Hz ──
  unsigned long now2_us = micros();
  long report_dt_us = (long)(now2_us - last_report_us);

  if (report_dt_us >= 50000L) {  // 50 ms = 20 Hz
    // Format: "E <left_ticks> <right_ticks> <dt_us>\n"
    //Serial.print("E ");
    //Serial.print(left_ticks_report);
    //Serial.print(" ");
    //Serial.print(right_ticks_report);
    //Serial.print(" ");
    //Serial.println(report_dt_us);
    Serial.print("E ");
    Serial.print(left_ticks_report);
    Serial.print(" ");
    Serial.print(right_ticks_report);
    Serial.print(" ");
    Serial.print(report_dt_us);
    Serial.print(" ");
    Serial.print(left_target_rpm);
    Serial.print(" ");
    Serial.println(right_target_rpm);

    left_ticks_report  = 0;
    right_ticks_report = 0;
    last_report_us = now2_us;
  }
}
