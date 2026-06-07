/*
 * ESP32 Serial Bridge Firmware — Approach 1 (velocity-window edition)
 * --------------------------------------------------------------------
 * Keeps the proven 1x encoder counting (RISING edge of A only).
 * Fixes PID vibration by averaging velocity over 5 cycles (100 ms)
 * instead of 1 cycle (20 ms) — giving 10 ticks per measurement
 * instead of 2, which is enough for smooth, stable control.
 *
 * Changes vs previous uploaded version:
 *   COUNTS_PER_REV : 500 → 204  (correct calibrated value)
 *   KP / KI / KD   : 3.0/3.5/0.08 → 2.0/1.5/0.05
 *   Velocity window : 1 cycle → 5 cycles (100 ms average)
 *   ISRs            : unchanged — proven 1× RISING-edge counting
 *   METRES_PER_TICK in serial_bridge_node.py must be 1.0 / 1001.0
 */

#include <Arduino.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define LEFT_PWM_PIN   25
#define LEFT_IN_A      27
#define LEFT_IN_B      26

#define RIGHT_PWM_PIN  19
#define RIGHT_IN_A     5
#define RIGHT_IN_B     18

#define LEFT_ENC_A     34
#define LEFT_ENC_B     35
#define RIGHT_ENC_A    32
#define RIGHT_ENC_B    39

// ── PWM / LEDC ───────────────────────────────────────────────────────────────
#define PWM_FREQ        20000
#define PWM_RESOLUTION  10
#define LEFT_PWM_CH     0
#define RIGHT_PWM_CH    1

// ── Robot parameters ─────────────────────────────────────────────────────────
<<<<<<< HEAD
// Nominal value COUNTS_PER_REV: 937 -> 11 CPR × 21.3 gear ratio × 4 quadrature, but not fitting the observed data.
// Derived from calibration: 1001 ticks/m × π × 0.065 m/rev = 204 ticks/rev. 
// The formula is: COUNTS_PER_REV = (TICKS_PER_METER × WHEEL_CIRCUMFERENCE) / GEAR_RATIO, where TICKS_PER_METER is 1001, WHEEL_CIRCUMFERENCE is π × 0.065 m, and GEAR_RATIO is 21.3.
// 4× quadrature: 204 ticks/rev (1× calibrated) × 4 = 816 ticks/rev
// Calibration reminder: 1001 ticks/m at 1× → 4004 ticks/m at 4×
// METRES_PER_TICK in serial_bridge_node.py must be set to 1.0 / 4004.0
#define COUNTS_PER_REV   816
#define MOTOR_MAX_RPM    259   //173   // 280 RPM × (7.4V / 12V)
#define CONTROL_HZ       50    // PID loop runs at 50 Hz
#define CONTROL_PERIOD_US (1000000UL / CONTROL_HZ)

// ── PID gains ────────────────────────────────────────────────────────────────
// Tune these once the robot is assembled and driving
#define KP  1.5   // was 3.0 - provides adequate initial kick for velocity control
#define KI  1.0   // increase from 0.8 -> 3.5 to reduce steady-state error, now try 1.0 
#define KD  0.05  // increase from 0.05 -> 0.08 to reduce overshoot and oscillations, now set to 0.05 agian  

// ── Encoder direction signs ──────────────────────────────────────────────────
// Set to +1 or -1. Flip a sign if that wheel counts backward when rolling forward.
// Verify with: push robot 1 m forward → both printed tick totals should be positive.
#define LEFT_ENC_SIGN  -1
#define RIGHT_ENC_SIGN +1

// ── 4× quadrature state-transition table ─────────────────────────────────────
// index = (previous_state << 2) | current_state
// state encoding: (A_pin << 1) | B_pin
// Gray-code sequence forward:  00→01→11→10→00  (+1 each step)
// Gray-code sequence backward: 00→10→11→01→00  (-1 each step)
const int8_t QUAD_TABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

// ── Encoder state (volatile — modified in ISR) ───────────────────────────────
volatile long    left_ticks  = 0;
volatile long    right_ticks = 0;
volatile uint8_t left_state  = 0;   // last known (A<<1)|B state for left
volatile uint8_t right_state = 0;   // last known (A<<1)|B state for right
=======
// Calibrated from 1 m push test: 1001 ticks/m with 1× counting
//   COUNTS_PER_REV = 1001 × π × 0.065 = 204 ticks/wheel-revolution
#define COUNTS_PER_REV    204
#define MOTOR_MAX_RPM     259    // 280 RPM × (11.1 V / 12 V)
#define CONTROL_HZ        50
#define CONTROL_PERIOD_US (1000000UL / CONTROL_HZ)

// ── PID gains ────────────────────────────────────────────────────────────────
#define KP   2.0
#define KI   1.5
#define KD   0.05

// ── Velocity averaging window ─────────────────────────────────────────────────
// Average RPM over the last VEL_WINDOW PID cycles.
// At 50 Hz with VEL_WINDOW=5: 100 ms window → ~10 ticks per measurement
// instead of ~2 ticks per single cycle → smooth PID without changing ISR.
#define VEL_WINDOW 5

// ── Encoder state ────────────────────────────────────────────────────────────
volatile long left_ticks  = 0;
volatile long right_ticks = 0;
>>>>>>> fa4ba8b (update of CPR and PID gains (now robot runs smoothly))

// ── PID state ────────────────────────────────────────────────────────────────
float left_target_rpm  = 0.0;
float right_target_rpm = 0.0;

float left_integral  = 0.0;
float right_integral = 0.0;
float left_prev_err  = 0.0;
float right_prev_err = 0.0;

// ── Velocity window buffers ───────────────────────────────────────────────────
long  left_dl_buf[VEL_WINDOW]  = {0};
long  right_dl_buf[VEL_WINDOW] = {0};
float left_dt_buf[VEL_WINDOW]  = {0.0f};
float right_dt_buf[VEL_WINDOW] = {0.0f};
int   vel_buf_idx = 0;

// ── Timing ───────────────────────────────────────────────────────────────────
unsigned long last_control_us = 0;
unsigned long last_cmd_ms     = 0;
long left_ticks_prev  = 0;
long right_ticks_prev = 0;
long left_ticks_report  = 0;
long right_ticks_report = 0;
unsigned long last_report_us = 0;

<<<<<<< HEAD
// ── Encoder ISRs — 4× quadrature ─────────────────────────────────────────────
// Each ISR fires on ANY edge of EITHER channel for that motor.
// The state-transition table determines direction from the AB phase pattern.
=======
// ── Encoder ISRs — proven 1× counting, RISING edge of A ──────────────────────
// These ISRs were confirmed correct by the 1 m push test (L=1009, R=992).
// Left:  B=LOW  on A-rise → forward (+1)
// Right: B=HIGH on A-rise → forward (+1)   [direction inverted vs left]
>>>>>>> fa4ba8b (update of CPR and PID gains (now robot runs smoothly))
void IRAM_ATTR left_encoder_isr() {
  uint8_t a    = digitalRead(LEFT_ENC_A);
  uint8_t b    = digitalRead(LEFT_ENC_B);
  uint8_t curr = (a << 1) | b;
  left_ticks  += LEFT_ENC_SIGN * QUAD_TABLE[(left_state << 2) | curr];
  left_state   = curr;
}

void IRAM_ATTR right_encoder_isr() {
  uint8_t a    = digitalRead(RIGHT_ENC_A);
  uint8_t b    = digitalRead(RIGHT_ENC_B);
  uint8_t curr = (a << 1) | b;
  right_ticks += RIGHT_ENC_SIGN * QUAD_TABLE[(right_state << 2) | curr];
  right_state  = curr;
}

// ── Motor output ─────────────────────────────────────────────────────────────
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
int pid_step(float target_rpm, float measured_rpm,
             float &integral, float &prev_err, float dt_s) {
  float err  = target_rpm - measured_rpm;
  integral  += err * dt_s;
  integral   = constrain(integral, -500.0, 500.0);
  float deriv = (err - prev_err) / dt_s;
  prev_err = err;
  float output = (KP * err + KI * integral + KD * deriv)
                 * (1023.0 / MOTOR_MAX_RPM);
  return (int)constrain(output, -1023.0, 1023.0);
}

// ── Windowed RPM calculation ──────────────────────────────────────────────────
// Sum ticks and time over the last VEL_WINDOW cycles for a smooth estimate.
float windowed_rpm(long* dl_buf, float* dt_buf) {
  long  total_ticks = 0;
  float total_dt    = 0.0f;
  for (int i = 0; i < VEL_WINDOW; i++) {
    total_ticks += dl_buf[i];
    total_dt    += dt_buf[i];
  }
  if (total_dt < 1e-6f) return 0.0f;
  return (total_ticks / (float)COUNTS_PER_REV) / total_dt * 60.0f;
}

// ── Serial command parser ─────────────────────────────────────────────────────
void parse_command(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line.charAt(0) == 'W') {
    float l = 0.0, r = 0.0;
    if (sscanf(line.c_str(), "W %f %f", &l, &r) == 2) {
      left_target_rpm  = constrain(l, -(float)MOTOR_MAX_RPM, (float)MOTOR_MAX_RPM);
      right_target_rpm = constrain(r, -(float)MOTOR_MAX_RPM, (float)MOTOR_MAX_RPM);
      last_cmd_ms = millis();
    }
  } else if (line.charAt(0) == 'S') {
    left_target_rpm  = 0.0;
    right_target_rpm = 0.0;
    stop_motors();
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LEFT_IN_A,  OUTPUT);
  pinMode(LEFT_IN_B,  OUTPUT);
  pinMode(RIGHT_IN_A, OUTPUT);
  pinMode(RIGHT_IN_B, OUTPUT);

  ledcSetup(LEFT_PWM_CH,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM_PIN,  LEFT_PWM_CH);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CH);

  pinMode(LEFT_ENC_A,  INPUT);
  pinMode(LEFT_ENC_B,  INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

<<<<<<< HEAD
  // Initialise encoder states from current pin levels before attaching interrupts
  left_state  = (digitalRead(LEFT_ENC_A)  << 1) | digitalRead(LEFT_ENC_B);
  right_state = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);

  // 4× quadrature: attach BOTH channels on CHANGE (both rising and falling edges)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  left_encoder_isr,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  left_encoder_isr,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), right_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), right_encoder_isr, CHANGE);
=======
  // 1× counting — proven by calibration measurement
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  left_encoder_isr,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), right_encoder_isr, RISING);
>>>>>>> fa4ba8b (update of CPR and PID gains (now robot runs smoothly))

  stop_motors();

  last_control_us = micros();
  last_report_us  = micros();
  last_cmd_ms     = millis();

<<<<<<< HEAD
  Serial.println("ESP32 serial bridge ready (4x quadrature)");
=======
  Serial.println("ESP32 serial bridge ready (velocity-window)");
>>>>>>> fa4ba8b (update of CPR and PID gains (now robot runs smoothly))
}

// ── Loop ─────────────────────────────────────────────────────────────────────
String serial_buf = "";

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parse_command(serial_buf);
      serial_buf = "";
    } else {
      serial_buf += c;
      if (serial_buf.length() > 64) serial_buf = "";
    }
  }

  if (millis() - last_cmd_ms > 500) {
    left_target_rpm  = 0.0;
    right_target_rpm = 0.0;
    stop_motors();
  }

  // ── 50 Hz PID loop with 5-cycle velocity window ──
  unsigned long now_us = micros();
  long dt_us = (long)(now_us - last_control_us);

  if (dt_us >= (long)CONTROL_PERIOD_US) {
    float dt_s = dt_us * 1e-6f;
    last_control_us = now_us;

    noInterrupts();
    long l_ticks = left_ticks;
    long r_ticks = right_ticks;
    interrupts();

    long dl = l_ticks - left_ticks_prev;
    long dr = r_ticks - right_ticks_prev;
    left_ticks_prev  = l_ticks;
    right_ticks_prev = r_ticks;

    // Store this cycle in the window buffer
    left_dl_buf[vel_buf_idx]  = dl;
    right_dl_buf[vel_buf_idx] = dr;
    left_dt_buf[vel_buf_idx]  = dt_s;
    right_dt_buf[vel_buf_idx] = dt_s;
    vel_buf_idx = (vel_buf_idx + 1) % VEL_WINDOW;

    // Compute smoothed RPM from the full window
    float left_rpm  = windowed_rpm(left_dl_buf,  left_dt_buf);
    float right_rpm = windowed_rpm(right_dl_buf, right_dt_buf);

    // Accumulate raw ticks for odometry report (unchanged)
    left_ticks_report  += dl;
    right_ticks_report += dr;

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

  // ── Report to RPi4 at 20 Hz ──
  unsigned long now2_us = micros();
  long report_dt_us = (long)(now2_us - last_report_us);

<<<<<<< HEAD
  if (report_dt_us >= 50000L) {  // 50 ms = 20 Hz
    // Format: "E <left_ticks> <right_ticks> <dt_us> <left_target_rpm> <right_target_rpm>\n"
=======
  if (report_dt_us >= 50000L) {
>>>>>>> fa4ba8b (update of CPR and PID gains (now robot runs smoothly))
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
