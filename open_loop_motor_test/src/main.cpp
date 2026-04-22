#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
// OPEN-LOOP DC MOTOR SPEED CONTROL
// Hardware: ESP32 DevKit C V2 + Pololu TB6612FNG + GA23-370 12V 280RPM
// ═══════════════════════════════════════════════════════════════

// ── Pin definitions ─────────────────────────────────────────────
#define STBY  25   // TB6612FNG Standby — HIGH = driver enabled
#define AIN1  26   // Motor A direction bit 1
#define AIN2  27   // Motor A direction bit 2
#define PWMA  14   // Motor A PWM speed signal
#define BIN1  33   // Motor B direction bit 1
#define BIN2  32   // Motor B direction bit 2
#define PWMB  15   // Motor B PWM speed signal

// ── PWM configuration ───────────────────────────────────────────
#define PWM_FREQ  20000  // 20 kHz — above human hearing, quiet operation
#define PWM_RES   8      // 8-bit → duty values 0 (0%) to 255 (100%)
#define CH_A      0      // LEDC hardware channel 0 → Motor A
#define CH_B      1      // LEDC hardware channel 1 → Motor B

// ── setup() runs ONCE on power-on or reset ──────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("=== Motor Open-Loop Test Starting ===");

  // Configure direction and standby pins as digital outputs
  // This ensures the physical pin is driven, not just the internal register
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Initialise PWM channels (ESP32 Arduino v2.x API)
  ledcSetup(CH_A, PWM_FREQ, PWM_RES);  // Channel 0: 20kHz, 8-bit
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);  // Channel 1: 20kHz, 8-bit
  ledcAttachPin(PWMA, CH_A);           // GPIO14 → Channel 0
  ledcAttachPin(PWMB, CH_B);           // GPIO15 → Channel 1

  // Enable the TB6612FNG driver
  // If STBY is LOW, no motor will move regardless of other signals
  digitalWrite(STBY, HIGH);

  Serial.println("Driver enabled. Starting test sequence...");
}

// ── driveMotor() — central control function ─────────────────────
// Parameters:
//   motorNum : 'A' or 'B'
//   speed    : integer from -255 (full reverse) to +255 (full forward)
//              0 = coast (motor spins freely, no braking)
// Approximate motor speed mapping (open-loop estimate):
//   speed=255 → ~12V average  → ~280 RPM (no-load, full speed)
//   speed=128 → ~6V average   → ~140 RPM
//   speed=64  → ~3V average   → ~70 RPM
//   speed=0   → 0V            → 0 RPM (coast)
void driveMotor(char motorNum, int speed) {
  int in1, in2, ch;

  // Select the correct set of pins for the requested motor
  if (motorNum == 'A') {
    in1 = AIN1; in2 = AIN2; ch = CH_A;
  } else {
    in1 = BIN1; in2 = BIN2; ch = CH_B;
  }

  // Clamp speed to valid range — prevents accidental overflow
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    // FORWARD: IN1=HIGH, IN2=LOW
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(ch, speed);       // Write duty to PWM channel

  } else if (speed < 0) {
    // REVERSE: IN1=LOW, IN2=HIGH, duty = absolute value
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(ch, -speed);      // -(-200) = 200, always positive

  } else {
    // COAST (stop): both direction pins LOW, PWM = 0
    // Motor spins down freely — no electrical braking
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
  }
}

// ── brakeMotor() — hard electrical stop ─────────────────────────
// IN1=HIGH, IN2=HIGH → TB6612FNG applies short-circuit brake
// Motor stops quickly. Use instead of driveMotor(x, 0) when fast stopping is needed.
void brakeMotor(char motorNum) {
  int in1, in2, ch;
  if (motorNum == 'A') { in1 = AIN1; in2 = AIN2; ch = CH_A; }
  else                  { in1 = BIN1; in2 = BIN2; ch = CH_B; }
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  ledcWrite(ch, 255);  // Full PWM with both HIGH = brake mode
}

// ── loop() runs FOREVER after setup() ───────────────────────────
void loop() {

  // ── Test 1: Full speed forward ─────────────────────────────
  Serial.println("[1] Both motors: 100% FORWARD (speed=255, ~12V, ~280 RPM)");
  driveMotor('A', 255);
  driveMotor('B', 255);
  delay(3000);  // Run for 3 seconds

  // ── Test 2: Coast to stop ──────────────────────────────────
  Serial.println("[2] COAST stop - motors spin down freely");
  driveMotor('A', 0);
  driveMotor('B', 0);
  delay(2000);

  // ── Test 3: Half speed forward ─────────────────────────────
  Serial.println("[3] Both motors: 50% FORWARD (speed=128, ~6V, ~140 RPM)");
  driveMotor('A', 128);
  driveMotor('B', 128);
  delay(3000);

  // ── Test 4: Hard brake ──────────────────────────────────────
  Serial.println("[4] HARD BRAKE -- immediate stop");
  brakeMotor('A');
  brakeMotor('B');
  delay(1000);

  // ── Test 5: Quarter speed reverse ──────────────────────────
  Serial.println("[5] Both motors: 25% REVERSE (speed=-64, ~3V, ~70 RPM)");
  driveMotor('A', -64);
  driveMotor('B', -64);
  delay(3000);

  // ── Test 6: Ramp up Motor A slowly ─────────────────────────
  // A for-loop counts spd from 0 up to 255 in steps of 5
  // Each step: set that speed, wait 50ms, then move to next step
  // Total: 51 steps × 50ms = 2.55 seconds to reach full speed
  Serial.println("[6] Motor A RAMP UP: 0 → 255 over 2.5 seconds");
  driveMotor('B', 0);
  for (int spd = 0; spd <= 255; spd += 5) {
    driveMotor('A', spd);
    Serial.print("  Ramp speed: "); Serial.println(spd);
    delay(50);  // 50ms per step × 51 steps = ~2.55 seconds total
  }

  // ── Test 7: Ramp down Motor A slowly ───────────────────────
  Serial.println("[7] Motor A RAMP DOWN: 255 → 0");
  for (int spd = 255; spd >= 0; spd -= 5) {
    driveMotor('A', spd);
    Serial.print("  Ramp speed: "); Serial.println(spd);
    delay(50);
  }

  // ── Final stop before repeating ────────────────────────────
  Serial.println("[8] All stop. Repeating in 3 seconds...\n");
  brakeMotor('A');
  brakeMotor('B');
  delay(3000);
}