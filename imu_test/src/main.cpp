#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/*
  VISUALIZATION NOTE
  ------------------
  This version is almost the same as the earlier estimator, but the serial
  output is formatted as CSV (comma-separated values). That makes it easy to:

  1) watch data live in a PC helper script,
  2) save data to a CSV file,
  3) open the log later in Excel,
  4) import the log into plotting tools.

  The first printed line is a CSV header. Every next line is one data row.
*/

/*
  ================================================================
  Beginner-friendly Kalman estimator for ESP32 + MPU-6050
  ================================================================

  IMPORTANT HONEST NOTE:
  ----------------------
  This code can give you:
    1) clean roll and pitch angles,
    2) relative yaw angle (but yaw will drift over time),
    3) de-noised angular velocities,
    4) linear acceleration with gravity approximately removed,
    5) rough velocity and rough position by integrating acceleration.

  But this code CANNOT make position "correct and reliable" for long times
  when you only have one MPU-6050.

  Why?
  - The accelerometer contains noise and tiny offsets.
  - Position needs TWO integrations of acceleration.
  - Tiny errors become big drift.
  - The MPU-6050 has no magnetometer, so yaw (heading) has no absolute reference.

  So:
  - roll/pitch: useful
  - gyro rates: useful
  - yaw: relative only, drifts
  - velocity/position: experimental only, drifts quickly

  For a real 2WD robot, the practical next step is:
  - add wheel encoders for distance/speed,
  - optionally add a magnetometer for heading,
  - then fuse encoder + IMU in a 2D EKF.

  ----------------------------------------------------------------
  HOW THIS CODE WORKS IN SIMPLE WORDS
  ----------------------------------------------------------------
  1) Read accelerometer and gyroscope from MPU-6050.
  2) Use a Kalman filter for roll and pitch.
     - gyro tells fast changes but drifts
     - accelerometer tells gravity direction but is noisy
     - Kalman combines both
  3) Estimate gyro bias on X and Y inside the Kalman filters.
  4) Estimate gyro bias on Z using a simple startup calibration.
  5) Low-pass filter the angular rates to make them less noisy.
  6) Rotate body acceleration into the world frame.
  7) Subtract gravity.
  8) Integrate to velocity and position.
  9) Apply a simple "stationary detector" to clamp velocity to zero
     when the robot looks still. This helps a little, but it does NOT
     solve the fundamental drift problem.

  ----------------------------------------------------------------
  WIRING (typical ESP32 dev board)
  ----------------------------------------------------------------
  MPU-6050   ->   ESP32
  VCC        ->   3V3
  GND        ->   GND
  SDA        ->   GPIO 21   (default common I2C SDA)
  SCL        ->   GPIO 22   (default common I2C SCL)

  NOTE:
  If your board uses different I2C pins, change Wire.begin(SDA, SCL).

  ----------------------------------------------------------------
  AXIS NOTE
  ----------------------------------------------------------------
  The sign of roll/pitch formulas depends on how your MPU-6050 module is
  physically mounted on your robot.

  This code assumes a common "sensor flat, top face upward" mounting.
  If roll or pitch moves in the wrong direction, you may need to flip signs
  or swap axes.
*/

Adafruit_MPU6050 mpu;

// ---------------------------
// Constants
// ---------------------------
const float GRAVITY = 9.80665f;   // gravity in m/s^2
const float RAD_TO_DEG_F = 57.2957795f;
const float DEG_TO_RAD_F = 0.0174532925f;

// I2C pins for many ESP32 boards
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// ---------------------------
// Simple helper functions
// ---------------------------
float wrapAngle180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

float lowPass(float previousValue, float newValue, float alpha) {
  // alpha close to 1.0 = more smoothing, slower response
  return alpha * previousValue + (1.0f - alpha) * newValue;
}

// ---------------------------
// Small 1D Kalman filter for one angle and one gyro bias
// State = [angle, bias]
// ---------------------------
class KalmanAngle {
public:
  KalmanAngle() {
    angle = 0.0f;
    bias = 0.0f;
    rate = 0.0f;

    // Covariance matrix P
    P00 = 0.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 0.0f;

    // Tuning values
    // Q_angle: trust in angle model
    // Q_bias : how fast bias is allowed to change
    // R_measure: trust in accelerometer angle measurement
    Q_angle = 0.0015f;
    Q_bias = 0.0030f;
    R_measure = 0.03f;
  }

  void setAngle(float newAngle) {
    angle = newAngle;
  }

  float getAngle(float measuredAngle, float measuredRate, float dt) {
    // ---------------- PREDICTION STEP ----------------
    // Remove current bias estimate from gyro rate
    rate = measuredRate - bias;

    // Predict new angle by integrating angular rate
    angle += dt * rate;

    // Predict covariance matrix P
    P00 += dt * (dt * P11 - P01 - P10 + Q_angle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += Q_bias * dt;

    // ---------------- UPDATE STEP ----------------
    // Innovation = measurement - prediction
    float y = measuredAngle - angle;

    // Innovation covariance
    float S = P00 + R_measure;

    // Kalman gain
    float K0 = P00 / S;
    float K1 = P10 / S;

    // Correct state
    angle += K0 * y;
    bias  += K1 * y;

    // Correct covariance
    float P00_temp = P00;
    float P01_temp = P01;

    P00 -= K0 * P00_temp;
    P01 -= K0 * P01_temp;
    P10 -= K1 * P00_temp;
    P11 -= K1 * P01_temp;

    return angle;
  }

  float getRate() const {
    return rate;
  }

  float getBias() const {
    return bias;
  }

private:
  float angle;
  float bias;
  float rate;

  // Covariance matrix elements
  float P00, P01, P10, P11;

  // Noise tuning
  float Q_angle;
  float Q_bias;
  float R_measure;
};

KalmanAngle kalmanRoll;
KalmanAngle kalmanPitch;

// ---------------------------
// Global estimator variables
// ---------------------------
unsigned long lastMicros = 0;

// Startup gyro bias (found while sensor is still)
float gyroBiasZ = 0.0f;

// Filtered angular rates in rad/s
float gyroX_filtered = 0.0f;
float gyroY_filtered = 0.0f;
float gyroZ_filtered = 0.0f;

// Estimated orientation
float rollDeg  = 0.0f;
float pitchDeg = 0.0f;
float yawDeg   = 0.0f;   // relative yaw only, will drift

// World-frame linear acceleration (gravity removed)
float axWorld = 0.0f;
float ayWorld = 0.0f;
float azWorld = 0.0f;

// Velocity in world frame (m/s)
float vx = 0.0f;
float vy = 0.0f;
float vz = 0.0f;

// Position in world frame (m)
float px = 0.0f;
float py = 0.0f;
float pz = 0.0f;

// ---------------------------
// Gyro calibration at startup
// ---------------------------
void calibrateGyroAtRest() {
  Serial.println("\nGyro calibration: keep the sensor completely still...");

  const int samples = 1000;
  float sumGx = 0.0f;
  float sumGy = 0.0f;
  float sumGz = 0.0f;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumGx += g.gyro.x;
    sumGy += g.gyro.y;
    sumGz += g.gyro.z;

    delay(3);
  }

  // X and Y bias are also tracked by Kalman filters later,
  // but this startup average helps us begin from a better place.
  float gyroBiasX0 = sumGx / samples;
  float gyroBiasY0 = sumGy / samples;
  gyroBiasZ        = sumGz / samples;

  // Store startup angle bias guesses indirectly by setting initial filter bias.
  // We cannot write bias directly into the Kalman class here, so we simply print it.
  // The filter will adapt during operation.
  Serial.println("Calibration done.");
  Serial.print("Startup gyro bias X [rad/s]: "); Serial.println(gyroBiasX0, 6);
  Serial.print("Startup gyro bias Y [rad/s]: "); Serial.println(gyroBiasY0, 6);
  Serial.print("Startup gyro bias Z [rad/s]: "); Serial.println(gyroBiasZ, 6);
}

// ---------------------------
// Convert body acceleration to world acceleration
// ---------------------------
void bodyToWorldAndRemoveGravity(
  float axBody, float ayBody, float azBody,
  float rollRad, float pitchRad, float yawRad,
  float &axW, float &ayW, float &azW)
{
  /*
    Rotation matrix from body frame to world frame.
    We use roll, pitch, yaw in the common aerospace order.

    This lets us answer:
    "If the sensor says acceleration along its own x/y/z axes,
     what is that acceleration in the fixed world frame?"
  */

  float cr = cosf(rollRad);
  float sr = sinf(rollRad);
  float cp = cosf(pitchRad);
  float sp = sinf(pitchRad);
  float cy = cosf(yawRad);
  float sy = sinf(yawRad);

  // Body -> world rotation matrix
  float R11 = cy * cp;
  float R12 = cy * sp * sr - sy * cr;
  float R13 = cy * sp * cr + sy * sr;

  float R21 = sy * cp;
  float R22 = sy * sp * sr + cy * cr;
  float R23 = sy * sp * cr - cy * sr;

  float R31 = -sp;
  float R32 = cp * sr;
  float R33 = cp * cr;

  // Rotate acceleration into world frame
  axW = R11 * axBody + R12 * ayBody + R13 * azBody;
  ayW = R21 * axBody + R22 * ayBody + R23 * azBody;
  azW = R31 * axBody + R32 * ayBody + R33 * azBody;

  // Subtract gravity from world Z
  // If the sensor is standing still, world acceleration should become close to zero.
  azW -= GRAVITY;
}

// ---------------------------
// Detect if sensor is nearly stationary
// ---------------------------
bool isStationary(
  float axBody, float ayBody, float azBody,
  float gx, float gy, float gz)
{
  float accelNorm = sqrtf(axBody * axBody + ayBody * ayBody + azBody * azBody);
  float gyroNorm  = sqrtf(gx * gx + gy * gy + gz * gz);

  // Heuristic thresholds. These are not universal.
  // You may need to tune them for your robot.
  bool accelLooksStill = fabs(accelNorm - GRAVITY) < 0.15f;
  bool gyroLooksStill  = gyroNorm < 0.08f;

  return accelLooksStill && gyroLooksStill;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32 + MPU-6050 beginner Kalman estimator");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU-6050 not found. Check wiring.");
    while (1) {
      delay(1000);
    }
  }

  // Choose sensor ranges.
  // Smaller range = better resolution, but easier to saturate.
  // These are good starting values for a small robot.
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU-6050 found.");
  Serial.println("Keep the sensor flat and still for initialization...");
  delay(1500);

  // Read one sample to initialize roll/pitch from accelerometer
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Accelerometer-based tilt angles
  float rollAccDeg  = atan2f(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG_F;
  float pitchAccDeg = atan2f(-a.acceleration.x,
                             sqrtf(a.acceleration.y * a.acceleration.y +
                                   a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG_F;

  kalmanRoll.setAngle(rollAccDeg);
  kalmanPitch.setAngle(pitchAccDeg);

  rollDeg  = rollAccDeg;
  pitchDeg = pitchAccDeg;
  yawDeg   = 0.0f;

  calibrateGyroAtRest();

  lastMicros = micros();

  Serial.println("Initialization finished.");
  Serial.println("CSV stream starting...");
  Serial.println("time_s,roll_deg,pitch_deg,yaw_deg,gx_rad_s,gy_rad_s,gz_rad_s,ax_lin,ay_lin,az_lin,vx,vy,vz,px,py,pz,stationary");
}

void loop() {
  // ---------------------------
  // Time step dt
  // ---------------------------
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastMicros) * 1e-6f;
  lastMicros = nowMicros;

  // Protect against strange dt values
  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  // ---------------------------
  // Read sensor
  // ---------------------------
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Adafruit Unified Sensor gives:
  // acceleration in m/s^2
  // gyro in rad/s
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  // ---------------------------
  // Accelerometer-only angle estimate
  // (noisy, but not drifting when gravity is clean)
  // ---------------------------
  float rollAccDeg = atan2f(ay, az) * RAD_TO_DEG_F;
  float pitchAccDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;

  // ---------------------------
  // Kalman filter for roll and pitch
  // Use gyro rates converted to deg/s for the angle filters
  // ---------------------------
  float gxDeg = gx * RAD_TO_DEG_F;
  float gyDeg = gy * RAD_TO_DEG_F;

  rollDeg  = kalmanRoll.getAngle(rollAccDeg,  gxDeg, dt);
  pitchDeg = kalmanPitch.getAngle(pitchAccDeg, gyDeg, dt);

  // ---------------------------
  // De-noised angular velocity estimates
  // X and Y use Kalman bias correction from roll/pitch filters.
  // Z uses startup bias only, because yaw has no absolute correction sensor.
  // ---------------------------
  float gxCorrected = kalmanRoll.getRate() * DEG_TO_RAD_F;
  float gyCorrected = kalmanPitch.getRate() * DEG_TO_RAD_F;
  float gzCorrected = gz - gyroBiasZ;

  // Extra low-pass filtering to make rate output smoother
  gyroX_filtered = lowPass(gyroX_filtered, gxCorrected, 0.85f);
  gyroY_filtered = lowPass(gyroY_filtered, gyCorrected, 0.85f);
  gyroZ_filtered = lowPass(gyroZ_filtered, gzCorrected, 0.85f);

  // ---------------------------
  // Relative yaw estimation
  // IMPORTANT: this will drift because MPU-6050 has no magnetometer.
  // ---------------------------
  yawDeg += gyroZ_filtered * dt * RAD_TO_DEG_F;
  yawDeg = wrapAngle180(yawDeg);

  // ---------------------------
  // Compute world-frame linear acceleration
  // ---------------------------
  float rollRad  = rollDeg  * DEG_TO_RAD_F;
  float pitchRad = pitchDeg * DEG_TO_RAD_F;
  float yawRad   = yawDeg   * DEG_TO_RAD_F;

  float axW_raw, ayW_raw, azW_raw;
  bodyToWorldAndRemoveGravity(ax, ay, az, rollRad, pitchRad, yawRad,
                              axW_raw, ayW_raw, azW_raw);

  // Low-pass filter world acceleration to reduce noise
  axWorld = lowPass(axWorld, axW_raw, 0.70f);
  ayWorld = lowPass(ayWorld, ayW_raw, 0.70f);
  azWorld = lowPass(azWorld, azW_raw, 0.70f);

  // Deadband: remove tiny jitter around zero
  if (fabs(axWorld) < 0.05f) axWorld = 0.0f;
  if (fabs(ayWorld) < 0.05f) ayWorld = 0.0f;
  if (fabs(azWorld) < 0.05f) azWorld = 0.0f;

  // ---------------------------
  // Integrate acceleration -> velocity -> position
  // IMPORTANT: useful only for short demonstrations.
  // ---------------------------
  bool stationary = isStationary(ax, ay, az, gyroX_filtered, gyroY_filtered, gyroZ_filtered);

  if (stationary) {
    // Zero-velocity update (heuristic)
    vx = 0.0f;
    vy = 0.0f;
    vz = 0.0f;
  } else {
    vx += axWorld * dt;
    vy += ayWorld * dt;
    vz += azWorld * dt;
  }

  px += vx * dt;
  py += vy * dt;
  pz += vz * dt;

  // ---------------------------
  // Print results
  // ---------------------------
  // Units:
  // roll/pitch/yaw = degrees
  // angular rates   = rad/s
  // velocity        = m/s
  // position        = m
  // linear accel    = m/s^2

  static unsigned long lastPrintMs = 0;
  unsigned long nowMs = millis();

  if (nowMs - lastPrintMs >= 50) { // 20 Hz print rate
    lastPrintMs = nowMs;

    // Print one clean CSV row.
    // This is much easier for plotting tools and Python scripts to read.
    float timeSeconds = nowMs * 0.001f;

    Serial.print(timeSeconds, 3);    Serial.print(",");
    Serial.print(rollDeg, 2);        Serial.print(",");
    Serial.print(pitchDeg, 2);       Serial.print(",");
    Serial.print(yawDeg, 2);         Serial.print(",");

    Serial.print(gyroX_filtered, 4); Serial.print(",");
    Serial.print(gyroY_filtered, 4); Serial.print(",");
    Serial.print(gyroZ_filtered, 4); Serial.print(",");

    Serial.print(axWorld, 3);        Serial.print(",");
    Serial.print(ayWorld, 3);        Serial.print(",");
    Serial.print(azWorld, 3);        Serial.print(",");

    Serial.print(vx, 3);             Serial.print(",");
    Serial.print(vy, 3);             Serial.print(",");
    Serial.print(vz, 3);             Serial.print(",");

    Serial.print(px, 3);             Serial.print(",");
    Serial.print(py, 3);             Serial.print(",");
    Serial.print(pz, 3);             Serial.print(",");

    Serial.println(stationary ? 1 : 0);
  }
}
