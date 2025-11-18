/*
 * Arduino Nano 33 BLE Sense Rev2 - Driftfreie Orientierungsbestimmung
 * 
 * Autor: Copilot (basierend auf Recherche)
 * Hardware: Arduino Nano 33 BLE Sense Rev2
 * Sensoren: BMI270 (IMU) + BMM150 (Magnetometer)
 * Algorithmus: Extended Kalman Filter (EKF) für 9-DOF Sensor Fusion
 * 
 * Output: Quaternionen (q0, q1, q2, q3) + Euler-Winkel (Pitch, Roll, Yaw)
 * 
 * === WICHTIG ===
 * Vor dem ersten Einsatz:
 * 1. Magnetometer-Kalibrierung durchführen (siehe Anleitung unten)
 * 2. Kalibrierungswerte in die Variablen MAG_OFFSET_X/Y/Z und MAG_SCALE_X/Y/Z eintragen
 * 3. Board beim Start flach hinlegen für Gyro/Accel-Kalibrierung
 */

#include <Arduino_BMI270_BMM150.h>

// ============================================================================
// KONFIGURATION - HIER KALIBRIERUNGSWERTE EINTRAGEN
// ============================================================================

// --- MAGNETOMETER HARD-IRON OFFSET (nach Kalibrierung) ---
// Standardwerte (ohne Kalibrierung) - MUSS ANGEPASST WERDEN!
float MAG_OFFSET_X =  21.092;  // µT 17.9, -4.26, 
float MAG_OFFSET_Y = -24.715;  // µT
float MAG_OFFSET_Z = -62.876;  // µT

// --- MAGNETOMETER SOFT-IRON SCALE MATRIX (3x3) ---
// Standardwerte (Einheitsmatrix = keine Korrektur) - MUSS ANGEPASST WERDEN!
float MAG_SCALE[3][3] = {
  {0.915, -0.001, -0.05},
  {0.000,  1.115,  0.006},
  {0.053,  0.000,  0.996}
};

// --- KALMAN FILTER TUNING PARAMETER ---
float Q_ANGLE = 0.001;    // Process noise für Quaternion (kleiner = glatter)
float Q_BIAS = 0.003;     // Process noise für Gyro-Bias
float R_ACCEL = 0.05;     // Measurement noise Accelerometer (größer = weniger Vertrauen)
float R_MAG = 0.08;       // Measurement noise Magnetometer (größer = weniger Vertrauen)

// --- ACHSEN-KORREKTUR FÜR BMM150 ---
// Für Nano 33 BLE Sense Rev2: Y und Z sind invertiert
#define MAG_AXIS_FLIP_Y   true
#define MAG_AXIS_FLIP_Z   true

// ============================================================================
// GLOBALE VARIABLEN
// ============================================================================

// Kalman State: [q0, q1, q2, q3, bias_x, bias_y, bias_z]
float q[4] = {1.0, 0.0, 0.0, 0.0};  // Quaternion (w, x, y, z)
float bias[3] = {0.0, 0.0, 0.0};    // Gyro Bias (rad/s)

// Kalman Covariance Matrix (7x7) - nur Diagonale für Effizienz
float P[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// Sensor-Offsets (Auto-Kalibrierung beim Start)
float gyro_offset[3] = {0, 0, 0};
float accel_offset[3] = {0, 0, 0};

// Timing
unsigned long lastUpdate = 0;
float dt = 0.01;  // Initial 10ms

// ============================================================================
// HILFSFUNKTIONEN
// ============================================================================

// Normalisierung eines Vektors
void normalize(float v[3]) {
  float norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if (norm > 0.0001) {
    v[0] /= norm;
    v[1] /= norm;
    v[2] /= norm;
  }
}

// Normalisierung eines Quaternions
void normalizeQuaternion(float q[4]) {
  float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm > 0.0001) {
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
  }
}

// DEBUG:
// Quaternion zu Euler (Pitch, Roll, Yaw in Grad)
void quaternionToEuler(float q[4], float* pitch, float* roll, float* yaw) {
  // Roll (x-axis rotation)
  float sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]);
  float cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
  *roll = atan2(sinr_cosp, cosr_cosp) * 180.0 / PI;

  // Pitch (y-axis rotation)
  float sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);
  if (abs(sinp) >= 1)
    *pitch = copysign(90.0, sinp); // Use 90 degrees if out of range
  else
    *pitch = asin(sinp) * 180.0 / PI;

  // Yaw (z-axis rotation)
  float siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
  float cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
  *yaw = atan2(siny_cosp, cosy_cosp) * 180.0 / PI;
}

// ============================================================================
// KALIBRIERUNG
// ============================================================================

void calibrateGyroAccel() {
  Serial.println("=== AUTO-KALIBRIERUNG ===");
  Serial.println("Bitte Board flach hinlegen und nicht bewegen!");
  delay(2000);
  
  const int samples = 200;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  
  Serial.print("Sammle ");
  Serial.print(samples);
  Serial.println(" Samples...");
  
  for (int i = 0; i < samples; i++) {
    float gx, gy, gz, ax, ay, az;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      gx_sum += gx; gy_sum += gy; gz_sum += gz;
    }
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      ax_sum += ax; ay_sum += ay; az_sum += az;
    }
    delay(10);
  }
  
  gyro_offset[0] = gx_sum / samples;
  gyro_offset[1] = gy_sum / samples;
  gyro_offset[2] = gz_sum / samples;
  
  // Accel: Nur X und Y kalibrieren (Z sollte ~1g sein)
  accel_offset[0] = ax_sum / samples;
  accel_offset[1] = ay_sum / samples;
  accel_offset[2] = (az_sum / samples) - 1.0;  // Schwerkraft = 1g
  
  Serial.println("Kalibrierung abgeschlossen!");
  Serial.print("Gyro Offset: ");
  Serial.print(gyro_offset[0]); Serial.print(", ");
  Serial.print(gyro_offset[1]); Serial.print(", ");
  Serial.println(gyro_offset[2]);
  Serial.print("Accel Offset: ");
  Serial.print(accel_offset[0]); Serial.print(", ");
  Serial.print(accel_offset[1]); Serial.print(", ");
  Serial.println(accel_offset[2]);
  Serial.println();
}

// ============================================================================
// EXTENDED KALMAN FILTER
// ============================================================================

// Prediction Step: Integriere Gyro-Daten
void kalmanPredict(float gx, float gy, float gz, float dt) {
  // Korrigiere Gyro mit Bias
  gx -= bias[0];
  gy -= bias[1];
  gz -= bias[2];
  
  // Quaternion-Kinematik: q_dot = 0.5 * q * omega
  float qDot[4];
  qDot[0] = 0.5 * (-q[1]*gx - q[2]*gy - q[3]*gz);
  qDot[1] = 0.5 * ( q[0]*gx + q[2]*gz - q[3]*gy);
  qDot[2] = 0.5 * ( q[0]*gy - q[1]*gz + q[3]*gx);
  qDot[3] = 0.5 * ( q[0]*gz + q[1]*gy - q[2]*gx);
  
  // Euler-Integration
  q[0] += qDot[0] * dt;
  q[1] += qDot[1] * dt;
  q[2] += qDot[2] * dt;
  q[3] += qDot[3] * dt;
  
  normalizeQuaternion(q);
  
  // Update Covariance (vereinfacht: nur Diagonale)
  for (int i = 0; i < 4; i++) P[i] += Q_ANGLE * dt;
  for (int i = 4; i < 7; i++) P[i] += Q_BIAS * dt;
}

// Correction Step mit Accelerometer
void kalmanUpdateAccel(float ax, float ay, float az) {
  // Normalisiere Accel-Vektor
  float accel[3] = {ax, ay, az};
  normalize(accel);
  
  // Erwartete Schwerkraft-Richtung aus Quaternion (body frame)
  float grav[3];
  grav[0] = 2.0 * (q[1]*q[3] - q[0]*q[2]);
  grav[1] = 2.0 * (q[0]*q[1] + q[2]*q[3]);
  grav[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  // Fehler (Innovation)
  float error[3];
  error[0] = accel[0] - grav[0];
  error[1] = accel[1] - grav[1];
  error[2] = accel[2] - grav[2];
  
  // Kalman Gain (vereinfacht)
  float K = P[0] / (P[0] + R_ACCEL);
  
  // Update Quaternion (Feedback von error in Quaternion-Space)
  // Vereinfachte Achsen-Rotation
  q[1] += K * error[0] * 0.5;
  q[2] += K * error[1] * 0.5;
  q[3] += K * error[2] * 0.5;
  normalizeQuaternion(q);
  
  // Update Covariance
  P[0] = (1.0 - K) * P[0];
  P[1] = (1.0 - K) * P[1];
  P[2] = (1.0 - K) * P[2];
  P[3] = (1.0 - K) * P[3];
}

// Correction Step mit Magnetometer
void kalmanUpdateMag(float mx, float my, float mz) {
  // Normalisiere Mag-Vektor
  float mag[3] = {mx, my, mz};
  normalize(mag);
  
  // Erwartete Nord-Richtung aus Quaternion (horizontal projection)
  // Rotiere Magnetfeld in Earth Frame
  float hx = 2.0*mag[0]*(0.5 - q[2]*q[2] - q[3]*q[3]) + 2.0*mag[1]*(q[1]*q[2] - q[0]*q[3]) + 2.0*mag[2]*(q[1]*q[3] + q[0]*q[2]);
  float hy = 2.0*mag[0]*(q[1]*q[2] + q[0]*q[3]) + 2.0*mag[1]*(0.5 - q[1]*q[1] - q[3]*q[3]) + 2.0*mag[2]*(q[2]*q[3] - q[0]*q[1]);
  
  float bx = sqrt(hx*hx + hy*hy);  // Horizontale Komponente
  
  // Erwartete Mag-Messung im Body Frame
  float mag_expected[3];
  mag_expected[0] = 2.0*bx*(0.5 - q[2]*q[2] - q[3]*q[3]);
  mag_expected[1] = 2.0*bx*(q[1]*q[2] - q[0]*q[3]);
  mag_expected[2] = 2.0*bx*(q[1]*q[3] + q[0]*q[2]);
  
  // Fehler
  float error[3];
  error[0] = mag[0] - mag_expected[0];
  error[1] = mag[1] - mag_expected[1];
  error[2] = mag[2] - mag_expected[2];
  
  // Kalman Gain
  float K = P[3] / (P[3] + R_MAG);
  
  // Update nur Yaw-relevante Quaternion-Komponenten
  q[0] -= K * error[2] * 0.3;
  q[3] += K * error[2] * 0.3;
  normalizeQuaternion(q);
  
  // Update Covariance
  P[3] = (1.0 - K) * P[3];
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("=== Arduino Nano 33 BLE Sense Rev2 ===");
  Serial.println("=== 9-DOF Quaternion Kalman Filter ===");
  Serial.println();
  
  // IMU initialisieren
  if (!IMU.begin()) {
    Serial.println("FEHLER: IMU konnte nicht initialisiert werden!");
    while (1);
  }
  
  Serial.print("BMI270 Gyro Sample Rate: ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  
  Serial.print("BMI270 Accel Sample Rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  
  Serial.print("BMM150 Mag Sample Rate: ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  
  // Auto-Kalibrierung
  calibrateGyroAccel();
  
  // Initialisierung abgeschlossen
  Serial.println("=== Kalman Approach v0.1 ===");
  Serial.println("Output: q0, q1, q2, q3, Pitch, Roll, Yaw");
  Serial.println();
  
  lastUpdate = micros();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Zeitdiff berechnen
  unsigned long now = micros();
  dt = (now - lastUpdate) / 1000000.0;
  lastUpdate = now;
  
  // Sensordaten
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  bool hasGyro = false, hasAccel = false, hasMag = false;
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);

    // 1. Offset
    // 2. In radian umrechnen
    gx = (gx - gyro_offset[0]) * DEG_TO_RAD;
    gy = (gy - gyro_offset[1]) * DEG_TO_RAD;
    gz = (gz - gyro_offset[2]) * DEG_TO_RAD;
    hasGyro = true;
  }
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);

    // Offset
    ax -= accel_offset[0];
    ay -= accel_offset[1];
    az -= accel_offset[2];
    hasAccel = true;
  }
  
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    
    // ACHSEN-KORREKTUR 
    if (MAG_AXIS_FLIP_Y) my = -my;    // Kalman erwartet my invertiert
    if (MAG_AXIS_FLIP_Z) mz = -mz;    // Einbau bedingt 
    
    // HARD-IRON KORREKTUR
    mx -= MAG_OFFSET_X;
    my -= MAG_OFFSET_Y;
    mz -= MAG_OFFSET_Z;
    
    // SOFT-IRON KORREKTUR (Matrixmultiplikation)
    float mx_cal = MAG_SCALE[0][0] * mx + MAG_SCALE[0][1] * my + MAG_SCALE[0][2] * mz;
    float my_cal = MAG_SCALE[1][0] * mx + MAG_SCALE[1][1] * my + MAG_SCALE[1][2] * mz;
    float mz_cal = MAG_SCALE[2][0] * mx + MAG_SCALE[2][1] * my + MAG_SCALE[2][2] * mz;
    
    mx = mx_cal;
    my = my_cal;
    mz = mz_cal;
    
    hasMag = true;
  }
  
  // Kalman Filter Update
  if (hasGyro) {
    kalmanPredict(gx, gy, gz, dt);
  }
  
  if (hasAccel) {
    kalmanUpdateAccel(ax, ay, az);
  }
  
  if (hasMag) {
    kalmanUpdateMag(mx, my, mz);
  }
  
  // Euler-Winkel berechnen
  float pitch, roll, yaw;
  quaternionToEuler(q, &pitch, &roll, &yaw);
  
  // Ausgabe über Serial (alle 20ms = 50 Hz)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 20) {
    lastPrint = millis();
    
    // Format: q0,q1,q2,q3,Pitch,Roll,Yaw
    Serial.print(q[0], 4); Serial.print(",");
    Serial.print(q[1], 4); Serial.print(",");
    Serial.print(q[2], 4); Serial.print(",");
    Serial.print(q[3], 4); Serial.print(",");
    Serial.print("\t ANGLES: ");
    Serial.print("P: ");
    Serial.print(pitch, 2); Serial.print(",\t R: ");
    Serial.print(roll, 2); Serial.print(",\t Y: ");
    Serial.println(yaw, 2);
  }
}