/*  
 *  MPU6050 DEBUG
 *  
 *  Misst die Gyro und Accel Werte des MPU6050
 *  Nutzt den Madgwick Filter um diese in Rotation (Roll, Pitch, Yawn) umzuwandeln
 *
 *  WICHTIG: 
 *  Wir verwenden eine modifizierte Version dieser Library: github.com/arduino-libraries/MadgwickAHRS/
 *
 *  In der normalen Version ist BETA nicht public, d.h. eine dynamische Anpassung des Faktors ist nicht möglich.
 *  Deswegen muss in der HEADER Datei "/src/MadgwickAHRS.h" die Variable beta von private in public bewegt werden muss.
 */ 

/*
#include <MPU6050.h>
#include <MadgwickAHRS.h>   // Modifizierte public library
#include <SmoothData4D.h>   // Eigene private Library
#include <Wire.h>
#include <math.h>

// ==========================
// Konfiguration / Konstanten
// ==========================
MPU6050 mpu;
Madgwick filter;

const float SAMPLE_RATE = 100.0f;        // Hz
unsigned long microsPerReading;
unsigned long microsPrevious = 0;

// WICHTIG: Für andere Sensoren anpassen
// Wir debuggen zuerst mit MPU6050 mitd den Werten:
const float ACCEL_SENSITIVITY = 16384.0f; // LSB/g (+/-2g)
const float GYRO_SENSITIVITY  = 131.0f;   // LSB/(Grad°/s) (+/-250°/s)

// Motion / accel checks
#define MOTION_THRESHOLD_DEG 200.0f   // deg/s Schwelle für erkennung von Rotation
#define ACCEL_NORM_MIN 0.9f
#define ACCEL_NORM_MAX 1.5f
#define ACCEL_SENSE_TIME 100          // Nachlaufzeit in Samples 

// Beta - Madgwick: Dynamisch angepasst
const float BETA_MOTION = 0.02f;   // während Dynamik -> Gyro dominanter (kleineres beta)
const float BETA_NORM   = 0.08f;   // ruhig -> Accel kann schneller korrigieren

// Offset wird in calibrateGyro gesetzt
float gyroOffsetRawX = 0.0f;  
float gyroOffsetRawY = 0.0f;
float gyroOffsetRawZ = 0.0f;
const float BIAS_ALPHA = 0.0005f; // sehr langsam: schützt vor falscher Anpassung   // NORMAL: 0.0005f

// Print-Frequenz (reduziert I/O blocking)
const int PRINT_EVERY_N = 5; // Ausgabe nur jedes N-te Sample
int printCounter = 0;

// Laufvariable
int accelSense = 0;
bool adaptive = false;    // DEBUG

void setup() {
  Serial.begin(115200); // btw schneller als Port 9600, deswegen immer das verwenden
  Wire.begin();
  delay(100);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 nicht gefunden! Prüfe I2C-Verbindung.");
    while (1) delay(1000);
  }

  // DLPF & Rate passend zu 100Hz
  mpu.setDLPFMode(2);    // müsste 100 Hz Bandbreite haben
  mpu.setRate(9);        // sampleRate 100 Hz
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  // Madgwick initialisieren
  filter.begin(SAMPLE_RATE);

  // Timing
  microsPerReading = 1000000UL / (unsigned long)SAMPLE_RATE;
  microsPrevious = micros();

  Serial.println("MPU6050 initialisiert. Kalibriere Gyro... (ruhe das Board)");
  calibrateGyro();
  Serial.println("Kalibrierung fertig. Starte Messung.");
  Serial.println("Roll(°)\t| Pitch(°)\t| Yaw(°)");
}

void loop() {
  unsigned long microsNow = micros();

  // wenn zu lange verzögert (z.B. bei Hochfahren), synchronisiere
  if (microsNow - microsPrevious > microsPerReading * 8UL) {
    microsPrevious = microsNow;
  }

  if (microsNow - microsPrevious >= microsPerReading) {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

    // 2. Rohdaten umwandeln:
    // Roh -> physikalische Einheiten
    float ax = (float)ax_raw / ACCEL_SENSITIVITY; // in g
    float ay = (float)ay_raw / ACCEL_SENSITIVITY;
    float az = (float)az_raw / ACCEL_SENSITIVITY;

    // Grad°/s
    float gx_deg = ((float)gx_raw - gyroOffsetRawX) / GYRO_SENSITIVITY;
    float gy_deg = ((float)gy_raw - gyroOffsetRawY) / GYRO_SENSITIVITY;
    float gz_deg = ((float)gz_raw - gyroOffsetRawZ) / GYRO_SENSITIVITY;
    
    // 3. Adaptives Update von BETA:
    // adaptive Update (setzt filter.beta intern, prüft accel-norm)
    adaptiveUpdate(gx_deg, gy_deg, gz_deg, ax, ay, az);

    // 4. Adaptive Bias Aktualisierung:
    // TODO: adaptiven Bias aktualisieren, wenn Gerät still ist (sehr langsam)
    float gyroMag = sqrt(gx_deg*gx_deg + gy_deg*gy_deg + gz_deg*gz_deg);
    float accelNorm = sqrt(ax*ax + ay*ay + az*az);
    if (gyroMag < 3.0f && accelNorm > 0.98f && accelNorm < 1.02f) {
      // Gerät still: sehr langsame Anpassung in Raw-LSB Einheiten
      gyroOffsetRawX = (1.0f - BIAS_ALPHA) * gyroOffsetRawX + BIAS_ALPHA * (float)gx_raw;
      gyroOffsetRawY = (1.0f - BIAS_ALPHA) * gyroOffsetRawY + BIAS_ALPHA * (float)gy_raw;
      gyroOffsetRawZ = (1.0f - BIAS_ALPHA) * gyroOffsetRawZ + BIAS_ALPHA * (float)gz_raw;
    }

    // Ausgabe nur alle PRINT_EVERY_N Messungen (verringert Blocking) -> Serielle Kommunikation ist Arsch langsam
    if (++printCounter >= PRINT_EVERY_N) {
      printCounter = 0;
      // 5. Berechnung/ Ausgabe der Euler Winkel:
      float roll = filter.getRoll();
      float pitch = filter.getPitch();
      float yaw = filter.getYaw();
      printMeasurement(roll, pitch, yaw);
      Serial.print("|\t ");
      Serial.print(gyroMag);
      Serial.print("|\t ");
      Serial.print(adaptive);
      Serial.print("|\t ");
      Serial.print(accelNorm);
      Serial.println("");
    }

    // Timing-Update: addiere konstanten Step (anstatt aktuelle micros() -> verhindert Drift)
    microsPrevious += microsPerReading;
  }
}

// ===================================
// Kalibrierung: Gyro Offset Bestimmen
// ===================================
void calibrateGyro() {
  const int calibrationSamples = 500;
  long sumGX = 0, sumGY = 0, sumGZ = 0;
  for (int i = 0; i < calibrationSamples; ++i) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sumGX += gx;
    sumGY += gy;
    sumGZ += gz;
    delay(5); // ca. 2500 ms insgesamt
    if (i % 100 == 0) Serial.print(".");
  }
  Serial.println();
  gyroOffsetRawX = (float)sumGX / (float)calibrationSamples;
  gyroOffsetRawY = (float)sumGY / (float)calibrationSamples;
  gyroOffsetRawZ = (float)sumGZ / (float)calibrationSamples;

  Serial.print("Gyro Offsets (raw LSB) X:");
  Serial.print(gyroOffsetRawX, 2);
  Serial.print(" Y:");
  Serial.print(gyroOffsetRawY, 2);
  Serial.print(" Z:");
  Serial.println(gyroOffsetRawZ, 2);
}

// ==================================================
// Adaptive Update: Motion-Detection + Beta-Anpassung
// ==================================================
void adaptiveUpdate(float gx_deg, float gy_deg, float gz_deg, float ax, float ay, float az) {
  // gx_deg etc. sind in deg/s (Madgwick erwartet deg/s; Lib konvertiert intern zu rad/s)
  float gyroMag = sqrt(gx_deg * gx_deg + gy_deg * gy_deg + gz_deg * gz_deg);  // in deg/s
  float accelNorm = sqrt(ax * ax + ay * ay + az * az);                        // in g

  bool accelUnreliable = (accelNorm < ACCEL_NORM_MIN) || (accelNorm > ACCEL_NORM_MAX);
  bool gyroDynamic = (gyroMag > MOTION_THRESHOLD_DEG);

  if (gyroDynamic || accelUnreliable) {
    // Dynamik erkannt -> kleineres BETA (mehr Fokus auf Gyro)
    filter.beta = BETA_MOTION;
    adaptive = true;
    accelSense = ACCEL_SENSE_TIME;
  } else if (accelSense > 0) {
    // kurze Nachlaufphase 
    filter.beta = BETA_MOTION;
    adaptive = true;
    accelSense--;
  } else {
    // ruhig: mehr Fokus auf Accel
    // TODO: Smoothing einbauen!
    adaptive = false;
    filter.beta = BETA_NORM;
  }

  // Die Madgwick Lib normalisiert schon auf rad/s intern (Wir müssen das nich nochmal machen)
  filter.updateIMU(gx_deg, gy_deg, gz_deg, ax, ay, az);
}

// ================
// Ausgabe
// ================
void printMeasurement(float roll, float pitch, float yaw) {
  Serial.print(roll, 2);
  Serial.print("\t| ");
  Serial.print(pitch, 2);
  Serial.print("\t| ");
  Serial.print(yaw, 2);
}
*/