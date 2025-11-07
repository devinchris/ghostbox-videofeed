/*  
 *  Arduino Nano 33 BLE Sense Rev2 - 9-DOF IMU DEBUG
 *  
 *  Misst die Gyro, Accel und Magnetometer Werte des eingebauten BMI270/BMM150 IMU Sensors
 *  Nutzt den Madgwick Filter um die Quaternionen zu berechnen
 * 
 *  TODO: QUATERNIONEN direkt ausgeben anstatt EULER-WINKEL!    -> Verhindert Gimball-Lock
 *
 *  Hardware: Arduino Nano 33 BLE Sense Rev2
 *  IMU: BMI270 (6-axis: Accel + Gyro) & BMM150 (3-axis: Magnetometer)
 *  
 *  Sensor-Konfiguration (durch die Library):
 *  - Accelerometer: ±4 g, Auflösung 0.122 mg
 *  - Gyroscope: ±2000 dps, Auflösung 70 mdps
 *  - Magnetometer: ±1300 uT, Auflösung 0.3 uT
 *  - Accel/Gyro Output Data Rate: 99.84 Hz
 *  - Magnetometer Output Data Rate: 10 Hz
 *    -> https://docs.arduino.cc/libraries/arduino_bmi270_bmm150/
 * 
 *  9-DOF Modus:
 *  Der Madgwick Filter verwendet alle 9 Achsen (Gyro + Accel + Mag) für bessere
 *  Yaw-Genauigkeit und absolute Orientierung relativ zum magnetischen Nordpol.
 *
 *  WICHTIG: 
 *  Wir verwenden eine modifizierte Version dieser Library: github.com/arduino-libraries/MadgwickAHRS/
 *
 *  Die Library wurde angepasst, damit Beta dynamisch gesetzt werden kann.
 *  Die Quaternionen können zusätlich direkt ausgelesen werden, anstatt auf Euler-Winkel zu beschränken.
 *  Das verhindert den Gimball-Lock
 */ 


#include "Arduino_BMI270_BMM150.h"
#include <MadgwickAHRS.h>
#include <math.h>

// ==========================
// Konfiguration / Konstanten
// ==========================
Madgwick filter;

const float SAMPLE_RATE = 99.84f;        // Hz (laut Docs)
unsigned long microsPerReading;
unsigned long microsPrevious = 0;

// WICHTIG: BMI270/BMM150 gibt bereits physikalische Einheiten zurück
// Accel in g, Gyro in deg/s, Magnetometer in uT (keine manuelle Konvertierung nötig)

// Sensor Spezifikationen:
// - Accelerometer: ±4 g, Auflösung 0.122 mg
// - Gyroscope: ±2000 dps, Auflösung 70 mdps
// - Magnetometer: ±1300 uT, Auflösung 0.3 uT
// - Accel/Gyro Output Data Rate: 99.84 Hz
// - Magnetometer Output Data Rate: 10 Hz (langsamer!)

// Motion / accel checks
#define MOTION_THRESHOLD_DEG 200.0f   // deg/s Schwelle für erkennung von Rotation
#define ACCEL_NORM_MIN 0.9f
#define ACCEL_NORM_MAX 1.5f
#define ACCEL_SENSE_TIME 100          // Nachlaufzeit in Samples 

// Magnetometer Kalibrierung (Hard-Iron Offset)
// TODO: Diese Werte müssen durch Kalibrierung ermittelt werden!
// Drehe das Board in alle Richtungen und notiere Min/Max Werte
float magOffsetX = 0.0f;
float magOffsetY = 0.0f;
float magOffsetZ = 0.0f;

// Magnetometer Scaling (Soft-Iron Kompensation)
// TODO: Optional für bessere Genauigkeit
float magScaleX = 1.0f;
float magScaleY = 1.0f;
float magScaleZ = 1.0f;

// Beta - Madgwick: Dynamisch angepasst
// WICHTIG: Mit Magnetometer kann Beta etwas höher sein für schnellere Konvergenz
const float BETA_MOTION = 0.03f;   // während Dynamik -> Gyro dominanter (kleineres beta)
const float BETA_NORM   = 0.1f;    // ruhig -> Accel+Mag kann schneller korrigieren (höher als bei 6-DOF)

// Offset wird in calibrateGyro gesetzt
float gyroOffsetX = 0.0f;  
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;
const float BIAS_ALPHA = 0.0005f; // sehr langsam: schützt vor falscher Anpassung

// Print-Frequenz (reduziert I/O blocking)
const int PRINT_EVERY_N = 5; // Ausgabe nur jedes N-te Sample
int printCounter = 0;

// Laufvariable
int accelSense = 0;
bool adaptive = false;    // DEBUG

// Magnetometer Daten-Buffer (wegen langsamerer Update-Rate)
float lastMagX = 0.0f;
float lastMagY = 0.0f;
float lastMagZ = 0.0f;
bool magDataValid = false;

void setup() {
  Serial.begin(115200); // btw schneller als Port 9600, deswegen immer das verwenden
  delay(100);

  // BMI270/BMM150 IMU initialisieren
  if (!IMU.begin()) {
    Serial.println("BMI270/BMM150 IMU nicht gefunden! Prüfe Board-Verbindung.");
    while (1) delay(1000);
  }

  // Madgwick initialisieren
  filter.begin(SAMPLE_RATE);

  // Timing (basierend auf 99.84 Hz)
  microsPerReading = 1000000UL / (unsigned long)SAMPLE_RATE;
  microsPrevious = micros();

  Serial.println("BMI270/BMM150 IMU initialisiert. Kalibriere Gyro... (ruhe das Board)");
  calibrateGyro();
  Serial.println("Gyro Kalibrierung fertig.");
  
  // Warte auf erste Magnetometer-Daten
  Serial.println("Warte auf Magnetometer-Daten...");
  while (!IMU.magneticFieldAvailable()) {
    delay(10);
  }

  Serial.println("Kalibrierung des Magnetometer...");
  Serial.println("Bewege den Sensor in alle Richtungen; FIGURE-8-BEWEGUNG");
  delay(200);
  Serial.println("Kalibrierung start.");
  calibrateMagnetometer();
  
  IMU.readMagneticField(lastMagX, lastMagY, lastMagZ);
  magDataValid = true;
  Serial.println("Magnetometer bereit.");
  
  Serial.println("Starte 9-DOF Messung.");
  Serial.println("Roll(°)\t| Pitch(°)\t| Yaw(°)\t| GyroMag\t| Adapt\t| AccNorm\t| MagNorm");
}

void loop() {
  unsigned long microsNow = micros();

  // wenn zu lange verzögert (z.B. bei Hochfahren), synchronisiere
  if (microsNow - microsPrevious > microsPerReading * 8UL) {
    microsPrevious = microsNow;
  }

  if (microsNow - microsPrevious >= microsPerReading) {
    float ax, ay, az;
    float gx_deg, gy_deg, gz_deg;
    float mx, my, mz;

    // Prüfe ob Accel und Gyro Daten verfügbar sind (99.84 Hz)
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      // Lese Sensordaten (bereits in physikalischen Einheiten: g und deg/s)
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx_deg, gy_deg, gz_deg);
      
      // Magnetometer hat nur 10 Hz Update-Rate -> nur wenn verfügbar lesen
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
        
        // Hard-Iron Offset Korrektur
        mx -= magOffsetX;
        my -= magOffsetY;
        mz -= magOffsetZ;
        
        // Soft-Iron Scaling (optional)
        mx *= magScaleX;
        my *= magScaleY;
        mz *= magScaleZ;
        
        // Speichere für nächste Iteration
        lastMagX = mx;
        lastMagY = my;
        lastMagZ = mz;
        magDataValid = true;
      } else {
        // Verwende letzte bekannte Magnetometer-Werte
        mx = lastMagX;
        my = lastMagY;
        mz = lastMagZ;
      }

      // Gyro-Offset abziehen
      gx_deg -= gyroOffsetX;
      gy_deg -= gyroOffsetY;
      gz_deg -= gyroOffsetZ;
      
      // 3. Adaptives Update von BETA und Madgwick Filter:
      // adaptive Update (setzt filter.beta intern, prüft accel-norm, verwendet Magnetometer)
      if (magDataValid) {
        adaptiveUpdate(gx_deg, gy_deg, gz_deg, ax, ay, az, mx, my, mz);
      }

      // 4. Adaptive Bias Aktualisierung:
      // Aktualisiere Bias nur wenn Gerät still ist (sehr langsam)
      float gyroMag = sqrt(gx_deg*gx_deg + gy_deg*gy_deg + gz_deg*gz_deg);
      float accelNorm = sqrt(ax*ax + ay*ay + az*az);
      float magNorm = sqrt(mx*mx + my*my + mz*mz);
      
      if (gyroMag < 3.0f && accelNorm > 0.98f && accelNorm < 1.02f) {
        // Gerät still: sehr langsame Anpassung in deg/s Einheiten
        gyroOffsetX = (1.0f - BIAS_ALPHA) * gyroOffsetX + BIAS_ALPHA * gx_deg;
        gyroOffsetY = (1.0f - BIAS_ALPHA) * gyroOffsetY + BIAS_ALPHA * gy_deg;
        gyroOffsetZ = (1.0f - BIAS_ALPHA) * gyroOffsetZ + BIAS_ALPHA * gz_deg;
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
        Serial.print(gyroMag, 1);
        Serial.print("\t| ");
        Serial.print(adaptive);
        Serial.print("\t| ");
        Serial.print(accelNorm, 2);
        Serial.print("\t| ");
        Serial.print(magNorm, 1);
        Serial.println("");
      }
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
  float sumGX = 0.0f, sumGY = 0.0f, sumGZ = 0.0f;
  int validSamples = 0;
  
  for (int i = 0; i < calibrationSamples; ++i) {
    if (IMU.gyroscopeAvailable()) {
      float gx, gy, gz;
      IMU.readGyroscope(gx, gy, gz);
      sumGX += gx;
      sumGY += gy;
      sumGZ += gz;
      validSamples++;
      if (i % 100 == 0) Serial.print(".");
    }
    delay(4);
  }
  Serial.println();
  
  if (validSamples > 0) {
    gyroOffsetX = sumGX / (float)validSamples;
    gyroOffsetY = sumGY / (float)validSamples;
    gyroOffsetZ = sumGZ / (float)validSamples;

    Serial.print("Gyro Offsets (deg/s) X:");
    Serial.print(gyroOffsetX, 2);
    Serial.print(" Y:");
    Serial.print(gyroOffsetY, 2);
    Serial.print(" Z:");
    Serial.println(gyroOffsetZ, 2);
  } else {
    Serial.println("Warnung: Keine gültigen Gyro-Messungen während Kalibrierung!");
  }
}

// ====================================================
// Magnetometer Kalibrierung (Hard-Iron & Soft-Iron)
// ====================================================
void calibrateMagnetometer() {
  Serial.println("Starte Magnetometer Kalibrierung...");
  Serial.println("Drehe das Board langsam in alle Richtungen (30 Sekunden)");
  
  float minX = 1000.0f, maxX = -1000.0f;
  float minY = 1000.0f, maxY = -1000.0f;
  float minZ = 1000.0f, maxZ = -1000.0f;
  
  unsigned long startTime = millis();
  const unsigned long calibrationTime = 30000; // 30 Sekunden empfohlen, testweise mit 10!
  const unsigned long shortCalibrationTime = 10000; // 10 Sekunden für schnelle Tests

  while (millis() - startTime < shortCalibrationTime) {
    if (IMU.magneticFieldAvailable()) {
      float mx, my, mz;
      IMU.readMagneticField(mx, my, mz);
      
      // Finde Min/Max Werte
      if (mx < minX) minX = mx;
      if (mx > maxX) maxX = mx;
      if (my < minY) minY = my;
      if (my > maxY) maxY = my;
      if (mz < minZ) minZ = mz;
      if (mz > maxZ) maxZ = mz;
      
      // Fortschritt anzeigen
      if ((millis() - startTime) % 3000 < 50) {
        Serial.print(".");
      }
    }
    delay(10);
  }
  
  Serial.println();
  Serial.println("Kalibrierung abgeschlossen!");
  
  // Berechne Hard-Iron Offsets (Mittelpunkt)
  magOffsetX = (maxX + minX) / 2.0f;
  magOffsetY = (maxY + minY) / 2.0f;
  magOffsetZ = (maxZ + minZ) / 2.0f;
  
  // Berechne Soft-Iron Scaling (Ellipsoid -> Sphäre)
  float avgDelta = ((maxX - minX) + (maxY - minY) + (maxZ - minZ)) / 3.0f;
  magScaleX = avgDelta / (maxX - minX);
  magScaleY = avgDelta / (maxY - minY);
  magScaleZ = avgDelta / (maxZ - minZ);
  
  Serial.println("Trage diese Werte im Code ein:");
  Serial.print("magOffsetX = "); Serial.print(magOffsetX, 2); Serial.println("f;");
  Serial.print("magOffsetY = "); Serial.print(magOffsetY, 2); Serial.println("f;");
  Serial.print("magOffsetZ = "); Serial.print(magOffsetZ, 2); Serial.println("f;");
  Serial.print("magScaleX = "); Serial.print(magScaleX, 3); Serial.println("f;");
  Serial.print("magScaleY = "); Serial.print(magScaleY, 3); Serial.println("f;");
  Serial.print("magScaleZ = "); Serial.print(magScaleZ, 3); Serial.println("f;");
}

// ==================================================================
// Adaptive Update: Motion-Erkennung + Beta-Anpassung (9-DOF Modus)
// ==================================================================
void adaptiveUpdate(float gx_deg, float gy_deg, float gz_deg, float ax, float ay, float az, float mx, float my, float mz) {
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
    // ruhig: mehr Fokus auf Accel und Magnetometer
    adaptive = false;
    filter.beta = BETA_NORM;
  }

  // 9-DOF Update mit Gyro, Accel UND Magnetometer
  // Die Madgwick Lib normalisiert schon auf rad/s intern (Wir müssen das nich nochmal machen)
  filter.update(gx_deg, gy_deg, gz_deg, ax, ay, az, mx, my, mz);
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


