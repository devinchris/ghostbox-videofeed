/*
 *  SensorManager für unseren ARDUINO NANO 33 BLE SENSE REV 2
 *
 *  Initialisiert und kalibriert Onboard Sensoren
 *  Liest die Werte der 9-Achsen Sensoren (IMU und Magno), sowie Temperatur und Druck aus
 *  Berechnet aktuelle Rotationslage durch AHRS Funktionen 
 *
 *  RETURNS:
 *  - Rotation im Raum (Quaternion) - geglättet, kalkuliert
 *  - Beschleunigung   (a)          - 3 Achsen
 *  // TODO:
 *  - Temperatur       (°C)
 *  - Timestamp        (millis)
 *  - Magnetometer     (T)          - 3 Achsen
 *  - Luftdruck        (bar)
 *
 *  DEPENDENCIES:
 *  - ReefwingAHRS    | AHRS Library zum berechnen der Rotation anhand 9-Achsen Sensoren, verwendet Mahony oder Madgwick
 *  - FlashPrefs      | Auslesen des internen Flash Speichers (verwendet für Kalibrierung vom Magnetometer)
 *  - BMI270_BMM150   | Auslesen der Sensordaten in Echtzeit der eingebauten BOSCH Sensoren des Arduino Nano
 *  - SmoothData4D    | GHOSTBOX Library für Deadzone, Filter für Vibration & Noise und Glättung
 *
 *  AUTOREN:
 *  ~ Team Ghostbox-Videofeed
 *  ~ D.R.
 * 
 *  PROBLEME IN UNSERER VERSION:
 *  - Visualisierung dreht sich auf zwei Achsen bei pitch und roll (Arbeitet mit Quaternionen, kein Gimbal Lock)
 *    -> Ebenso erkennbar bei der Ausgabe von Euler-Winkeln im Seriellen Monitor
 *    -> Bei Veränderung von ROLL oder PITCH dreht sich auch YAW mit (obwohl YAW eigentlich stabil bleiben sollte)
 *  - Magnetometer Kalibrierung könnte besser sein -> Yaw Drift nur gelegentlich
 * 
 *  WAS GUT FUNKTIONIERT:
 *  - Kaum bis gar kein drift bei pitch und roll
 *  - Schnelle Reaktion auf Bewegungen
 *  - Accel Werte einwandfrei (auch in der Visualisierung angezeigt)
 */

#include <ReefwingAHRS.h>           // AHRS Bibliothek zum Glätten, abgestimmt auf unseren Arduino. Unterstützt Madgwick und Mahony.
#include <NanoBLEFlashPrefs.h>      // Bibliothek um den Nano Flash Speicher auszulesen. Verwenden wir für Magnetometer Kalibrierung.
#include "Arduino_BMI270_BMM150.h"  // Bibliothek für den BMI270 Beschleunigungssensor und BMM150 Magnetometer
#include "SmoothData4D.h"           // GHOSTBOX-eigene Bibliothek um die Werte zu Glätten und Vibration wie Noise zu filtern
#include "SensorManager.h"

// ==========================
// Konstruktor
SensorManager::SensorManager() {
  loopFrequency = 0;
  previousMillis = 0;
  gx_off = gy_off = gz_off = 0;
  ax_off = ay_off = az_off = 0;
}

void SensorManager::init() {

  Serial.println("=== AHRS Initialisierung ===");
  
  // IMU initialisieren
  if (!IMU.begin()) {
    Serial.println("FEHLER: IMU konnte nicht initialisiert werden!");
    while (1);
  }
  
  Serial.println("✓ IMU initialisiert");
  
  // Lade Magnetometer Kalibrierung aus dem Flash
  int rc = flashPrefs.readPrefs(&magCal, sizeof(magCal));
  
  if (rc == 0 && magCal.isCalibrated) {
    Serial.println("✓ Magnetometer-Kalibrierung geladen:");
    Serial.print("  X-Offset: "); Serial.print(magCal.magOffsetX, 2); Serial.println(" µT");
    Serial.print("  Y-Offset: "); Serial.print(magCal.magOffsetY, 2); Serial.println(" µT");
    Serial.print("  Z-Offset: "); Serial.print(magCal.magOffsetZ, 2); Serial.println(" µT");
  } else {
    Serial.println("WARNUNG: Keine Kalibrierung gefunden!");
    Serial.println("   Bitte erst Kalibrier-Sketch ausführen!");
    magCal.magOffsetX = 0;
    magCal.magOffsetY = 0;
    magCal.magOffsetZ = 0;
  }
  calibrateGyro();
  calibrateAccel();
  // delay(1000);  //DEBUG

  // AHRS initialisieren
  ahrs.begin();

  // Anfangs- und Zeitwerte reseten
  ahrs.reset();

  // Fusion-Algorithmus auswählen
  ahrs.setFusionAlgorithm(SensorFusion::MAHONY);    
  // ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);  // -> Funktioniert schlecht
  
  ahrs.setDeclination(3.5); // Deklination für Deutschland (durchschnittlich)
  
  Serial.println();
  Serial.println("========== AHRS bereit ==========");
  Serial.println("Fusion: MAHONY | Deklination: 3.5°");
  
  delay(1000);
  // _lastUpdate = micros();
}

void SensorManager::calibrateGyro() {
  Serial.println("Kalibriere Gyroskop... Stillhalten!");

  float gx_off_calc = 0, gy_off_calc = 0, gz_off_calc = 0;
  for(int i = 1; i<=CALIBRATION_SAMPLES; i++){
    IMU.readGyroscope(data.gx, data.gy, data.gz);
    gx_off_calc += data.gx;
    gy_off_calc += data.gy;
    gz_off_calc += data.gz;
    delay(5);
  }
  // Kalibrierung durch Mittelwert
  gx_off = gx_off_calc / CALIBRATION_SAMPLES;
  gy_off = gy_off_calc / CALIBRATION_SAMPLES;
  gz_off = gz_off_calc / CALIBRATION_SAMPLES;
  Serial.println("✓ Gyroskop kalibriert");
  Serial.print("Offsets: ");
  Serial.print(gx_off, 2); Serial.print(", ");
  Serial.print(gy_off, 2); Serial.print(", ");
  Serial.println(gz_off, 2);
}

void SensorManager::calibrateAccel() {
Serial.println("Kalibriere Accelerometer... Stillhalten!");

  float ax_off_calc = 0, ay_off_calc = 0, az_off_calc = 0;
  for(int i = 1; i<=CALIBRATION_SAMPLES; i++){
    IMU.readAcceleration(data.ax, data.ay, data.az);
    ax_off_calc += data.ax;
    ay_off_calc += data.ay;
    az_off_calc += data.az;
    delay(5);
  }
  
  // Kalibrierung durch Mittelwert
  ax_off = ax_off_calc / CALIBRATION_SAMPLES;
  ay_off = ay_off_calc / CALIBRATION_SAMPLES;
  az_off = az_off_calc / CALIBRATION_SAMPLES - 1;
  Serial.println("✓ Accelerometer kalibriert");
  Serial.print("Offsets: ");
  Serial.print(ax_off, 2); Serial.print(", ");
  Serial.print(ay_off, 2); Serial.print(", ");
  Serial.println(az_off, 2);
}

// =============== DEBUG ===============
void debugMeasurements(Quaternion _Q, float ax, float ay, float az) {
  Serial.println("");
  Serial.print("Q0: "); Serial.print(_Q.q0); Serial.print(" |\t");
  Serial.print("Q1: "); Serial.print(_Q.q1); Serial.print(" |\t");
  Serial.print("Q2: "); Serial.print(_Q.q2); Serial.print(" |\t");
  Serial.print("Q3: "); Serial.print(_Q.q3); Serial.print(" |\t");

  Serial.print("aX: "); Serial.print(ax); Serial.print(" |\t");
  Serial.print("aY: "); Serial.print(ay); Serial.print(" |\t");
  Serial.print("aZ: "); Serial.print(az); Serial.print(" |\t");
}

void SensorManager::debugEulerAngles() {
  float roll = ahrs.angles.roll;
  float pitch = ahrs.angles.pitch;
  float yaw = ahrs.angles.yaw;

  Serial.println("");
  Serial.print("Roll: "); Serial.print(roll, 2); Serial.print(" |\t");
  Serial.print("Pitch: "); Serial.print(pitch, 2); Serial.print(" |\t");
  Serial.print("Yaw: "); Serial.print(yaw, 2); Serial.print(" |\t");
}
// END DEBUG
// =======================================

// ==========================
// === LIBRARY FUNCTIONS ===
// ==========================

void SensorManager::ahrsMeasure(){
  // Lese Sensordaten
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data.gx, data.gy, data.gz);
    // Kalibrierungsdaten
    
    data.gx -= gx_off;
    data.gy -= gy_off;
    data.gz -= gz_off;
  }
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(data.ax, data.ay, data.az);
    // Kalibrierungsdaten einberechnen
    realignAccel(data.ax, data.ay, data.az);

    data.ax -= ax_off;
    data.ay -= ay_off;
    data.az -= az_off;
  } else {
    Serial.println("ERROR: CANT READ ACCEL!!1!11!");
  }
  
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(data.mx, data.my, data.mz);
    
    // Hard-Iron-Kompensation
    data.mx -= magCal.magOffsetX;
    data.my -= magCal.magOffsetY;
    data.mz -= magCal.magOffsetZ;

    // Y-Achse invertieren (BMM150 vs BMI270)
    data.my = -data.my;    
  }
  
  // Übergebe Daten an AHRS und update
  ahrs.setData(data, false);    // -> false bedeutet die Library invertiert die Achsen nicht nochmal
  ahrs.update();
}

void SensorManager::getAccelValues(float& ax, float& ay, float& az){
  ax = data.ax;
  ay = data.ay;
  az = data.az;
  return;
}

void SensorManager::getTemperature(float& celsius){
  // TODO: Implementieren
  celsius = 67; // DEBUG
}

void SensorManager::getCalculatedData(Quaternion& _quat, float& ax, float& ay, float& az){
    // MAIN Function um die Sensordaten zu erhalten
    ahrsMeasure();
    _quat = ahrs.getQuaternion();
    getAccelValues(ax, ay, az);
    // TODO: SMOOTHING
    // DEBUG: 
    debugEulerAngles();
    debugMeasurements(_quat, ax, ay, az);
    return;
}



/* int SensorManager::getLoopFrequency() {
  return loopFrequency;
} */

// DATENKORREKTUR (Achseninvertierungen, etc)
void SensorManager::realignAccel(float& ax, float& ay, float& az) {
  ax = ax * ACCEL_REALIGNMENT[0];
  ay = ay * ACCEL_REALIGNMENT[1];
  az = az * ACCEL_REALIGNMENT[2];
}