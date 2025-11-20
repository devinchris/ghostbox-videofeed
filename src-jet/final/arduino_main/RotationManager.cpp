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
 *  - MadgwickAHRS    | AHRS Library zum berechnen der Rotation anhand 9-Achsen Sensoren, verwendet Mahony oder Madgwick
 *  - FlashPrefs      | Auslesen des internen Flash Speichers (verwendet für Kalibrierung vom Magnetometer)
 *  - BMI270_BMM150   | Auslesen der Sensordaten in Echtzeit der eingebauten BOSCH Sensoren des Arduino Nano
 *  - SmoothData4D    | GHOSTBOX Library für Deadzone, Filter für Vibration & Noise und Glättung
 *
 *  AUTOREN:
 *  ~ Team Ghostbox-Videofeed
 *  ~ D.R.
 * 
 *  PROBLEME IN UNSERER VERSION:
 *  - YAW Drift über Zeit und bei schnellen Bewegungen
 *    -> Magnetometer korrigiert zu wenig
 *  - ROLL & PITCH Drift bei Rotation
 * 
 *  WAS GUT FUNKTIONIERT:
 *  - Schnelle Reaktion auf Bewegungen
 *  - Accel Werte einwandfrei (auch in der Visualisierung angezeigt)
 */

#include <MadgwickAHRS.h>           // AHRS Bibliothek zum Glätten auf Basis des Madgwick-Algorithmus.
#include <NanoBLEFlashPrefs.h>      // Bibliothek um den Nano Flash Speicher auszulesen. Verwenden wir für Magnetometer Kalibrierung.
#include "Arduino_BMI270_BMM150.h"  // Bibliothek für den BMI270 Beschleunigungssensor und BMM150 Magnetometer
#include "SmoothData4D.h"           // GHOSTBOX-eigene Bibliothek um die Werte zu Glätten und Vibration wie Noise zu filtern
#include "RotationManager.h"

// ==========================
// Konstruktor
RotationManager::RotationManager() {
  loopFrequency = 0;
  previousMillis = 0;
  gx_off = gy_off = gz_off = 0;
  ax_off = ay_off = az_off = 0;
}

void RotationManager::init(float SAMPLE_FREQ) {
  currTime = millis();
  Serial.print("AHRS init: ");
  Serial.print(SAMPLE_FREQ); Serial.println(" Hz");
  
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
  Serial.println(SAMPLE_FREQ);
  filter.begin(SAMPLE_FREQ);
  
  Serial.println();
  Serial.println("========== AHRS bereit ==========");
  Serial.println("Fusion: MAHONY | Deklination: 3.5°");
  
  delay(1000);
  // _lastUpdate = micros();
}

void RotationManager::calibrateGyro() {
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

void RotationManager::calibrateAccel() {
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
void RotationManager::debugMeasurements(Quaternion _Q, float ax, float ay, float az, float mx, float my, float mz) {
  Serial.print(" aX: "); Serial.print(ax); Serial.print(" |\t");
  Serial.print("aY: "); Serial.print(ay); Serial.print(" |\t");
  Serial.print("aZ: "); Serial.print(az); Serial.print(" |\t");

  Serial.print("mX: "); Serial.print(mx); Serial.print(" |\t");
  Serial.print("mY: "); Serial.print(my); Serial.print(" |\t");
  Serial.print("mZ: "); Serial.print(mz); Serial.print(" |\t");
  
  // Magnetfeld-Magnitude berechnen (sollte ~48 µT in Deutschland sein)
  float magMagnitude = sqrt(mx*mx + my*my + mz*mz);
  Serial.print("MAG: "); Serial.print(magMagnitude, 2); Serial.print(" µT");
}

// DEBUG: 
void RotationManager::debugEulerAngles() {
  // Euler-Winkel aus aktueller Quaternion des Madgwick Filters berechnen
  // Madgwick: q0 = w, q1 = x, q2 = y, q3 = z
  float q0 = filter.q0;
  float q1 = filter.q1;
  float q2 = filter.q2;
  float q3 = filter.q3;

  float ysqr = q2 * q2;

  // Roll (x-Achse Rotation)
  float t0 = +2.0f * (q0 * q1 + q2 * q3);
  float t1 = +1.0f - 2.0f * (q1 * q1 + ysqr);
  float roll = atan2(t0, t1) * 180.0f / PI;

  // Pitch (y-Achse Rotation)
  float t2 = +2.0f * (q0 * q2 - q3 * q1);
  t2 = t2 > 1.0f ? 1.0f : t2;
  t2 = t2 < -1.0f ? -1.0f : t2;
  float pitch = asin(t2) * 180.0f / PI;

  // Yaw (z-Achse Rotation)
  float t3 = +2.0f * (q0 * q3 + q1 * q2);
  float t4 = +1.0f - 2.0f * (ysqr + q3 * q3);
  float yaw = atan2(t3, t4) * 180.0f / PI;

  Serial.print("ROT: ");
  Serial.print(" R: "); printAligned(roll);
  Serial.print(", P: "); printAligned(pitch);
  Serial.print(", Y: "); printAligned(yaw);
}

// Funktion um floats immer mit selber länge zu printen (übersichtlicher im Serial)
void RotationManager::printAligned(float value) {
  // Immer Vorzeichen anzeigen (+ oder -)
  if (value >= 0) Serial.print("+");
  
  // Wert mit fester Dezimalstellen-Anzahl
  if (abs(value) < 10) Serial.print(" ");      // Extra Space für einstellige Zahlen
  if (abs(value) < 100) Serial.print(" ");     // Extra Space für zweistellige Zahlen
  
  Serial.print(value, 2);  // 2 Dezimalstellen
}
void RotationManager::debugRawData() {
  Serial.print("\t | RAW_ACC: ");
  printAligned(data.ax); Serial.print(",");
  printAligned(data.ay); Serial.print(",");
  printAligned(data.az);
  
  Serial.print("\t | RAW_GYR: ");
  printAligned(data.gx); Serial.print(",");
  printAligned(data.gy); Serial.print(",");
  printAligned(data.gz);
  
  Serial.print("\t | RAW_MAG: ");
  printAligned(data.mx); Serial.print(",");
  printAligned(data.my); Serial.print(",");
  printAligned(data.mz);
  Serial.println();
}

// END DEBUG
// =======================================

// ===================================
// ========== LOOP FUNCTION ==========
// ===================================
void RotationManager::ahrsMeasure(){
  // 1. GYRO auslesen und transformieren
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(data.gx, data.gy, data.gz);
    data.gx -= gx_off;
    data.gy -= gy_off;
    data.gz -= gz_off;
    realignGyro(data.gx, data.gy, data.gz);  // Realignment NACH Offset-Korrektur
  }
  
  // 2. ACCEL auslesen und transformieren
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(data.ax, data.ay, data.az);
    data.ax -= ax_off;
    data.ay -= ay_off;
    data.az -= az_off;
    realignAccel(data.ax, data.ay, data.az);  // Realignment NACH Offset-Korrektur
  }
  
  // 3. MAGNO Daten nur jedes n-te Sample miteinbeziehen
  // -> Block average funktion
  // DEBUG - TEMP:
  calculateMagnetometerData(data.mx, data.my, data.mz);

  // calcMagEverytime(data.mx, data.my, data.mz);

  if(magValid && (data.mx == 0 && data.my == 0 && data.mz == 0)){
    magValid = false;
  }
  adaptiveFilterGain(data.ax, data.ay, data.az, data.gx, data.gy, data.gz);

  // 4. An AHRS (Madgwick) übergeben
  // Wenn Magnetometer-Werte nicht berücksichtigt werden, updateIMU() verwenden
/*   if (magValid) {
    filter.update(data.gx, data.gy, data.gz,
                  data.ax, data.ay, data.az,
                  data.mx, data.my, data.mz);
  } else {
    filter.updateIMU(data.gx, data.gy, data.gz, data.ax, data.ay, data.az);
  } */
  if (magValid) {
    filter.update(data.gy, data.gx, data.gz,
                  data.ay, data.ax, data.az,
                  data.my, data.mx, data.mz);
  } else {
    filter.updateIMU(data.gy, data.gx, data.gz, data.ay, data.ax, data.az);
  }

  // DEBUG
  Serial.println("");
  debugEulerAngles();
  debugMeasurements(Quaternion(filter.q0, filter.q1, filter.q2, filter.q3), data.ax, data.ay, data.az, data.mx, data.my, data.mz);
 }

void RotationManager::getTemperature(float& celsius){
  // TODO: Implementieren
  celsius = 67; // DEBUG
}

// =======================================
// ==== MAIN Function für Sensordaten ====
// =======================================
void RotationManager::getCalculatedData(Quaternion& _quat, float& ax, float& ay, float& az, float& mx, float& my, float& mz, float& gyroMag){
    currTime = millis();
    ahrsMeasure();
    // Quaternion aus Madgwick Filter übernehmen 
    _quat = Quaternion(filter.q0, filter.q1, filter.q2, filter.q3);

    //SmoothData.smoothQuaternions(_quat, currTime);

    // ACCEL:
    ax = data.ax;
    ay = data.ay;
    az = data.az;

    // MAG:
    mx = data.mx;
    my = data.my;
    mz = data.mz;

    gyroMag = sqrt(data.gx*data.gx + data.gy*data.gy + data.gz*data.gz);

    //DEBUG:
    /* Serial.print("GYRO_MAG: ");
    Serial.print(sqrt(data.gx*data.gx + data.gy*data.gy + data.gz*data.gz), 2);
    Serial.print(" | ACC_MAG: ");
    Serial.print(sqrt(data.ax*data.ax + data.ay*data.ay + data.az*data.az), 2);
    Serial.println(); */
    
    return;
}

// ========== MAGNETOMETER ===========
void RotationManager::calculateMagnetometerData(float& mx, float& my, float& mz) {
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx_raw, my_raw, mz_raw);
    
    // 1. Hard Iron Correction
    // Offsets Berechnet mit Magneto 1.2
    float mx_hard = mx_raw - MAG_HARD_IRON_OFFSET[0];
    float my_hard = my_raw - MAG_HARD_IRON_OFFSET[1];
    float mz_hard = mz_raw - MAG_HARD_IRON_OFFSET[2];
    
    // 2. Soft Iron Correction
    // 3x3 Matrix berechnet mit Magneto 1.2
    float mx_soft = MAG_SOFT_IRON_MATRIX[0][0] * mx_hard +
                    MAG_SOFT_IRON_MATRIX[0][1] * my_hard +
                    MAG_SOFT_IRON_MATRIX[0][2] * mz_hard;
    
    float my_soft = MAG_SOFT_IRON_MATRIX[1][0] * mx_hard +
                    MAG_SOFT_IRON_MATRIX[1][1] * my_hard +
                    MAG_SOFT_IRON_MATRIX[1][2] * mz_hard;
    
    float mz_soft = MAG_SOFT_IRON_MATRIX[2][0] * mx_hard +
                    MAG_SOFT_IRON_MATRIX[2][1] * my_hard +
                    MAG_SOFT_IRON_MATRIX[2][2] * mz_hard;
    
    float mag_magnitude = sqrt(mx_soft*mx_soft + my_soft*my_soft + mz_soft*mz_soft);

    if(mag_magnitude < MAG_MIN || mag_magnitude > MAG_MAX){
      // Ungültiger Messwert
      Serial.println("");
      Serial.print("Magnetometer Messwert ungültig: MAG:  ");
      Serial.println(mag_magnitude);
      
      mx_avg += mx_last; my_avg += my_last; mz_avg += mz_last;
      magValid = false;
      return;
    }
    MAG_COUNTER++;

    mx_last = mx_soft; my_last = my_soft; mz_last = mz_soft;

    mx_avg += mx_soft;
    my_avg += my_soft;
    mz_avg += mz_soft;
    
    if(MAG_COUNTER >= MAG_SAMPLES_COUNT) {
      mx = mx_avg / MAG_COUNTER;
      my = my_avg / MAG_COUNTER;
      mz = mz_avg / MAG_COUNTER;

      // Reset für nächsten Durchlauf
      MAG_COUNTER = 0;
      mx_avg = 0; my_avg = 0; mz_avg = 0;

      magValid = true;
      return;
    }
    else {
      magValid = false;
      return;
    }
  }
}

void RotationManager::adaptiveFilterGain(float ax, float ay, float az, float gx, float gy, float gz){
  float acc_mag = sqrt(ax*ax + ay*ay + az*az);
  float gyro_mag = sqrt(gx*gx + gy*gy + gz*gz);
  int counter;

  if(gyro_mag > 200.0f){
    // Bei schnellen Drehungen
    filter.beta = BETA_NORM_HIGH;
    counter = 50;
    return;
  } else if(counter > 0 || acc_mag < ACCEL_MAG_MIN || acc_mag > ACCEL_MAG_MAX){
    counter--;
    filter.beta = BETA_MIN;
    return;
  }
  if(acc_mag < ACCEL_MAG_MIN || acc_mag > ACCEL_MAG_MAX){
    if(az < 0.75){
      filter.beta = 0.0001f;
    }
    filter.beta = BETA_MAX;
  } else if(acc_mag > 0.9 || acc_mag < 1.1){
    filter.beta = BETA_NORM_LOW;
    return;
  } else {
    filter.beta = BETA_NORM_HIGH;
  }

}

/* void SensorManager::calcMagEverytime(float& mx, float& my, float& mz) {
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx_raw, my_raw, mz_raw);
    
    // 1. Hard Iron Correction
    // Offsets Berechnet mit Magneto 1.2
    float mx_hard = mx_raw - MAG_HARD_IRON_OFFSET[0];
    float my_hard = my_raw - MAG_HARD_IRON_OFFSET[1];
    float mz_hard = mz_raw - MAG_HARD_IRON_OFFSET[2];
    
    // 2. Soft Iron Correction
    // 3x3 Matrix berechnet mit Magneto 1.2
    float mx_soft = MAG_SOFT_IRON_MATRIX[0][0] * mx_hard +
                    MAG_SOFT_IRON_MATRIX[0][1] * my_hard +
                    MAG_SOFT_IRON_MATRIX[0][2] * mz_hard;
    
    float my_soft = MAG_SOFT_IRON_MATRIX[1][0] * mx_hard +
                    MAG_SOFT_IRON_MATRIX[1][1] * my_hard +
                    MAG_SOFT_IRON_MATRIX[1][2] * mz_hard;
    
    float mz_soft = MAG_SOFT_IRON_MATRIX[2][0] * mx_hard +
                    MAG_SOFT_IRON_MATRIX[2][1] * my_hard +
                    MAG_SOFT_IRON_MATRIX[2][2] * mz_hard;
    
    float mag_magnitude = sqrt(mx_soft*mx_soft + my_soft*my_soft + mz_soft*mz_soft);

    if(mag_magnitude < MAG_MIN || mag_magnitude > MAG_MAX){
      Serial.println("");
      Serial.print("Magnetometer Messwert ungültig: MAG:  ");
      Serial.println(mag_magnitude);
      data.mx = 0; data.my = 0; data.mz = 0;
      magValid = false;
      return;
    }
    data.mx = mx_soft;
    data.my = my_soft;
    data.mz = mz_soft;
    magValid = true;
  } else {
    mx = 0;
    my = 0;
    mz = 0;
    magValid = false;
  }
} */

// DATENKORREKTUR (Achseninvertierungen etc)
void RotationManager::realignAccel(float& ax, float& ay, float& az) {
  ax = ax * ACCEL_REALIGNMENT[0];
  ay = ay * ACCEL_REALIGNMENT[1];
  az = az * ACCEL_REALIGNMENT[2];
}

void RotationManager::realignGyro(float& gx, float& gy, float& gz) {
  gx = gx * GYRO_REALIGNMENT[0];
  gy = gy * GYRO_REALIGNMENT[1];
  gz = gz * GYRO_REALIGNMENT[2];
}

void RotationManager::realignMag(float& mx, float& my, float& mz) {
  mx = mx * MAG_REALIGNMENT[0];
  my = my * MAG_REALIGNMENT[1];
  mz = mz * MAG_REALIGNMENT[2];
}
