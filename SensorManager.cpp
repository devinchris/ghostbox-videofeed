#include <ReefwingAHRS.h>           // AHRS Bibliothek zum Glätten, abgestimmt auf unseren Arduino. Unterstützt Madgwick und Mahony.
#include <NanoBLEFlashPrefs.h>      // Bibliothek um den Nano Flash Speicher auszulesen. Verwenden wir für Magnetometer Kalibrierung.
#include "Arduino_BMI270_BMM150.h"  // Bibliothek für den BMI270 Beschleunigungssensor und BMM150 Magnetometer
#include "SmoothData4D.h"           // GHOSTBOX-eigene Bibliothek um die Werte zu Glätten und Vibration wie Noise zu filtern

// KALIBRIERUNG
// Kalibrierungs-Struktur
typedef struct {
  float magOffsetX;
  float magOffsetY;
  float magOffsetZ;
  bool isCalibrated;
} MagCalibration;

#define CALIBRATION_SAMPLES 200
float gx_off, gy_off, gz_off;
float ax_off, ay_off, az_off;

MagCalibration magCal;
// Flash Speicher wird genutzt für Magnetometer Kalibrierung
NanoBLEFlashPrefs myFlashPrefs;
// Sensordaten auslesen & berechnen
ReefwingAHRS ahrs;
SensorData data;

// Display und Loop-Frequenz
int loopFrequency = 0;
const long displayPeriod = 100;  // 100ms = 10 Hz Display
unsigned long previousMillis = 0;

// ==========================

// Konstruktor
SensorManager::SensorManager() {
  loopFrequency = 0;
  previousMillis = 0;
  gx_off = gy_off = gz_off = 0;
  ax_off = ay_off = az_off = 0;
}
void SensorManager::begin() {
/*   if(!Serial){
    Serial.begin(115200);
  }
  while (!Serial) delay(10); */     // DEBUG!

  Serial.println("=== AHRS Initialisierung ===");
  
  // IMU initialisieren
  if (!IMU.begin()) {
    Serial.println("FEHLER: IMU konnte nicht initialisiert werden!");
    while (1);
  }
  
  Serial.println("✓ IMU initialisiert");
  
  // Lade Magnetometer Kalibrierung aus dem Flash
  int rc = myFlashPrefs.readPrefs(&magCal, sizeof(magCal));
  
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
  delay(1000);  //DEBUG

  // AHRS initialisieren
  ahrs.begin();

  // Fusion-Algorithmus auswählen
  ahrs.setFusionAlgorithm(SensorFusion::MAHONY);    // TODO: DEBUG beide
  // ODER: ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
  
  ahrs.setDeclination(3.5); // Deklination für Deutschland (durchschnittlich)
  
  Serial.println();
  Serial.println("========== AHRS bereit ==========");
  Serial.println("Fusion: MAHONY | Deklination: 3.5°");
  Serial.println();
  Serial.println("Roll    |   Pitch   |   Yaw   |   Heading   |   Hz");
  Serial.println("--------------------------------------------------");
  
  delay(2000);
}

void SensorManager::calibrateGyro() {
  Serial.println("Kalibriere Gyroskop... Stillhalten!");

  float gx_off_calc, gy_off_calc, gz_off_calc;
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

  float ax_off_calc, ay_off_calc, az_off_calc;
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
  az_off = az_off_calc / CALIBRATION_SAMPLES;
  Serial.println("✓ Accelerometer kalibriert");
  Serial.print("Offsets: ");
  Serial.print(ax_off, 2); Serial.print(", ");
  Serial.print(ay_off, 2); Serial.print(", ");
  Serial.println(az_off, 2);
}

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
    data.ax -= ax_off;
    data.ay -= ay_off;
    data.az -= az_off;
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
  ahrs.setData(data);
  ahrs.update();
  
/*   // Zeige Daten periodisch an
  if (millis() - previousMillis >= displayPeriod) {
    Serial.print("R: ");
    Serial.print(ahrs.angles.roll, 1);
    Serial.print("° | \t P: ");
    Serial.print(ahrs.angles.pitch, 1);
    Serial.print("° | \t Y: ");
    Serial.print(ahrs.angles.yaw, 1);
    Serial.print("° | \t H: ");
    Serial.print(ahrs.angles.heading, 1);
    Serial.print("° | ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");
    
    loopFrequency = 0;
    previousMillis = millis();
  }
  
  loopFrequency++; */
}

/* void SensorManager::getRotationQuaternion(float& q0, float& q1, float& q2, float& q3){
  Quaternion filteredRotation = ahrs.getQuaternion();
  // TODO:
  // float smoothedRotation = smoothData4D(filteredRotation.q0, filteredRotation.q1, filteredRotation.q2, filteredRotation.q3,);
  q0 = filteredRotation.q0;
  q1 = filteredRotation.q1;
  q2 = filteredRotation.q2;
  q3 = filteredRotation.q3;
  return;
} */

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
    return;
}

/* int SensorManager::getLoopFrequency() {
  return loopFrequency;
} */
