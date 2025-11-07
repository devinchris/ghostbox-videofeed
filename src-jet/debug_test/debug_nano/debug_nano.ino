#include "Arduino_BMI270_BMM150.h"
#include <ReefwingAHRS.h>
#include <NanoBLEFlashPrefs.h>

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
NanoBLEFlashPrefs myFlashPrefs;
ReefwingAHRS ahrs;
SensorData data;

// Display und Loop-Frequenz
int loopFrequency = 0;
const long displayPeriod = 100;  // 100ms = 10 Hz Display
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("=== AHRS Initialisierung ===");
  
  // IMU initialisieren
  if (!IMU.begin()) {
    Serial.println("FEHLER: IMU konnte nicht initialisiert werden!");
    while (1);
  }
  
  Serial.println("✓ IMU initialisiert");
  
  // Lade Kalibrierung aus Flash
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
  
  // AHRS initialisieren
  ahrs.begin();
  
  // Wähle Fusion-Algorithmus
  ahrs.setFusionAlgorithm(SensorFusion::MAHONY);
  // Oder: ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
  
  // Setze magnetische Deklination für Deutschland
  ahrs.setDeclination(3.5);
  
  Serial.println();
  Serial.println("========== AHRS bereit ==========");
  Serial.println("Fusion: MAHONY | Deklination: 3.5°");
  Serial.println();
  Serial.println("Roll    |   Pitch   |   Yaw   |   Heading   |   Hz");
  Serial.println("--------------------------------------------------");
  
  delay(2000);
}


void calibrateGyro() {
  float gx_off_calc, gy_off_calc, gz_off_calc;
  for(int i = 1; i<=CALIBRATION_SAMPLES; i++){
    IMU.readGyroscope(data.gx, data.gy, data.gz);
    gx_off_calc += data.gx;
    gy_off_calc += data.gy;
    gz_off_calc += data.gz;
  }
  // Kalibrierung durch Mittelwert
  gx_off = gx_off_calc / CALIBRATION_SAMPLES;
  gy_off = gy_off_calc / CALIBRATION_SAMPLES;
  gz_off = gz_off_calc / CALIBRATION_SAMPLES;
}

void calibrateAccel() {
  float ax_off_calc, ay_off_calc, az_off_calc;
  for(int i = 1; i<=CALIBRATION_SAMPLES; i++){
    IMU.readGyroscope(data.ax, data.ay, data.az);
    ax_off_calc += data.ax;
    ay_off_calc += data.ay;
    az_off_calc += data.az;
  }
  // Kalibrierung durch Mittelwert
  ax_off = ax_off_calc / CALIBRATION_SAMPLES;
  ay_off = ay_off_calc / CALIBRATION_SAMPLES;
  az_off = az_off_calc / CALIBRATION_SAMPLES;
}

void loop() {
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
  
  // Zeige Daten periodisch an
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
  
  loopFrequency++;
}

// ==========================
// === LIBRARY FUNCTIONS ===
// ==========================

void begin(float FREQUENCY, float ){

}

void measure(){

}

void getRotationQuaternion(float& q0, float& q1, float& q2, float& q3){
  Quaternion filteredRotation = ahrs.getQuaternion();
  // TODO:
  // float smoothedRotation = smoothData4D(filteredRotation.q0, filteredRotation.q1, filteredRotation.q2, filteredRotation.q3,);
  q0 = filteredRotation.q0;
  q1 = filteredRotation.q1;
  q2 = filteredRotation.q2;
  q3 = filteredRotation.q3;
  return;
}

void getAccelValues(float& ax, float& ay, float& az){
  ax = data.ax;
  ay = data.ay;
  az = data.az;
  return;
}

void getTemperature(float& celsius){
  // TODO
}

