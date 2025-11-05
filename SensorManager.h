#ifndef SensorManager_h
#define SensorManager_h

#include <ReefwingAHRS.h>
#include <NanoBLEFlashPrefs.h>
#include "Arduino_BMI270_BMM150.h"
#include "SmoothData4D.h"

// Kalibrierungs-Struktur
typedef struct {
  float magOffsetX;
  float magOffsetY;
  float magOffsetZ;
  bool isCalibrated;
} MagCalibration;

class SensorManager {
public:
    // Konstruktor
    SensorManager();
    
    // Initialisierung
    void begin(/* float frequency = 100.0 */);

    // PRIMARY Getter für alle berechneten Sensordaten
    void getCalculatedData(Quaternion& quat, float& ax, float& ay, float& az);
    
    // Getter für Quaternion
    // void getRotationQuaternion(float& q0, float& q1, float& q2, float& q3);
    
    // Getter für Beschleunigung
    void getAccelValues(float& ax, float& ay, float& az);
    
    // Getter für Temperatur
    void getTemperature(float& celsius);
    
    // Getter für Euler-Winkel -> Irrelevant aber mb für debugging idgaf
    // void getEulerAngles(float& roll, float& pitch, float& yaw);
    
    // Loop-Frequenz abrufen
    int getLoopFrequency();

private:
    // AHRS-Objekte
    ReefwingAHRS ahrs;
    SensorData data;
    
    // Kalibrierung
    NanoBLEFlashPrefs flashPrefs;
    MagCalibration magCal;
    float gx_off, gy_off, gz_off;  // Gyro-Offsets
    float ax_off, ay_off, az_off;  // Accel-Offsets
    
    // Display und Frequenz
    int loopFrequency;
    float updateFrequency;
    const long displayPeriod = 100;  // 100ms
    unsigned long previousMillis;
    
    // Kalibrierungsfunktionen
    void calibrateGyro();
    void calibrateAccel();
    void loadMagCalibration();
    
    // Sensor-Messung
    void measureSensors();
};

#endif
