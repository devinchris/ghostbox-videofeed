#ifndef RotationManager_h
#define RotationManager_h

#include <MadgwickAHRS.h>
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

class RotationManager {
public:
    // Konstruktor
    RotationManager();
    
    // Initialisierung
    void init(const float SAMPLE_FREQ = 25.0f);

    // PRIMARY Getter für alle berechneten Sensordaten
    void getCalculatedData(Quaternion& quat, float& ax, float& ay, float& az, float& mx, float& my, float& mz);
    
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
  Madgwick filter;
  SensorData data;
    
    unsigned long currTime;
    SmoothQuaternionData SmoothData;

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
    static const int CALIBRATION_SAMPLES = 200;

    // Magnetometer Daten
    float mx_raw, my_raw, mz_raw;
    int MAG_COUNTER = 0;
    static const int MAG_SAMPLES_COUNT = 1;    // Berücksichtige Magnetometer Werte jedes n-te sample
    // Ziel: 48µT in Deutschland
    static const int MAG_MIN = 28;
    static const int MAG_MAX = 78;
    float mx_last, my_last, mz_last;
    float mx_avg, my_avg, mz_avg;
    bool magValid;
    void calculateMagnetometerData(float& mx, float& my, float& mz);
    void calcMagEverytime(float& mx, float& my, float& mz);

    // Dynamische Gain Anpassung
    void adaptiveFilterGain(float ax, float ay, float az, float gx, float gy, float gz);
    static constexpr float ACCEL_MAG_MIN = 0.75f; // g
    static constexpr float ACCEL_MAG_MAX = 1.25f; // g
    static constexpr float BETA_MIN = 0.01f;      // TODO:
    static constexpr float BETA_MAX = 0.3f;       // DEBUG & test
    static constexpr float BETA_NORM_LOW = 0.05f;
    static constexpr float BETA_NORM_HIGH = 0.1f;


    // Axis alginment
    static constexpr int ACCEL_REALIGNMENT[3] = {1, 1, 1};
    static constexpr int GYRO_REALIGNMENT[3]  = {1, 1, 1}; 
    static constexpr int MAG_REALIGNMENT[3]   = {1, -1, 1};
    void realignAccel(float& ax, float& ay, float& az);
    void realignGyro(float& gx, float& gy, float& gz);
    void realignMag(float& mx, float& my, float& mz);

    // Magnetische Soft-Iron-Kalibrierung (3x3 Matrix)
    const float MAG_SOFT_IRON_MATRIX[3][3] = {
      {0.915, -0.001, -0.05},
      {0.000,  1.115,  0.006},
      {0.053,  0.000,  0.996}
    };

    const float MAG_HARD_IRON_OFFSET[3] = {
      21.092, -24.715, -62.876              // X, Y, Z Offsets
    };
    
    // Sensor-Messung
    void ahrsMeasure();

    // DEBUG:
    void debugEulerAngles();
    void debugRawData();
    void debugQuaternion(Quaternion _Q);
    void printAligned(float value);
    void debugMeasurements(Quaternion _Q, float ax, float ay, float az, float mx, float my, float mz);
};

#endif