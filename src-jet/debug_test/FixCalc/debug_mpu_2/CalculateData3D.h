#pragma once                // Code wird nur einmal vom compiler eingebunden
#include <Arduino.h>
#include <math.h>           // Mathe
#include <MadgwickAHRS.h>   // Madgwick Filter wird verwendet, um aus Winkelgeschwindigkeiten und Beschleunigung eine

class CalculateData3D {
public:
    // alpha_base        = Grundglättung (wie stark neue Werte einfließen)
    // adaptivity        = Faktor für Veränderung für Alpha
    // deadzone          = kleine Änderungen werden ignoriert
    // dynamic_threshold = Schwelle für adaptive Anpassung (schnelle Bewegung)
    CalculateData3D(float alpha_base = 0.2f, float adaptivity = 0.2f,
                 float deadzone = 0.05f, float dynamic_threshold = 0.1f,
                 float frequency = 250.0f)
        : alpha_base(alpha_base), adaptivity(adaptivity),
          DEADZONE(deadzone), DYNAMIC_THRESHOLD(dynamic_threshold),
          target_frequency(frequency), last_time(0),
          accel_lsb_per_g(8192.0f), gyro_lsb_per_dps(16.4f), // Default-Werte (z. B. MPU6050 @ ±4g/±2000dps)
          smoothing_enabled(true)
    {
        // Anfangszustand: keine Werte gespeichert
        for (int i = 0; i < 3; i++) last_smoothed[i] = NAN;
    }

    // --- Filter Initialisierung (sollte im setup() aufgerufen werden) ---
    void initFilter(float sampleFrequencyHz = 100.0f, float beta = 0.1f) {      // Vorsicht: die function kommt vom chat
        target_frequency = sampleFrequencyHz;
        // Initialisierung je nach verwendeter Madgwick-Library
        // (einige Libs brauchen begin(), andere nicht)
        // Versuche begin() aufzurufen — die Arduino-Beispiele verwenden filter.begin(frequency)
        // Falls deine Madgwick-Variante kein begin() hat, entferne die folgende Zeile oder
        // definiere MadgwickBeginAvailable entsprechend in deinem Build.
        #ifdef MadgwickBeginAvailable
            filter.begin(sampleFrequencyHz);
        #else
            // Falls die Library kein begin() hat, sollte in der jeweiligen Madgwick-Variante
            // invSampleFreq / sample frequency passend gesetzt werden (extern/anders).
            // Wir belassen es hier bedingt kompatibel; sicherstelle beim Testen, dass die
            // Madgwick-Library auf die gewünschte Sample-Rate eingestellt ist.
        #endif
        #ifdef MadgwickSetBetaAvailable
            filter.setBeta(beta);
        #endif
    }

    // In der public-Sektion der Klasse hinzufügen:
    void setAccelScale(float lsb_per_g) {
        accel_lsb_per_g = lsb_per_g;
    }

    void setGyroScale(float lsb_per_dps) {
        gyro_lsb_per_dps = lsb_per_dps;
    }

    // optional: smoothing an/aus
    void setSmoothingEnabled(bool enabled) {
        smoothing_enabled = enabled;
        if (!enabled) {
            // reset smoothing state damit beim nächsten Durchlauf direkt aktuelle Werte benutzt werden
            for (int i = 0; i < 3; i++) last_smoothed[i] = NAN;
        }
    }

    /*
     *  gyro[3]   = Gyroskop in LSB (oder bereits in deg/s, siehe Skalierungseinstellungen)
     *  accel[3]  = Beschleunigung in LSB (oder bereits in g, siehe Skalierungseinstellungen)
     *  magnet[3] = Norden ausger. in T     (x, y, z)  (wenn vorhanden, sonst {0,0,0})
     *  output[3] = Rotationsmatrix Werte   (x, y, z)
     */
    void getCalculated3DResults(const float gyro[3], const float accel[3], const float magnet[3], float output[3]) {
        unsigned long now = micros();
        float delta_time = computeDeltaTime(now);

        // Lokale Kopien und Skalierung
        float accelG[3];
        float gyroDPS[3];
        float magTmp[3];
        scaleSensorData(accel, gyro, magnet, accelG, gyroDPS, magTmp);

        // Madgwick Filter Update (aus Gyro + Accel + optional Magnet)
        updateMadgwickFilter(gyroDPS, accelG, magTmp);

        // Berechne Euler-Winkel (Roll, Pitch, Yaw)
        float updated_deg[3];
        computeEulerDegrees(updated_deg);

        // Glättung (mit dynamischem Alpha und Deadzone)
        if (smoothing_enabled)
            smoothRotation(updated_deg, output, delta_time);
        else {
            // direkt ausgeben wenn smoothing deaktiviert
            for (int i = 0; i < 3; i++) {
                last_smoothed[i] = updated_deg[i];
                output[i] = updated_deg[i];
            }
        }
    }

private:
    // Config
    float alpha_base;         // Grundwert für Glättung (0.0 -> 1.0)
    float adaptivity;         // Stärke der Anpassung bei Bewegung
    float DEADZONE;           // Bewegungsschwelle zum Ignorieren kleiner Änderungen
    float DYNAMIC_THRESHOLD;  // Schwelle für "starke Bewegung"
    float target_frequency;   // Ziel-Abtastrate in Hz

    // Sensor-Skalierung (Standardwerte anpassbar)
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;

    Madgwick filter;          // Madgwick Filter Objekt
    float last_smoothed[3];   // letzter geglätteter Wert je Achse
    unsigned long last_time;  // Zeitpunkt der letzten Aktualisierung (micros)
    bool smoothing_enabled;   // smoothing on/off
    
    // === Zeitdifferenz-Berechnung ===
    float computeDeltaTime(unsigned long now) {
        float delta_time;
        if (last_time == 0 || now < last_time)
            delta_time = 1.0f / target_frequency;
        else
            delta_time = (now - last_time) / 1e6f;
        last_time = now;
        return delta_time;
    }

    // === Sensorwerte skalieren und normalisieren ===
    void scaleSensorData(const float accelIn[3], const float gyroIn[3], const float magnetIn[3],
                         float accelOut[3], float gyroOut[3], float magnetOut[3]) {
        for (int i = 0; i < 3; i++) {
            // Wenn der Nutzer bereits in physikalischen Einheiten (g / deg/s) liefert,
            // setzt er die LSB-Scale entsprechend: setAccelScale(1.0f) bzw. setGyroScale(1.0f).
            accelOut[i] = accelIn[i] / accel_lsb_per_g;   // -> g
            gyroOut[i] = gyroIn[i] / gyro_lsb_per_dps;    // -> deg/s
            // magnetIn ist ein Array; wenn du keinen Magnetometer hast, übergib {0,0,0}
            magnetOut[i] = magnetIn ? magnetIn[i] : 0.0f;
        }
    }

    // === Madgwick Filter Update ===
    void updateMadgwickFilter(const float gyroDPS[3], const float accelG[3], const float mag[3]) {
        // IMPORTANT: die Madgwick-Library (wie die in deinem Beispiel) erwartet Gyro in deg/s
        // und konvertiert intern in rad/s. Deshalb **nicht** nochmal in rad/s wandeln.

        // TEMP: Zum debuggen wird der magnetometer zunächst nicht verwendet
        if (mag[0] != 0.0f || mag[1] != 0.0f || mag[2] != 0.0f) {
            filter.update(gyroDPS[0], gyroDPS[1], gyroDPS[2],
                          accelG[0], accelG[1], accelG[2],
                          mag[0], mag[1], mag[2]);
        } else {
            filter.updateIMU(gyroDPS[0], gyroDPS[1], gyroDPS[2],
                             accelG[0], accelG[1], accelG[2]);
        }
    }

    // === Euler-Winkel aus Quaternion berechnen ===
    void computeEulerDegrees(float output[3]) {
        /*float q0 = filter.q0;
        *float q1 = filter.q1;
        *float q2 = filter.q2;
        *float q3 = filter.q3;
        *
        *float roll_rad  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
        *float pitch_rad = asinf( fmaxf(-1.0f, fminf(1.0f, 2.0f * (q0*q2 - q3*q1))) );
        *float yaw_rad   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
        */
        //output[0] = roll_rad  * 180.0f / PI;
        //output[1] = pitch_rad * 180.0f / PI;
        //output[2] = yaw_rad   * 180.0f / PI;
        output[0] = filter.getRoll();
        output[1] = filter.getPitch();
        output[2] = filter.getYaw();
    }

    // === Dynamische Glättung mit Deadzone ===
    void smoothRotation(const float input[3], float output[3], float delta_time) {
        float delta_time_target = 1.0f / target_frequency;

        for (int i = 0; i < 3; i++) {
            float x_t = input[i];

            // Erste Messung
            if (isnan(last_smoothed[i])) {
                last_smoothed[i] = x_t;
                output[i] = x_t;
                continue;
            }

            float diff = fabsf(x_t - last_smoothed[i]);

            // Dynamische Anpassung von Alpha
            float alpha;
            if (diff > DYNAMIC_THRESHOLD)
                alpha = fminf(1.0f, alpha_base + adaptivity * diff);
            else
                alpha = 1.0f - expf(-delta_time / delta_time_target);

            // Deadzone (kleine Änderungen ignorieren)
            if (fabsf(diff) < DEADZONE)
                output[i] = last_smoothed[i];
            else {
                last_smoothed[i] = alpha * x_t + (1.0f - alpha) * last_smoothed[i];
                output[i] = last_smoothed[i];
            }
        }
    }
};
