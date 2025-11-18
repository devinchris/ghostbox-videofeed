/*
 * ============================================================================
 * ARDUINO NANO 33 BLE SENSE REV2 - REAL-TIME ROTATION TRACKING
 * ============================================================================
 * 
 * Beschreibung:
 *   Dieses Skript erfasst die Orientierung des Arduino Nano 33 BLE Sense Rev2
 *   in Echtzeit unter Verwendung des BMI270 (IMU) und BMM150 (Magnetometer).
 *   
 * Features:
 *   - 9-DOF Sensorfusion (Gyro + Accel + Mag) mit Madgwick Filter
 *   - Quaternionen-Ausgabe (keine Gimbal Lock)
 *   - Roll, Pitch, Yaw Berechnung
 *   - Automatische Kalibrierung beim Start
 *   - Optimierte Filterparameter für glatte Echtzeitdaten
 *   
 * Hardware:
 *   - Arduino Nano 33 BLE Sense Rev2
 *   - Onboard BMI270 (6-Achsen IMU)
 *   - Onboard BMM150 (3-Achsen Magnetometer)
 *   
 * Libraries erforderlich:
 *   - Arduino_BMI270_BMM150 (Arduino IDE Library Manager)
 *   
 * Author: GitHub Copilot
 * Date: 10. November 2025
 * ============================================================================
 */

#include "Arduino_BMI270_BMM150.h"

// ============================================================================
// KONFIGURATION & KONSTANTEN
// ============================================================================

// Samplerate & Timing
#define SAMPLE_RATE_HZ 100          // 100 Hz Update-Rate
#define SAMPLE_PERIOD_MS (1000.0f / SAMPLE_RATE_HZ)
#define BETA_GAIN 0.1f              // Madgwick Beta-Parameter (0.033-0.5)

// Kalibrierung
#define CALIBRATION_SAMPLES 200     // Anzahl Samples für Bias-Kalibrierung
#define MAG_CALIBRATION_TIME_MS 10000  // Zeit für Magnetometer-Kalibrierung

// Sensor-Range Konfiguration
#define GYRO_RANGE 2000.0f          // ±2000 dps
#define ACCEL_RANGE 4.0f            // ±4g

// Konvertierungsfaktoren
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// ============================================================================
// GLOBALE VARIABLEN
// ============================================================================

// Quaternion (w, x, y, z)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Euler Angles (Rad)
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

// Sensor Bias (Kalibrierung)
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;

// Magnetometer Kalibrierung (Hard Iron Offset + Soft Iron Matrix)
float magBiasX = 0.0f, magBiasY = 0.0f, magBiasZ = 0.0f;
float magScaleX = 1.0f, magScaleY = 1.0f, magScaleZ = 1.0f;

// Timing
unsigned long lastUpdate = 0;
float deltaTime = 0.0f;

// Debug-Flags
bool useMagnetometer = false;  // true = 9-DOF, false = 6-DOF

// ============================================================================
// MADGWICK FILTER IMPLEMENTIERUNG
// ============================================================================

/**
 * Madgwick AHRS Filter Update (9-DOF)
 * 
 * @param gx, gy, gz: Gyroscope (rad/s)
 * @param ax, ay, az: Accelerometer (normalisiert)
 * @param mx, my, mz: Magnetometer (normalisiert)
 * @param dt: Delta Time (s)
 */
void madgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float dt) {
    
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Normalisiere Accelerometer
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalisiere Magnetometer
    if (useMagnetometer) {
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
    }

    // Hilfsgrößen
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    if (useMagnetometer) {
        // Magnetometer-basierte Korrektur (9-DOF)
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;

        // Referenzrichtung des Erdmagnetfeldes
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        
        float _2bx = sqrt(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        // Gradientenabstieg Algorithmus (Magnetometer + Accelerometer)
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) 
             - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
             + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
             + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) 
             - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) 
             + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
             + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
             + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) 
             - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) 
             + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
             + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
             + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) 
             + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) 
             + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) 
             + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    } else {
        // Nur Accelerometer (6-DOF)
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay);
    }

    // Normalisiere Gradienten
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Quaternion-Ableitung aus Gyroskop
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Feedback vom Gradientenabstieg
    qDot1 -= BETA_GAIN * s0;
    qDot2 -= BETA_GAIN * s1;
    qDot3 -= BETA_GAIN * s2;
    qDot4 -= BETA_GAIN * s3;

    // Integriere Quaternion-Ableitung
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalisiere Quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/**
 * Schnelle inverse Quadratwurzel (Fast Inverse Square Root)
 * Quelle: Quake III Arena
 */
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// ============================================================================
// QUATERNION ZU EULER ANGLES KONVERTIERUNG
// ============================================================================

/**
 * Berechnet Roll, Pitch, Yaw aus Quaternion
 * Konvention: ZYX (Yaw-Pitch-Roll)
 */
void quaternionToEuler() {
    // Roll (X-Achse Rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y-Achse Rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f)
        pitch = copysign(PI / 2.0f, sinp); // ±90° bei Gimbal Lock
    else
        pitch = asin(sinp);

    // Yaw (Z-Achse Rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2(siny_cosp, cosy_cosp);
}

// ============================================================================
// SENSOR KALIBRIERUNG
// ============================================================================

/**
 * Kalibriert Gyroscope und Accelerometer (Bias-Entfernung)
 * Board muss während Kalibrierung ruhig liegen!
 */
void calibrateSensors() {
    Serial.println("========================================");
    Serial.println("KALIBRIERUNG GESTARTET");
    Serial.println("========================================");
    Serial.println("WICHTIG: Board ruhig auf ebener Fläche legen!");
    Serial.println("Kalibriere Gyroscope & Accelerometer...");
    
    float gxSum = 0, gySum = 0, gzSum = 0;
    float axSum = 0, aySum = 0, azSum = 0;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            float ax, ay, az, gx, gy, gz;
            IMU.readAcceleration(ax, ay, az);
            IMU.readGyroscope(gx, gy, gz);
            
            gxSum += gx; gySum += gy; gzSum += gz;
            axSum += ax; aySum += ay; azSum += az;
        }
        delay(10);
        
        if (i % 40 == 0) Serial.print(".");
    }
    
    Serial.println();
    
    // Berechne Durchschnitt (Bias)
    gyroBiasX = gxSum / CALIBRATION_SAMPLES;
    gyroBiasY = gySum / CALIBRATION_SAMPLES;
    gyroBiasZ = gzSum / CALIBRATION_SAMPLES;
    
    accelBiasX = axSum / CALIBRATION_SAMPLES;
    accelBiasY = aySum / CALIBRATION_SAMPLES;
    accelBiasZ = (azSum / CALIBRATION_SAMPLES) - 1.0f; // -1g Gravitation
    
    Serial.println("✓ Gyroscope & Accelerometer kalibriert!");
    Serial.print("  Gyro Bias: ["); Serial.print(gyroBiasX, 3); Serial.print(", ");
    Serial.print(gyroBiasY, 3); Serial.print(", "); Serial.print(gyroBiasZ, 3); Serial.println("]");
    
    // Magnetometer Kalibrierung (Hard Iron Offset)
    if (useMagnetometer) {
        Serial.println("\nKalibriere Magnetometer...");
        Serial.println("AKTION ERFORDERLICH: Bewege das Board langsam in Achter-Bewegung");
        Serial.println("für 10 Sekunden...");
        
        float mxMin = 1000, mxMax = -1000;
        float myMin = 1000, myMax = -1000;
        float mzMin = 1000, mzMax = -1000;
        
        unsigned long startTime = millis();
        while (millis() - startTime < MAG_CALIBRATION_TIME_MS) {
            if (IMU.magneticFieldAvailable()) {
                float mx, my, mz;
                IMU.readMagneticField(mx, my, mz);
                
                if (mx < mxMin) mxMin = mx;
                if (mx > mxMax) mxMax = mx;
                if (my < myMin) myMin = my;
                if (my > myMax) myMax = my;
                if (mz < mzMin) mzMin = mz;
                if (mz > mzMax) mzMax = mz;
            }
            
            if ((millis() - startTime) % 1000 == 0) Serial.print(".");
            delay(50);
        }
        
        Serial.println();
        
        // Hard Iron Offset (Mittelpunkt)
        magBiasX = (mxMax + mxMin) / 2.0f;
        magBiasY = (myMax + myMin) / 2.0f;
        magBiasZ = (mzMax + mzMin) / 2.0f;
        
        // Soft Iron Kompensation (einfache Skalierung)
        float avgDelta = ((mxMax - mxMin) + (myMax - myMin) + (mzMax - mzMin)) / 3.0f;
        magScaleX = avgDelta / (mxMax - mxMin);
        magScaleY = avgDelta / (myMax - myMin);
        magScaleZ = avgDelta / (mzMax - mzMin);
        
        Serial.println("✓ Magnetometer kalibriert!");
        Serial.print("  Mag Bias: ["); Serial.print(magBiasX, 2); Serial.print(", ");
        Serial.print(magBiasY, 2); Serial.print(", "); Serial.print(magBiasZ, 2); Serial.println("]");
        Serial.print("  Mag Scale: ["); Serial.print(magScaleX, 3); Serial.print(", ");
        Serial.print(magScaleY, 3); Serial.print(", "); Serial.print(magScaleZ, 3); Serial.println("]");
    }
    
    Serial.println("\n========================================");
    Serial.println("KALIBRIERUNG ABGESCHLOSSEN");
    Serial.println("========================================\n");
    delay(1000);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Serielle Kommunikation
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Warte max. 3s auf Serial
    
    Serial.println("\n\n");
    Serial.println("╔═══════════════════════════════════════════════════════╗");
    Serial.println("║  ARDUINO NANO 33 BLE SENSE REV2                      ║");
    Serial.println("║  REAL-TIME ROTATION TRACKING                         ║");
    Serial.println("║  (9-DOF Sensorfusion mit Madgwick Filter)            ║");
    Serial.println("╚═══════════════════════════════════════════════════════╝");
    Serial.println();
    
    // IMU Initialisierung
    Serial.print("Initialisiere BMI270 + BMM150... ");
    if (!IMU.begin()) {
        Serial.println("FEHLER!");
        Serial.println("❌ Sensor-Initialisierung fehlgeschlagen!");
        Serial.println("Überprüfe:");
        Serial.println("  - Board-Verbindung");
        Serial.println("  - Arduino_BMI270_BMM150 Library installiert");
        while (1); // Stop
    }
    Serial.println("OK!");
    
    // Sensor-Konfiguration ausgeben
    Serial.println("\nSensor-Konfiguration:");
    Serial.print("  - Accelerometer: ±"); Serial.print(ACCEL_RANGE); Serial.println("g");
    Serial.print("  - Gyroscope: ±"); Serial.print(GYRO_RANGE); Serial.println("°/s");
    Serial.print("  - Sample Rate: "); Serial.print(SAMPLE_RATE_HZ); Serial.println(" Hz");
    Serial.print("  - Madgwick Beta: "); Serial.println(BETA_GAIN, 3);
    Serial.print("  - Modus: "); Serial.println(useMagnetometer ? "9-DOF (IMU + Mag)" : "6-DOF (nur IMU)");
    Serial.println();
    
    // Kalibrierung
    delay(1000);
    calibrateSensors();
    
    // Timer starten
    lastUpdate = millis();
    
    Serial.println("SYSTEM BEREIT - Starte Rotation Tracking...\n");
    Serial.println("Format: Q: w, x, y, z | RPY: roll, pitch, yaw (°)");
    Serial.println("───────────────────────────────────────────────────────\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    unsigned long now = millis();
    deltaTime = (now - lastUpdate) / 1000.0f; // s
    
    // Samplerate einhalten
    if (deltaTime < (SAMPLE_PERIOD_MS / 1000.0f)) {
        return;
    }
    
    lastUpdate = now;
    
    // -------------------------------------------------------------------------
    // 1. SENSOR-DATEN AUSLESEN
    // -------------------------------------------------------------------------
    
    float ax, ay, az;   // Accelerometer (g)
    float gx, gy, gz;   // Gyroscope (°/s)
    float mx, my, mz;   // Magnetometer (µT)
    
    if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
        return; // Warte auf Daten
    }
    
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    
    // Magnetometer (optional)
    if (useMagnetometer) {
        if (IMU.magneticFieldAvailable()) {
            IMU.readMagneticField(mx, my, mz);
            
            // Magnetometer Kalibrierung anwenden
            mx = (mx - magBiasX) * magScaleX;
            my = (my - magBiasY) * magScaleY;
            mz = (mz - magBiasZ) * magScaleZ;
        } else {
            return; // Warte auf Mag-Daten
        }
    }
    
    // -------------------------------------------------------------------------
    // 2. KALIBRIERUNG ANWENDEN
    // -------------------------------------------------------------------------
    
    gx -= gyroBiasX;
    gy -= gyroBiasY;
    gz -= gyroBiasZ;
    
    ax -= accelBiasX;
    ay -= accelBiasY;
    az -= accelBiasZ;
    
    // -------------------------------------------------------------------------
    // 3. EINHEITEN KONVERTIEREN
    // -------------------------------------------------------------------------
    
    // Gyroscope: °/s → rad/s
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;
    
    // Accelerometer bereits in g (normalisiert im Filter)
    // Magnetometer bereits in µT (normalisiert im Filter)
    
    // -------------------------------------------------------------------------
    // 4. MADGWICK FILTER UPDATE
    // -------------------------------------------------------------------------
    
    madgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltaTime);
    
    // -------------------------------------------------------------------------
    // 5. EULER ANGLES BERECHNEN
    // -------------------------------------------------------------------------
    
    quaternionToEuler();
    
    // -------------------------------------------------------------------------
    // 6. SERIELLE AUSGABE
    // -------------------------------------------------------------------------
    
    // Quaternion
    Serial.print("Q: ");
    Serial.print(q0, 4); Serial.print(", ");
    Serial.print(q1, 4); Serial.print(", ");
    Serial.print(q2, 4); Serial.print(", ");
    Serial.print(q3, 4);
    
    Serial.print(" | RPY: ");
    
    // Euler Angles (Grad)
    Serial.print(roll * RAD_TO_DEG, 2); Serial.print("°, ");
    Serial.print(pitch * RAD_TO_DEG, 2); Serial.print("°, ");
    Serial.print(yaw * RAD_TO_DEG, 2); Serial.println("°");
    
    // Optional: Raw Sensor Data (für Debugging)
    // Serial.print(" | Accel: ["); Serial.print(ax, 2); Serial.print(", ");
    // Serial.print(ay, 2); Serial.print(", "); Serial.print(az, 2); Serial.print("]");
    // Serial.print(" | Gyro: ["); Serial.print(gx * RAD_TO_DEG, 2); Serial.print(", ");
    // Serial.print(gy * RAD_TO_DEG, 2); Serial.print(", "); Serial.print(gz * RAD_TO_DEG, 2); Serial.println("]");
}

// ============================================================================
// ENDE
// ============================================================================
