/* 
 *         ========== SmoothData4D ==========
 *  Eigene Library für die Glättung von 4D-Quaternionen
 *  Filtert Noise und Vibrationen
 * 
 *  Optimiert für unseren Nano 33 BLE Sense Rev2
 *  Berechnung laufen teils auf der FPU unseres Gerätes
 */
#include "SmoothData4D.h"
#include <Arduino.h>
#include <arm_math.h>  // Auf unseren ARM Chip optimiert

// Konstruktor
SmoothQuaternionData::SmoothQuaternionData() {
    // TODO: DEBUG Standardwerte
    _isInitialized = false;
    lastSmoothedTime = 0;
    alpha = 0.6f;
    targetFrequency = 100.0f;
    DEADZONE = 0.003f;
    MOTION_THRESHOLD = 0.5f;
    FAST_ALPHA = 0.75f;
    
    // Nutze CMSIS für Initialisierung (optimal für ARM)
    arm_fill_f32(0.0f, _smoothedQuaternions, 4);
    arm_fill_f32(0.0f, _lastQuaternions, 4);
}
void SmoothQuaternionData::initSmoothing(float targetFrequency, float alpha) {
    // ARM FPU: constrain sehr schnell
    this->alpha = constrain(alpha, 0.01f, 0.99f);
    this->targetFrequency = targetFrequency;
    this->FAST_ALPHA = constrain(alpha * 1.5f, 0.01f, 0.95f);
    lastSmoothedTime = micros();
    _isInitialized = false;
}

// ARM-optimierte Normalisierung mit Hardware-SQRT
void SmoothQuaternionData::normalizeQuaternion(float q[4]) {
    float magSquared;
    
    // Skalarprodukt:
    // ARM optimiert (passiert auf der FPU)
    arm_dot_prod_f32(q, q, 4, &magSquared);
    
    // Sicherheitscheck
    if(magSquared < 0.0001f || magSquared > 10.0f) {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        return;
    }
    
    // Quadratwurzel -> Hardware-beschleunigt auf FPU
    float invMagnitude;
    arm_sqrt_f32(magSquared, &invMagnitude);
    invMagnitude = 1.0f / invMagnitude;
    
    // Korrekte Skalierung
    arm_scale_f32(q, invMagnitude, q, 4);
}

void SmoothQuaternionData::smoothQuaternion(Quaternion& _Quat, unsigned long currentTime) {
    // Konvertiere Quaternion zu Array für ARM-optimierte Operationen
    float q[4] = {_Quat.q0, _Quat.q1, _Quat.q2, _Quat.q3};
    
    // Initialisierung
    if(!_isInitialized) {
        // ARM optimierte Kopie zwischenspeichern
        arm_copy_f32(q, _smoothedQuaternions, 4);
        arm_copy_f32(q, _lastQuaternions, 4);
        _isInitialized = true;
        lastSmoothedTime = currentTime;
        
        // Aktualisiere Quaternion mit initialisierten Werten
        _Quat.q0 = _smoothedQuaternions[0];
        _Quat.q1 = _smoothedQuaternions[1];
        _Quat.q2 = _smoothedQuaternions[2];
        _Quat.q3 = _smoothedQuaternions[3];
        return;
    }

    // 1. Skalarprodukt (von Quaternionen) -> Mathematisch kompliziert ig -> Passiert auf unserer FPU (optimiert)
    float dotProduct;
    arm_dot_prod_f32(_lastQuaternions, q, 4, &dotProduct);

    // 2. Vorzeichen für kürzesten Pfad (+/-)
    float sign = (dotProduct < 0.0f) ? -1.0f : 1.0f;
    
    // 3. Korrigierte Quaternion
    float q_corrected[4];
    arm_scale_f32(q, sign, q_corrected, 4);
    // -> Der Filter schaltet gelegentlich um (weil q = -q)
    // -> Dem müssen wir entgegenwirken

    // 4. Differenz mit CMSIS (optimiert für ARM)
    float qDiff[4];
    arm_sub_f32(q_corrected, _lastQuaternions, qDiff, 4);
    // Betrag via CMSIS Dot-Product + SQRT
    float qBetragSquared;
    arm_dot_prod_f32(qDiff, qDiff, 4, &qBetragSquared);
    
    float qBetrag;
    arm_sqrt_f32(qBetragSquared, &qBetrag);

    // 5. Deadzone
    if(qBetrag < DEADZONE) {    // Starke Bewegungen werden nicht geglättet (Fixt einige Probleme aus der Vergangenheit)
        lastSmoothedTime = currentTime;
        
        // Aktualisiere Quaternion mit aktuellen geglätteten Werten
        _Quat.q0 = _smoothedQuaternions[0];
        _Quat.q1 = _smoothedQuaternions[1];
        _Quat.q2 = _smoothedQuaternions[2];
        _Quat.q3 = _smoothedQuaternions[3];
        return;
    }

    // 6. Adaptives Alpha
    float effectiveAlpha = (qBetrag > MOTION_THRESHOLD) ? FAST_ALPHA : alpha;

    // 7. Exponentielles Smoothing
    // Formel: smoothed = alpha * q_neu + (1-alpha) * q_alt
    float temp1[4], temp2[4];
    
    arm_scale_f32(q_corrected, effectiveAlpha, temp1, 4);
    arm_scale_f32(_smoothedQuaternions, (1.0f - effectiveAlpha), temp2, 4);
    arm_add_f32(temp1, temp2, _smoothedQuaternions, 4);

    // 8. Normalisieren
    normalizeQuaternion(_smoothedQuaternions);
    // -> WICHTIG: Nötig weil |q|=1 seien muss (sonst: ungültiges Quaternion!)

    // 9. Letzten Wert speichern
    arm_copy_f32(q_corrected, _lastQuaternions, 4);
    
    lastSmoothedTime = currentTime;
    
    // 10. Aktualisiere das übergebene Quaternion mit den geglätteten Werten
    _Quat.q0 = _smoothedQuaternions[0];
    _Quat.q1 = _smoothedQuaternions[1];
    _Quat.q2 = _smoothedQuaternions[2];
    _Quat.q3 = _smoothedQuaternions[3];
}