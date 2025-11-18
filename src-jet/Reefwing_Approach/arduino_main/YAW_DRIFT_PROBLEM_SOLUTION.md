# YAW-DRIFT BEI ROLL/PITCH - PROBLEM & LÖSUNG

## 🔴 Ihr Problem

Sie haben beobachtet:
- Arduino wird nur auf **Roll** oder **Pitch** gekippt (eine Achse)
- **Yaw dreht sich trotzdem mit** (sollte stabil bleiben)
- Magnetometer-Daten zeigen große Schwankungen und Offsets

## 📊 Analyse Ihrer Daten

```
ROT:  R:   -5.78, P:   -8.97, Y: +291.14
mX: 6.21 | mY: 12.09 | mZ: 29.11 | MAG: ~33 µT

ROT:  R:   -2.24, P: +  5.84, Y: +287.51
mX: 5.21 | mY: 11.11 | mZ: 29.10 | MAG: ~33 µT
```

### Probleme:
1. **Magnitude ist zu niedrig**: ~33 µT statt 48 µT
2. **Große Schwankungen**: 
   - mZ: 26-35 µT (9 µT Variation!)
   - mX: 2-7 µT (5 µT Variation)
3. **Yaw springt**: 283° bis 293° (10° Spanne!)

## ❌ Ihr Missverständnis

> "Die Data für mx, my und mz sollte close to zero µT sein when in idle, right?"

**FALSCH!** ❌

### Richtig ist:

Das Magnetometer misst das **Erdmagnetfeld**, das IMMER vorhanden ist:
- **Deutschland**: ~48 µT Gesamtstärke
- **Nicht null**: Die Werte ändern sich je nach Orientierung, aber die **Magnitude bleibt konstant**

**Analogie**: Wie ein Kompass - die Nadel zeigt immer nach Norden (Erdmagnetfeld), auch wenn der Kompass still liegt.

### Was konstant sein sollte:

```cpp
// Magnitude berechnen:
float magnitude = sqrt(mx*mx + my*my + mz*mz);
// Sollte immer ~48 µT sein, egal wie Sie das Board drehen!
```

**Bei Ihnen**: 33-35 µT ❌  
**Sollte sein**: 46-50 µT ✓

## 🔍 Warum Yaw mitdreht

1. **Sie kippen Roll/Pitch** → Magnetometer sieht Erdfeld aus anderer Richtung
2. **Falsche Kalibrierung** → Messwerte sind verfälscht
3. **AHRS denkt**: "Magnetfeld hat sich gedreht = Yaw-Änderung!"
4. **Ergebnis**: Yaw dreht mit, obwohl er stabil bleiben sollte

### Visualisierung:

```
Richtige Kalibrierung:
Roll +10° → Mag-Vektor dreht korrekt mit → Yaw stabil ✓

Falsche Kalibrierung:
Roll +10° → Mag-Vektor zeigt falsch → AHRS korrigiert mit Yaw ✗
```

## ✅ Die Lösung

### Schritt 1: Neu kalibrieren mit verbessertem Sketch

Ich habe einen verbesserten Kalibrierungs-Sketch erstellt:
- **Datei**: `calibrate_magno_improved.ino`
- **Features**:
  - Hard-Iron Kalibrierung
  - Magnitude-Validierung
  - Live-Test-Funktion
  - Bessere Anweisungen

### Schritt 2: Richtige Kalibrierung durchführen

```
1. Upload calibrate_magno_improved.ino
2. Serial Monitor öffnen (115200 baud)
3. Sende 'c' zum Starten
4. 2 Minuten lang rotieren:
   - Langsame Figure-8 Bewegungen
   - Alle Achsen abdecken
   - Fern von Metall/Elektronik
5. Kopiere die Hard-Iron Offsets
```

### Schritt 3: Werte in SensorManager.h einfügen

Nach erfolgreicher Kalibrierung erhalten Sie Output wie:

```cpp
const float MAG_HARD_IRON_OFFSET[3] = {
  23.45,  // X
  -8.67,  // Y
  -42.31  // Z
};
```

→ Diese Werte in `SensorManager.h` eintragen!

### Schritt 4: Validieren

Nach Upload des Haupt-Sketches:

```cpp
// In loop() oder setup():
sensorManager.diagnoseMagnetometer();
```

**Erwartete Ausgabe**:
```
=== MAGNETOMETER DIAGNOSE ===
RAW:   [25.5, -15.2, -40.1] MAG: 50.3 µT
FINAL: [12.3, -8.5, 15.2] MAG: 48.1 µT
TARGET: 48 µT (Deutschland) | DIFF: +0.1 µT
============================
```

✓ Magnitude sollte ~48 µT sein  
✓ Differenz sollte < ±2 µT sein

## 🧪 Temporäre Test-Optionen

### Test 1: AHRS ohne Magnetometer

```cpp
// In SensorManager::ahrsMeasure(), ersetze:
ahrs.setData(data, false);  // Mit Mag

// Mit:
ahrs.setData(data.gx, data.gy, data.gz, 
             data.ax, data.ay, data.az);  // Ohne Mag
```

**Erwartung**:
- Roll/Pitch: Perfekt stabil ✓
- Yaw: Driftet langsam (NORMAL ohne Mag)

→ Bestätigt, dass das Problem beim Magnetometer liegt!

### Test 2: Magnetometer-Gewichtung reduzieren

```cpp
// In SensorManager::init(), nach ahrs.begin():
ahrs.setKp(5.0);  // Reduziert Mag-Einfluss
```

Reduziert Yaw-Drift temporär, aber behebt nicht die Ursache.

## 🛠️ Checkliste für perfekte Kalibrierung

- [ ] **Umgebung prüfen**:
  - Keine Metallgegenstände in der Nähe
  - Keine Lautsprecher, Motoren, Magnete
  - Nicht am Metalltisch
  - Am besten: Arduino in der Luft halten

- [ ] **Kalibrierung durchführen**:
  - Vollständige 2 Minuten durchhalten
  - Langsame, gleichmäßige Bewegungen
  - Figure-8 in allen drei Ebenen
  - Samples > 450 gesammelt

- [ ] **Validierung**:
  - Test-Funktion ('t') zeigt ~48 µT
  - Variation < 5 µT
  - diagnoseMagnetometer() zeigt gute Werte

- [ ] **Integration**:
  - Neue Offsets in SensorManager.h eingefügt
  - Sketch neu kompiliert
  - Yaw bleibt stabil bei Roll/Pitch-Änderungen

## 🎯 Erwartetes Ergebnis

**VORHER** (Ihr aktueller Zustand):
```
Idle: R: -2.54, P: +6.17, Y: +288.71 | MAG: 33 µT
Idle: R: -5.61, P: -7.36, Y: +291.05 | MAG: 35 µT
→ Yaw springt 3° bei kleinen Roll/Pitch-Änderungen ✗
```

**NACHHER** (nach richtiger Kalibrierung):
```
Idle: R: -0.05, P: +0.12, Y: +285.50 | MAG: 48 µT
Idle: R: -0.08, P: -0.15, Y: +285.48 | MAG: 48 µT
→ Yaw stabil (±0.1°), Magnitude konstant ✓

Roll +45°: R: +45.2, P: +0.05, Y: +285.52 | MAG: 48 µT
→ Nur Roll ändert sich, Yaw bleibt stabil! ✓
```

## 📚 Weiterführende Informationen

### Soft-Iron Kalibrierung (fortgeschritten)

Wenn nach Hard-Iron-Kalibrierung die Magnitude immer noch schwankt:
- Verwenden Sie Tools wie **Magneto 1.2** oder **MotionCal**
- Sammeln Sie Datenpunkte (mit 'p' im Kalibrier-Sketch)
- Berechnen Sie 3x3 Soft-Iron Matrix
- Tragen Sie Matrix in SensorManager.h ein

### Typische Fehlerquellen

1. **Unvollständige Rotation**: Nicht alle Orientierungen abgedeckt
2. **Zu schnelle Bewegungen**: Magnetometer kann nicht folgen
3. **Metallische Umgebung**: Verfälscht Messungen
4. **Elektrische Störungen**: USB-Kabel, Stromversorgung

### Debugging-Kommandos

```cpp
// Im Haupt-Sketch:
sensorManager.diagnoseMagnetometer();  // Einmalige Diagnose
// Oder im Kalibrier-Sketch:
't' → Test Magnitude (5 Sekunden)
's' → Stream Live-Daten
```

## 📞 Nächste Schritte

1. **Führen Sie `calibrate_magno_improved.ino` aus**
2. **Senden Sie 't' nach Kalibrierung** → Magnitude sollte ~48 µT sein
3. **Wenn Magnitude immer noch falsch**:
   - Umgebung wechseln (andere Location)
   - Auf elektromagnetische Störungen prüfen
   - Eventuell Hardware-Problem am Magnetometer

4. **Wenn Magnitude gut, aber Yaw immer noch driftet**:
   - Soft-Iron Kalibrierung durchführen
   - AHRS-Parameter anpassen (Declination, Kp/Ki)

## ✨ Zusammenfassung

**Ihr Verständnis-Fehler**: Magnetometer sollte Null sein → **FALSCH**  
**Richtig**: Magnitude sollte konstant ~48 µT sein → **Das ist das Erdmagnetfeld**

**Ihr Problem**: Falsche/unvollständige Kalibrierung  
**Lösung**: Neu kalibrieren mit verbessertem Sketch + richtige Rotation

**Nach Fix**: Yaw bleibt stabil bei Roll/Pitch-Änderungen! 🎉
