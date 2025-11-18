# Magnetometer Kalibrierungs-Anleitung

## Problem-Diagnose

Ihre Ausgabe zeigt:
- **mX**: 2.17 bis 7.25 µT (Schwankung: ~5 µT)
- **mY**: 10.11 bis 14.08 µT (Schwankung: ~4 µT)
- **mZ**: 26.07 bis 35.15 µT (Schwankung: ~9 µT!)
- **Magnitude**: ~33-35 µT (sollte konstant ~48 µT sein)

### Was das bedeutet:

1. **Die Kalibrierung ist NICHT korrekt**
   - Ziel: Konstante Magnitude von ~48 µT
   - Aktuell: Nur ~33-35 µT und schwankend

2. **Hard-Iron-Offset ist falsch**
   - Ihre Werte: `{17.9, -4.26, -36.9}`
   - Diese sollten so gewählt sein, dass die Magnitude bei 48 µT liegt

3. **Soft-Iron-Matrix scheint fast identisch**
   - Das deutet darauf hin, dass keine echte Soft-Iron-Kalibrierung durchgeführt wurde

## Warum Yaw driftet

Wenn Sie Roll/Pitch ändern:
- Das Magnetometer sieht das Erdmagnetfeld aus anderer Richtung
- Falsche Kalibrierung → falsche magnetische Messungen
- AHRS interpretiert dies als Yaw-Änderung
- **Ergebnis**: Yaw dreht sich mit, obwohl Sie nur kippen

## Lösung: Richtige Kalibrierung

### Schritt 1: Raw-Data sammeln

Benutzen Sie ein Kalibrier-Programm, um:
1. Arduino in ALLE Orientierungen drehen (360° um jede Achse)
2. Min/Max Werte für mx, my, mz aufzeichnen
3. Mindestens 30-60 Sekunden lang drehen

### Schritt 2: Hard-Iron Offset berechnen

```cpp
// Formel:
offset_x = (max_x + min_x) / 2.0
offset_y = (max_y + min_y) / 2.0
offset_z = (max_z + min_z) / 2.0
```

### Schritt 3: Soft-Iron Matrix berechnen

Dies erfordert eine Ellipsoid-Fit-Analyse. Tools:
- **Magneto 1.2** (MATLAB/Octave)
- **MotionCal** (Kostenlos von PJRC)
- Python-Skripte mit `scipy`

### Schritt 4: Validierung

Nach Kalibrierung sollten Sie sehen:
- Magnitude konstant bei ~48 µT (±2 µT)
- Werte ändern sich glatt beim Drehen
- Yaw bleibt stabil bei Roll/Pitch-Änderungen

## Temporäre Test-Lösungen

### Option 1: AHRS ohne Magnetometer testen

```cpp
// In SensorManager::ahrsMeasure()
// Kommentieren Sie die Magnetometer-Zeile aus:

// ahrs.setData(data, false);  // Mit Mag
ahrs.setData(data.gx, data.gy, data.gz, 
             data.ax, data.ay, data.az);  // Ohne Mag
```

**Erwartung**: Roll/Pitch funktionieren perfekt, aber Yaw driftet (das ist NORMAL ohne Mag)

### Option 2: Reduzierte Magnetometer-Gewichtung

```cpp
// Nach ahrs.begin() in init():
ahrs.setKp(10.0);  // Reduzieren (Standard ist höher)
```

Dies reduziert den Einfluss des Magnetometers.

## Debugging-Funktion nutzen

Fügen Sie in Ihrem `loop()` (oder in der Arduino-Hauptdatei) ein:

```cpp
// Alle 2 Sekunden Diagnose ausgeben:
static unsigned long lastDiag = 0;
if (millis() - lastDiag > 2000) {
  sensorManager.diagnoseMagnetometer();
  lastDiag = millis();
}
```

Dies zeigt Ihnen:
- Raw-Werte VOR Kalibrierung
- Final-Werte NACH Kalibrierung
- Magnitude-Differenz zum Zielwert

## Elektromagnetische Interferenz prüfen

Mögliche Störquellen:
- **Metallische Objekte** in der Nähe (Tisch, Schrauben, etc.)
- **Stromkabel**, Netzteile
- **Andere elektronische Geräte**
- **Lautsprecher**, Magnete

**Test**: Arduino in der Luft halten, weit weg von Metall/Elektronik.

## Empfohlene Nächste Schritte

1. **Führen Sie `diagnoseMagnetometer()` aus** → Sehen Sie, wie weit Sie von 48 µT entfernt sind
2. **Testen Sie ohne Magnetometer** → Bestätigen Sie, dass Roll/Pitch stabil sind
3. **Neu kalibrieren** mit einem Tool wie MotionCal
4. **Umgebung prüfen** auf magnetische Störungen

## Erwartete Werte (nach guter Kalibrierung)

```
RAW:   [25.5, -15.2, -40.1] MAG: 50.3 µT
FINAL: [12.3, -8.5, 15.2] MAG: 48.1 µT
TARGET: 48 µT (Deutschland) | DIFF: +0.1 µT
```

Magnitude sollte **immer nahe 48 µT bleiben**, egal wie Sie den Arduino drehen.
