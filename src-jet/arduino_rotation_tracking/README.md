# Arduino Nano 33 BLE Sense Rev2 - Real-Time Rotation Tracking

## 📋 Übersicht

Dieses Projekt implementiert ein vollständiges Echtzeit-Orientierungssystem für den **Arduino Nano 33 BLE Sense Rev2**. Es verwendet **9-DOF Sensorfusion** (Gyroscope + Accelerometer + Magnetometer) mit dem **Madgwick AHRS Filter**, um präzise Quaternionen und Euler-Winkel (Roll, Pitch, Yaw) zu berechnen.

## ✨ Features

- ✅ **9-DOF Sensorfusion** mit BMI270 (IMU) + BMM150 (Magnetometer)
- ✅ **Madgwick Filter** für numerisch stabile Quaternionen-Berechnung
- ✅ **Keine Gimbal Lock** dank Quaternionen-Darstellung
- ✅ **Automatische Kalibrierung** (Gyro, Accel, Mag) beim Start
- ✅ **Echtzeit-Ausgabe** mit 100 Hz Samplerate
- ✅ **Roll, Pitch, Yaw** in Grad für intuitive Interpretation
- ✅ **Umschaltbar zwischen 6-DOF und 9-DOF** (nur IMU oder IMU + Mag)

## 🔧 Hardware-Anforderungen

- **Arduino Nano 33 BLE Sense Rev2**
- USB-Kabel für Stromversorgung und Datenübertragung
- Ebene Oberfläche für Kalibrierung

## 📚 Benötigte Libraries

Installiere über den Arduino IDE Library Manager:

```
Arduino_BMI270_BMM150
```

**Installation:**
1. Arduino IDE öffnen
2. `Sketch → Include Library → Manage Libraries...`
3. Nach `Arduino_BMI270_BMM150` suchen
4. Installieren

## 🚀 Schnellstart

### 1. Code hochladen

1. Arduino IDE öffnen
2. `arduino_rotation_tracking.ino` laden
3. Board auswählen: `Tools → Board → Arduino Mbed OS Nano Boards → Arduino Nano 33 BLE`
4. Port auswählen: `Tools → Port → /dev/cu.usbmodem... (Arduino Nano 33 BLE)`
5. Upload (Strg+U / Cmd+U)

### 2. Kalibrierung

**WICHTIG:** Das Board muss während der Kalibrierung auf einer ebenen, ruhigen Fläche liegen!

1. Serial Monitor öffnen (115200 Baud)
2. Board wird automatisch neu gestartet
3. **Phase 1 (5s):** Gyroscope & Accelerometer - **Board NICHT bewegen!**
4. **Phase 2 (10s):** Magnetometer - **Board langsam in Achter-Bewegung schwenken**
5. Nach Abschluss: "KALIBRIERUNG ABGESCHLOSSEN" erscheint

### 3. Daten auslesen

Nach der Kalibrierung beginnt die Echtzeit-Ausgabe:

```
Q: 0.9998, 0.0021, -0.0015, 0.0187 | RPY: 0.24°, -0.17°, 2.14°
Q: 0.9997, 0.0023, -0.0014, 0.0189 | RPY: 0.26°, -0.16°, 2.17°
...
```

**Format:**
- `Q:` Quaternion (w, x, y, z)
- `RPY:` Roll (°), Pitch (°), Yaw (°)

## 🧠 Technische Details

### Madgwick Filter

Der **Madgwick AHRS Filter** kombiniert Gyroscope-Integration mit Gradient-Descent-Korrektur basierend auf Accelerometer und Magnetometer.

**Warum Madgwick?**
- ✅ **Numerisch stabil** (keine Singularitäten)
- ✅ **Geringe Rechenlast** (läuft auf 64 MHz Cortex-M4)
- ✅ **Optimiert für Echtzeit** (konstante Ausführungszeit)
- ✅ **Kein Gimbal Lock** (Quaternionen-basiert)

**Beta-Parameter (β):**
- **Niedrig (0.033):** Geringes Rauschen, langsame Konvergenz
- **Mittel (0.1):** ⭐ **Standard** - guter Kompromiss
- **Hoch (0.5):** Schnelle Konvergenz, mehr Rauschen

*Aktuell: `β = 0.1` (anpassbar in Zeile 35)*

### Sensor-Konfiguration

| Sensor         | Range    | Sample Rate | Auflösung |
|----------------|----------|-------------|-----------|
| Accelerometer  | ±4g      | 100 Hz      | 16-bit    |
| Gyroscope      | ±2000°/s | 100 Hz      | 16-bit    |
| Magnetometer   | ±1300µT  | 100 Hz      | 16-bit    |

### Koordinatensystem

```
     Z (oben)
     |
     |
     +---- Y (rechts)
    /
   X (vorne)
```

**Euler-Konvention:** ZYX (Yaw → Pitch → Roll)
- **Roll:** Drehung um X-Achse (Kippen links/rechts)
- **Pitch:** Drehung um Y-Achse (Nicken vor/zurück)
- **Yaw:** Drehung um Z-Achse (Drehen im Uhrzeigersinn)

### Kalibrierung

**Gyroscope & Accelerometer:**
- Berechnet statischen Bias über 200 Samples
- Entfernt Drift und Offset
- Kompensiert Gravitationsvektor (1g)

**Magnetometer:**
- **Hard Iron Korrektur:** Entfernt konstante Magnetfeld-Störungen
- **Soft Iron Korrektur:** Kompensiert Achsen-Asymmetrie
- Benötigt Bewegung zur Erfassung des vollständigen Messbereichs

## 🎨 Visualisierung

### Python (Processing-Alternative)

Erstelle `visualize.py`:

```python
import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Serielle Verbindung
ser = serial.Serial('/dev/cu.usbmodem...', 115200)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])

def update(frame):
    line = ser.readline().decode('utf-8').strip()
    if line.startswith('Q:'):
        parts = line.split('|')[0].replace('Q:', '').split(',')
        q = [float(x) for x in parts]
        
        R = quaternion_to_rotation_matrix(q)
        
        ax.clear()
        ax.set_xlim(-1, 1); ax.set_ylim(-1, 1); ax.set_zlim(-1, 1)
        
        # Achsen zeichnen
        origin = [0, 0, 0]
        ax.quiver(*origin, *R[:, 0], color='r', label='X (Roll)')
        ax.quiver(*origin, *R[:, 1], color='g', label='Y (Pitch)')
        ax.quiver(*origin, *R[:, 2], color='b', label='Z (Yaw)')
        
        ax.legend()
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

ani = FuncAnimation(fig, update, interval=10)
plt.show()
```

**Ausführen:**
```bash
pip install pyserial matplotlib numpy
python visualize.py
```

### Processing Sketch

```processing
import processing.serial.*;

Serial port;
float[] q = new float[4];

void setup() {
  size(800, 600, P3D);
  port = new Serial(this, "/dev/cu.usbmodem...", 115200);
  port.bufferUntil('\n');
}

void draw() {
  background(255);
  translate(width/2, height/2);
  
  // Quaternion zu Rotation
  pushMatrix();
  float angle = 2 * acos(q[0]);
  float x = q[1] / sin(angle/2);
  float y = q[2] / sin(angle/2);
  float z = q[3] / sin(angle/2);
  rotate(angle, x, y, z);
  
  // 3D Box zeichnen
  box(100);
  popMatrix();
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line != null && line.startsWith("Q:")) {
    String[] parts = split(line.split("\\|")[0].replace("Q:", ""), ',');
    for (int i = 0; i < 4; i++) {
      q[i] = float(trim(parts[i]));
    }
  }
}
```

## 🛠️ Anpassungen & Tuning

### Beta-Parameter ändern (Zeile 35)

```cpp
#define BETA_GAIN 0.1f  // Erhöhen für schnellere Konvergenz
```

### 6-DOF Modus (nur IMU, kein Magnetometer)

```cpp
bool useMagnetometer = false;  // Zeile 71
```

**Wann 6-DOF verwenden?**
- In Umgebungen mit magnetischen Störungen (Motoren, Metall)
- Wenn keine absolute Yaw-Referenz benötigt wird
- Für höhere Update-Raten

### Sample Rate anpassen (Zeile 31)

```cpp
#define SAMPLE_RATE_HZ 100  // Bis zu 200 Hz möglich
```

**Hinweis:** Höhere Raten = mehr Rechenlast + schnellere Serial-Ausgabe

### Serial Output Format ändern

Für CSV-Export (Zeile 686):

```cpp
// Statt: Q: w, x, y, z | RPY: roll, pitch, yaw
Serial.print(q0, 4); Serial.print(",");
Serial.print(q1, 4); Serial.print(",");
Serial.print(q2, 4); Serial.print(",");
Serial.print(q3, 4); Serial.print(",");
Serial.print(roll * RAD_TO_DEG, 2); Serial.print(",");
Serial.print(pitch * RAD_TO_DEG, 2); Serial.print(",");
Serial.println(yaw * RAD_TO_DEG, 2);
```

## 🐛 Troubleshooting

### Problem: "Sensor-Initialisierung fehlgeschlagen"

**Lösung:**
- Board neu anschließen
- Arduino IDE neu starten
- `Arduino_BMI270_BMM150` Library überprüfen
- Board-Definition aktualisieren (`Tools → Board → Boards Manager`)

### Problem: Starkes Drift im Yaw

**Ursache:** Magnetische Störungen oder schlechte Kalibrierung

**Lösung:**
- Kalibrierung wiederholen (weg von Metall/Magneten)
- Auf 6-DOF Modus umschalten (`useMagnetometer = false`)
- Beta-Parameter erhöhen (z.B. `0.2`)

### Problem: Ruckelige Orientierung

**Lösung:**
- Beta-Parameter verringern (z.B. `0.05`)
- Sample Rate erhöhen (z.B. `200 Hz`)
- Kalibrierung überprüfen

### Problem: Gimbal Lock

**Antwort:** Sollte nicht auftreten! Quaternionen vermeiden Gimbal Lock.
Bei Euler-Ausgabe kann es bei ±90° Pitch zu numerischen Artefakten kommen (erwartet).

## 📖 Ressourcen

### Papers & Dokumentation

- **Madgwick Filter:** [Original Paper (2010)](https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf)
- **BMI270 Datasheet:** [Bosch Sensortec](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/)
- **BMM150 Datasheet:** [Bosch Sensortec](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm150/)
- **Quaternions:** [3Blue1Brown Video](https://www.youtube.com/watch?v=zjMuIxRvygQ)

### Alternative AHRS-Filter

| Filter   | Vorteile                 | Nachteile              | CPU-Last |
|----------|--------------------------|------------------------|----------|
| Madgwick | ⭐ Schnell, stabil       | Keine Adaptivität     | Niedrig  |
| Mahony   | PI-Controller, präzise   | Mehr Tuning nötig     | Niedrig  |
| Kalman   | Optimal bei Rauschen     | Komplex, höhere Last  | Hoch     |

## 🎯 Anwendungsfälle

- 🎮 **VR/AR Head-Tracking**
- 🤖 **Roboter-Orientierung**
- ✈️ **Drohnen-Flugsteuerung**
- 📱 **Motion Capture**
- 🎲 **Würfel-Tracking für Games**
- 🧭 **Kompass mit Neigungskompensation**

## 📝 Lizenz

Open Source - frei verwendbar für private und kommerzielle Projekte.

Basierend auf:
- Madgwick AHRS (Sebastian Madgwick, 2010)
- Fast Inverse Square Root (Quake III Arena, id Software)

## 👨‍💻 Support

Bei Problemen:
1. Serial Monitor auf 115200 Baud prüfen
2. Kalibrierung neu durchführen
3. Troubleshooting-Sektion konsultieren
4. GitHub Issues erstellen (falls vorhanden)

---

**Happy Tracking! 🚀**
