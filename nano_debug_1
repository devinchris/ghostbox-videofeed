/*
 * Arduino Nano 33 BLE Sense Rev2 - 9-Achsen Rotations-Tracker (Verbessert)
 * Nutzt offizielle MadgwickAHRS Library
 * 
 * INSTALLATION:
 * Arduino IDE -> Tools -> Manage Libraries -> "MadgwickAHRS" installieren
 */

#include <Arduino_BMI270_BMM150.h>
#include <MadgwickAHRS.h>

// Madgwick Filter Objekt
Madgwick filter;

// Sample Rate
const float sampleRate = 104.0f; // Hz - BMI270 Standard Rate
unsigned long lastUpdate = 0;

// Kalibrierungsdaten
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;
float magScaleX = 1.0f, magScaleY = 1.0f, magScaleZ = 1.0f;

bool gyroCalibrated = false;
bool magCalibrated = false;
bool outputEnabled = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("Arduino Nano 33 BLE Sense Rev2 - 9-Achsen Rotation v2");
  Serial.println("======================================================\n");
  
  if (!IMU.begin()) {
    Serial.println("FEHLER: IMU konnte nicht initialisiert werden!");
    while (1);
  }
  
  // Madgwick Filter initialisieren
  filter.begin(sampleRate);
  
  Serial.println("IMU erfolgreich initialisiert!");
  Serial.print("Sample Rate: "); Serial.print(sampleRate); Serial.println(" Hz");
  
  Serial.println("\n=== WICHTIG: KALIBRIERUNG ERFORDERLICH ===");
  Serial.println("Befehle:");
  Serial.println("  'g' - Gyro kalibrieren (Sensor FLACH und RUHIG halten!)");
  Serial.println("  'm' - Magnetometer kalibrieren (Sensor rotieren!)");
  Serial.println("  'b' - Beta-Wert ändern (Filterstärke)");
  Serial.println("  'r' - Rotation ausgeben (an/aus)");
  Serial.println("  's' - Status anzeigen\n");
  
  // Initiale Gyro-Kalibrierung (grob)
  Serial.println("Führe initiale Gyro-Kalibrierung durch...");
  delay(1000);
  calibrateGyroQuick();
  
  lastUpdate = micros();
}

void loop() {
  // Kommandos verarbeiten
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while(Serial.available()) Serial.read(); // Buffer leeren
    
    if (cmd == 'g') calibrateGyro();
    else if (cmd == 'm') calibrateMagnetometer();
    else if (cmd == 'r') {
      outputEnabled = !outputEnabled;
      Serial.print("Ausgabe: ");
      Serial.println(outputEnabled ? "AN" : "AUS");
    }
    else if (cmd == 'b') adjustBeta();
    else if (cmd == 's') showStatus();
  }
  
  // Sensordaten lesen und verarbeiten
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);
    
    // Gyro-Offset anwenden (wichtig für Drift-Reduktion)
    gx -= gyroOffsetX;
    gy -= gyroOffsetY;
    gz -= gyroOffsetZ;
    
    // Accel-Offset anwenden
    ax -= accelOffsetX;
    ay -= accelOffsetY;
    az -= accelOffsetZ;
    
    // Magnetometer Hard-Iron & Soft-Iron Korrektur
    mx = (mx - magOffsetX) * magScaleX;
    my = (my - magOffsetY) * magScaleY;
    mz = (mz - magOffsetZ) * magScaleZ;
    
    // WICHTIG: Achsen-Orientierung für BMI270/BMM150
    // Die Sensoren haben möglicherweise unterschiedliche Orientierungen
    // Diese Werte müssen eventuell invertiert werden
    filter.update(
      gx, gy, gz,           // Gyro in deg/s
      ax, ay, az,           // Accel in g
      mx, my, mz            // Mag in uT
    );
    
    // Zeit-Delta berechnen (für Debugging)
    unsigned long now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
    float actualRate = 1.0f / dt;
    
    // Rotation ausgeben
    if (outputEnabled) {
      float roll = filter.getRoll();
      float pitch = filter.getPitch();
      float yaw = filter.getYaw();
      
      Serial.print("Roll: "); 
      Serial.print(roll, 1);
      Serial.print("° | Pitch: "); 
      Serial.print(pitch, 1);
      Serial.print("° | Yaw: "); 
      Serial.print(yaw, 1);
      Serial.print("° | Rate: ");
      Serial.print(actualRate, 0);
      Serial.println(" Hz");
    }
  }
  
  // Minimales Delay für stabile Loop-Rate
  delayMicroseconds(500);
}

void calibrateGyroQuick() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    float gx, gy, gz;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      sumX += gx;
      sumY += gy;
      sumZ += gz;
    }
    delay(10);
  }
  
  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;
  gyroCalibrated = true;
  
  Serial.println("Initiale Gyro-Kalibrierung abgeschlossen.\n");
}

void calibrateGyro() {
  Serial.println("\n=== GYRO KALIBRIERUNG ===");
  Serial.println("WICHTIG:");
  Serial.println("1. Sensor auf EBENE FLÄCHE legen");
  Serial.println("2. Sensor NICHT bewegen!");
  Serial.println("3. Kalibrierung läuft 5 Sekunden...");
  delay(2000);
  Serial.println("START!");
  
  float sumX = 0, sumY = 0, sumZ = 0;
  float sumAx = 0, sumAy = 0, sumAz = 0;
  int samples = 500;
  
  for (int i = 0; i < samples; i++) {
    float gx, gy, gz, ax, ay, az;
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      IMU.readAcceleration(ax, ay, az);
      
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      sumAx += ax;
      sumAy += ay;
      sumAz += az;
      
      if (i % 100 == 0) Serial.print(".");
    }
    delay(10);
  }
  
  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;
  
  // Accel-Offset (Z-Achse sollte ~1g sein bei flacher Lage)
  accelOffsetX = sumAx / samples;
  accelOffsetY = sumAy / samples;
  accelOffsetZ = (sumAz / samples) - 1.0f; // -1g Schwerkraft
  
  gyroCalibrated = true;
  
  Serial.println("\n\nGyro kalibriert!");
  Serial.print("Gyro Offsets  - X: "); Serial.print(gyroOffsetX, 4);
  Serial.print(" | Y: "); Serial.print(gyroOffsetY, 4);
  Serial.print(" | Z: "); Serial.println(gyroOffsetZ, 4);
  Serial.print("Accel Offsets - X: "); Serial.print(accelOffsetX, 4);
  Serial.print(" | Y: "); Serial.print(accelOffsetY, 4);
  Serial.print(" | Z: "); Serial.println(accelOffsetZ, 4);
  
  // Filter zurücksetzen nach Kalibrierung
  filter.begin(sampleRate);
  Serial.println("Filter zurückgesetzt.\n");
}

void calibrateMagnetometer() {
  Serial.println("\n=== MAGNETOMETER KALIBRIERUNG ===");
  Serial.println("WICHTIG:");
  Serial.println("1. Sensor in alle Richtungen drehen");
  Serial.println("2. Langsame 8er-Bewegung durch ALLE Achsen");
  Serial.println("3. Weg von Metall/Magneten/Elektronik!");
  Serial.println("4. 15 Sekunden Zeit...");
  delay(3000);
  Serial.println("START!");
  
  float minX = 10000, maxX = -10000;
  float minY = 10000, maxY = -10000;
  float minZ = 10000, maxZ = -10000;
  
  unsigned long startTime = millis();
  int sampleCount = 0;
  
  while (millis() - startTime < 15000) {
    float mx, my, mz;
    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(mx, my, mz);
      
      if (mx < minX) minX = mx;
      if (mx > maxX) maxX = mx;
      if (my < minY) minY = my;
      if (my > maxY) maxY = my;
      if (mz < minZ) minZ = mz;
      if (mz > maxZ) maxZ = mz;
      
      sampleCount++;
      
      // Fortschritt anzeigen
      if ((millis() - startTime) % 1000 < 20) {
        Serial.print(".");
      }
    }
    delay(10);
  }
  
  Serial.println("\n");
  Serial.print("Samples gesammelt: "); Serial.println(sampleCount);
  
  // Hard-Iron Offset (Zentrierung)
  magOffsetX = (maxX + minX) / 2.0f;
  magOffsetY = (maxY + minY) / 2.0f;
  magOffsetZ = (maxZ + minZ) / 2.0f;
  
  // Soft-Iron Korrektur (Skalierung für elliptische Kalibrierung)
  float avgRange = ((maxX - minX) + (maxY - minY) + (maxZ - minZ)) / 3.0f;
  magScaleX = avgRange / (maxX - minX);
  magScaleY = avgRange / (maxY - minY);
  magScaleZ = avgRange / (maxZ - minZ);
  
  magCalibrated = true;
  
  Serial.println("Magnetometer kalibriert!");
  Serial.println("\nRaw Ranges:");
  Serial.print("X: "); Serial.print(minX); Serial.print(" bis "); Serial.println(maxX);
  Serial.print("Y: "); Serial.print(minY); Serial.print(" bis "); Serial.println(maxY);
  Serial.print("Z: "); Serial.print(minZ); Serial.print(" bis "); Serial.println(maxZ);
  
  Serial.println("\nKorrekturwerte:");
  Serial.print("Offsets - X: "); Serial.print(magOffsetX, 2);
  Serial.print(" | Y: "); Serial.print(magOffsetY, 2);
  Serial.print(" | Z: "); Serial.println(magOffsetZ, 2);
  Serial.print("Scales  - X: "); Serial.print(magScaleX, 4);
  Serial.print(" | Y: "); Serial.print(magScaleY, 4);
  Serial.print(" | Z: "); Serial.println(magScaleZ, 4);
  
  // Filter zurücksetzen nach Kalibrierung
  filter.begin(sampleRate);
  Serial.println("Filter zurückgesetzt.\n");
  Serial.println("Drücke 'r' um Rotation auszugeben!\n");
}

void adjustBeta() {
  Serial.println("\n=== BETA-WERT ANPASSEN ===");
  Serial.println("Beta steuert die Filterstärke:");
  Serial.println("  Niedriger (0.01-0.1): Langsamer, stabiler, weniger Drift");
  Serial.println("  Höher (0.5-2.0):      Schneller, reaktiver, mehr Drift");
  Serial.println("\nOptionen:");
  Serial.println("  1 - Beta = 0.033 (sehr stabil, Standard)");
  Serial.println("  2 - Beta = 0.1 (ausgewogen)");
  Serial.println("  3 - Beta = 0.3 (schnell)");
  Serial.println("  4 - Beta = 1.0 (sehr schnell)");
  Serial.print("\nWähle (1-4): ");
  
  while (!Serial.available());
  char choice = Serial.read();
  while(Serial.available()) Serial.read();
  
  float newBeta = 0.033f;
  switch(choice) {
    case '1': newBeta = 0.033f; break;
    case '2': newBeta = 0.1f; break;
    case '3': newBeta = 0.3f; break;
    case '4': newBeta = 1.0f; break;
    default: 
      Serial.println("Ungültige Wahl!");
      return;
  }
  
  filter.begin(sampleRate);
  // MadgwickAHRS Library hat kein setBeta(), nutzt internen Standardwert
  // Für erweiterte Kontrolle müsste man die Library modifizieren
  
  Serial.println(newBeta, 3);
  Serial.println("Hinweis: Standard MadgwickAHRS nutzt festen Beta-Wert.");
  Serial.println("Für variable Beta-Werte modifizierte Library verwenden.\n");
}

void showStatus() {
  Serial.println("\n=== STATUS ===");
  Serial.print("Gyro kalibriert: ");
  Serial.println(gyroCalibrated ? "JA" : "NEIN");
  Serial.print("Magnetometer kalibriert: ");
  Serial.println(magCalibrated ? "JA" : "NEIN");
  Serial.print("Ausgabe aktiv: ");
  Serial.println(outputEnabled ? "JA" : "NEIN");
  Serial.print("Sample Rate: ");
  Serial.print(sampleRate);
  Serial.println(" Hz");
  
  if (gyroCalibrated) {
    Serial.println("\nGyro Offsets:");
    Serial.print("  X: "); Serial.print(gyroOffsetX, 4);
    Serial.print(" | Y: "); Serial.print(gyroOffsetY, 4);
    Serial.print(" | Z: "); Serial.println(gyroOffsetZ, 4);
  }
  
  if (magCalibrated) {
    Serial.println("\nMag Korrektur:");
    Serial.print("  Offset X: "); Serial.print(magOffsetX, 2);
    Serial.print(" | Y: "); Serial.print(magOffsetY, 2);
    Serial.print(" | Z: "); Serial.println(magOffsetZ, 2);
    Serial.print("  Scale  X: "); Serial.print(magScaleX, 4);
    Serial.print(" | Y: "); Serial.print(magScaleY, 4);
    Serial.print(" | Z: "); Serial.println(magScaleZ, 4);
  }
  Serial.println();
}
