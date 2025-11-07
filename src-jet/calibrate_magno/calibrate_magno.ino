/*
 *  HARD-IRON KALIBRIERUNG
 *
 *  Misst die Magnetometer Offsets 
 *  -> bedingt durch mechanische Bauteile
 *  -> Metalle, Stromversorgung, etc.
 *  Muss einmal durchgeführt werden -> Werte auf dem EPROM gespeichert
 */


#include "Arduino_BMI270_BMM150.h"
#include <NanoBLEFlashPrefs.h>

// Struktur für gespeicherte Kalibrierungswerte
typedef struct {
  float magOffsetX;
  float magOffsetY;
  float magOffsetZ;
  bool isCalibrated;
} MagCalibration;

MagCalibration magCal;
NanoBLEFlashPrefs myFlashPrefs;

// Kalibrierungs-Variablen
float magXmin = 0, magXmax = 0;
float magYmin = 0, magYmax = 0;
float magZmin = 0, magZmax = 0;
bool firstReading = true;

unsigned long calibrationStartTime;
const unsigned long CALIBRATION_DURATION = 120000; // 2 Minuten

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("=== Magnetometer Kalibrierung ===");
  Serial.println("Arduino Nano 33 BLE Sense Rev2");
  Serial.println();
  
  if (!IMU.begin()) {
    Serial.println("FEHLER: IMU konnte nicht initialisiert werden!");
    while (1);
  }
  
  Serial.println("IMU initialisiert");
  Serial.println();
  Serial.println("ANLEITUNG:");
  Serial.println("1. Sende 'c' um Kalibrierung zu starten");
  Serial.println("2. Sende 'l' um gespeicherte Werte anzuzeigen");
  Serial.println("3. Sende 'd' um gespeicherte Werte zu löschen");
  Serial.println();
  
  // Versuche gespeicherte Kalibrierung zu laden
  loadCalibration();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'c' || cmd == 'C') {
      startCalibration();
    } else if (cmd == 'l' || cmd == 'L') {
      displayCalibration();
    } else if (cmd == 'd' || cmd == 'D') {
      deleteCalibration();
    }
  }
}

void startCalibration() {
  Serial.println();
  Serial.println("=== KALIBRIERUNG GESTARTET ===");
  Serial.println("Dauer: 2 Minuten");
  Serial.println();
  Serial.println("WICHTIG: Rotiere das Board jetzt in ALLE Richtungen!");
  Serial.println("- Langsame, gleichmäßige Bewegungen");
  Serial.println("- Alle Achsen abdecken (pitch, roll, yaw)");
  Serial.println("- Am besten: Figure-8-Bewegungen in verschiedenen Ebenen");
  Serial.println();
  
  firstReading = true;
  calibrationStartTime = millis();
  
  while (millis() - calibrationStartTime < CALIBRATION_DURATION) {
    if (IMU.magneticFieldAvailable()) {
      float mx, my, mz;
      IMU.readMagneticField(mx, my, mz);
      
      // Erste Messung als Startwerte
      if (firstReading) {
        magXmin = magXmax = mx;
        magYmin = magYmax = my;
        magZmin = magZmax = mz;
        firstReading = false;
      }
      
      // Min/Max tracken
      if (mx < magXmin) magXmin = mx;
      if (mx > magXmax) magXmax = mx;
      if (my < magYmin) magYmin = my;
      if (my > magYmax) magYmax = my;
      if (mz < magZmin) magZmin = mz;
      if (mz > magZmax) magZmax = mz;
      
      // Fortschritt alle 5 Sekunden
      if ((millis() - calibrationStartTime) % 5000 < 20) {
        unsigned long remaining = (CALIBRATION_DURATION - (millis() - calibrationStartTime)) / 1000;
        Serial.print("Verbleibend: ");
        Serial.print(remaining);
        Serial.println(" Sekunden - Weiter rotieren!");
      }
    }
    delay(20);
  }
  
  // Berechne Hard-Iron-Offsets (Mitte der Min/Max-Werte)
  magCal.magOffsetX = (magXmin + magXmax) / 2.0;
  magCal.magOffsetY = (magYmin + magYmax) / 2.0;
  magCal.magOffsetZ = (magZmin + magZmax) / 2.0;
  magCal.isCalibrated = true;
  
  // Speichere im Flash
  saveCalibration();
  
  Serial.println();
  Serial.println("=== KALIBRIERUNG ABGESCHLOSSEN ===");
  displayCalibration();
  Serial.println();
  Serial.println("Die Werte wurden dauerhaft gespeichert.");
  Serial.println("Sie bleiben auch nach Neustart erhalten!");
}

void saveCalibration() {
  int rc = myFlashPrefs.writePrefs(&magCal, sizeof(magCal));
  
  if (rc == 0) {
    Serial.println("✓ Kalibrierung erfolgreich im Flash gespeichert");
  } else {
    Serial.print("✗ FEHLER beim Speichern: ");
    Serial.println(myFlashPrefs.errorString(rc));
  }
}

void loadCalibration() {
  int rc = myFlashPrefs.readPrefs(&magCal, sizeof(magCal));
  
  if (rc == 0 && magCal.isCalibrated) {
    Serial.println("✓ Gespeicherte Kalibrierung geladen:");
    displayCalibration();
  } else {
    Serial.println("⚠ Keine Kalibrierung gefunden - bitte kalibrieren!");
    magCal.magOffsetX = 0;
    magCal.magOffsetY = 0;
    magCal.magOffsetZ = 0;
    magCal.isCalibrated = false;
  }
}

void displayCalibration() {
  Serial.println();
  Serial.println("--- Aktuelle Kalibrierwerte ---");
  Serial.print("X-Offset: "); Serial.print(magCal.magOffsetX, 2); Serial.println(" µT");
  Serial.print("Y-Offset: "); Serial.print(magCal.magOffsetY, 2); Serial.println(" µT");
  Serial.print("Z-Offset: "); Serial.print(magCal.magOffsetZ, 2); Serial.println(" µT");
  Serial.print("Status: "); Serial.println(magCal.isCalibrated ? "Kalibriert" : "Nicht kalibriert");
  
  if (magCal.isCalibrated) {
    Serial.println();
    Serial.println("Code für Hauptprogramm:");
    Serial.print("float magOffsetX = "); Serial.print(magCal.magOffsetX, 2); Serial.println(";");
    Serial.print("float magOffsetY = "); Serial.print(magCal.magOffsetY, 2); Serial.println(";");
    Serial.print("float magOffsetZ = "); Serial.print(magCal.magOffsetZ, 2); Serial.println(";");
  }
  Serial.println();
}

void deleteCalibration() {
  // VORSICHT:
  // EEPROM Werte nicht löschen!
  magCal.magOffsetX = 0;
  magCal.magOffsetY = 0;
  magCal.magOffsetZ = 0;
  magCal.isCalibrated = false;
  
  int rc = myFlashPrefs.deletePrefs();
  
  Serial.println();
  Serial.println("✓ Kalibrierung gelöscht");
  Serial.println();
}
