/*
 * Arduino Portenta H7 Lite BLE Debug Script
 * 
 * Testet BLE-Verbindung und sendet Testdaten mit Timestamps
 * für Latenzmessung
 * 
 * Board: Arduino Portenta H7 (M7 core)
 * Library: Arduino Mbed OS Portenta Boards (neueste Version)
 */

#include <WiFi.h>  // Enthält BLE für Portenta H7

// BLE Service & Characteristic
BLEService testService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic testChar("19B10001-E8F2-537E-4F6C-D104768A1214", 
                           BLERead | BLENotify, 512);

// Variablen
unsigned long lastSend = 0;
unsigned long sendInterval = 1000; // 1 Sekunde
int messageCount = 0;
bool clientConnected = false;

void setup() {
  Serial.begin(115200);
  delay(2000);  // Portenta braucht etwas Zeit
  
  Serial.println("=== Arduino Portenta H7 BLE Debug ===");
  Serial.println("Initialisiere BLE...");
  
  // BLE initialisieren
  if (!BLE.begin()) {
    Serial.println("❌ BLE Start fehlgeschlagen!");
    Serial.println("   Stelle sicher, dass:");
    Serial.println("   - Mbed OS Portenta Core installiert ist");
    Serial.println("   - Board: Portenta H7 (M7 core) ausgewählt ist");
    while (1);
  }
  
  // Device Name setzen
  BLE.setLocalName("JET-DEBUG");
  BLE.setDeviceName("JET-DEBUG");
  
  // Service hinzufügen
  BLE.setAdvertisedService(testService);
  testService.addCharacteristic(testChar);
  BLE.addService(testService);
  
  // Initial Wert setzen
  testChar.writeValue("");
  
  // Advertising starten
  BLE.advertise();
  
  Serial.println("✅ BLE bereit!");
  Serial.println("───────────────────────────────────");
  Serial.print("Device Name:    JET-DEBUG\n");
  Serial.print("MAC Adresse:    ");
  Serial.println(BLE.address());
  Serial.println("Service UUID:   19B10000-E8F2-537E-4F6C-D104768A1214");
  Serial.println("Char UUID:      19B10001-E8F2-537E-4F6C-D104768A1214");
  Serial.println("───────────────────────────────────");
  Serial.println("\n💡 Kopiere die MAC Adresse in deine config.yml!\n");
  Serial.println("Warte auf Verbindung...\n");
}

void loop() {
  // Auf Client warten
  BLEDevice central = BLE.central();
  
  // Client verbunden?
  if (central) {
    if (!clientConnected) {
      clientConnected = true;
      Serial.println("───────────────────────────────────");
      Serial.print("✅ Client verbunden: ");
      Serial.println(central.address());
      Serial.println("───────────────────────────────────");
      Serial.println("Sende Testdaten...\n");
    }
    
    // Solange verbunden
    while (central.connected()) {
      unsigned long now = millis();
      
      // Daten senden (alle sendInterval ms)
      if (now - lastSend >= sendInterval) {
        lastSend = now;
        sendTestData(now);
      }
      
      // Kurze Pause
      delay(10);
    }
    
    // Verbindung getrennt
    if (clientConnected) {
      clientConnected = false;
      Serial.println("\n───────────────────────────────────");
      Serial.println("❌ Client getrennt");
      Serial.println("───────────────────────────────────");
      Serial.println("\nWarte auf neue Verbindung...\n");
      messageCount = 0;
    }
  }
}

void sendTestData(unsigned long timestamp) {
  messageCount++;
  
  // JSON erstellen (ohne Library für minimale Latenz)
  // Simuliert echte Sensor-Daten
  String json = "{";
  json += "\"msg_id\":" + String(messageCount) + ",";
  json += "\"arduino_time\":" + String(timestamp) + ",";
  json += "\"temperature\":" + String(23.5 + (random(-10, 10) / 10.0)) + ",";
  json += "\"pressure\":" + String(1013.25 + (random(-50, 50) / 10.0)) + ",";
  json += "\"gyro_x\":" + String(random(-100, 100) / 100.0) + ",";
  json += "\"gyro_y\":" + String(random(-100, 100) / 100.0) + ",";
  json += "\"gyro_z\":" + String(random(-100, 100) / 100.0) + ",";
  json += "\"accel_x\":" + String(random(-50, 50) / 100.0) + ",";
  json += "\"accel_y\":" + String(9.81 + (random(-20, 20) / 100.0)) + ",";
  json += "\"accel_z\":" + String(random(-50, 50) / 100.0) + ",";
  json += "\"mag_x\":" + String(45.2 + random(-50, 50)) + ",";
  json += "\"mag_y\":" + String(-12.8 + random(-30, 30)) + ",";
  json += "\"mag_z\":" + String(38.1 + random(-40, 40)) + ",";
  json += "\"battery\":" + String(87 - (messageCount % 100));
  json += "}";
  
  // Via BLE senden
  testChar.writeValue(json.c_str());
  
  // Debug Ausgabe (alle 5 Nachrichten)
  if (messageCount % 5 == 0) {
    Serial.print("📤 Nachrichten gesendet: ");
    Serial.print(messageCount);
    Serial.print(" | Größe: ");
    Serial.print(json.length());
    Serial.print(" Bytes | Uptime: ");
    Serial.print(timestamp / 1000);
    Serial.println("s");
  }
}
