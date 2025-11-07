/*
 *    ARDUINO NANO 33 BLE REV2
 *
 *    BLE Sensordaten live senden
 */

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
// #include Library zum Glätten

// ========== BLE KONFIGURATION ==========

// UUIDS
#define SERVICE_UUID        "19B10000-E8F2-537E-4F6C-D104768A1214"
#define SENSOR_CHAR_UUID    "19B10001-E8F2-537E-4F6C-D104768A1214"

// BLE Service
BLEService sensorService(SERVICE_UUID);
BLECharacteristic sensorCharacteristic(
  SENSOR_CHAR_UUID,
  BLENotify,           // Nur Notifications 
  244                  // Max 244 bytes
);

// ========== TIMING KONFIGURATION ==========

// Ziel-Frequenz: 60 Hz = 16.67ms pro Sample
#define TARGET_SAMPLE_RATE_HZ    60
#define SAMPLE_INTERVAL_US       (1000000 / TARGET_SAMPLE_RATE_HZ)  // 16667 micorseconds
#define ACCEL_SAMPLE_INTERVAL    120                                // TODO: Optimal finden
#define GYRO_SAMPLE_INTERVAL     120                                // TODO: SAMPLE RATE implementieren

// 1 Sample pro BLE-Notification
// -> Sendefrequenz: 60 Hz, ergo 16.67ms
#define SAMPLES_PER_PACKET       1
#define PACKET_INTERVAL_US       SAMPLE_INTERVAL_US

#define DEVICE_NAME              "ARDUINO-NANO-JET-0001"

// ========== DATENSTRUKTUREN ==========

// Single Sensor Sample (32 Bytes)
struct SensorSample {
  uint32_t timestamp_ms;           // in ms
  float accelX, accelY, accelZ;   // in m/s
  float gyroX, gyroY, gyroZ;      // in Grad
  float magX, magY, magZ;         // in Tesla     TODO: DEBUG MAGNETOMETER Hat das Kackteil nur 20Hz?
  float temperature;              // in Celsius
} __attribute__((packed));        // Verhindert Padding


struct SensorPacket {
  uint16_t packet_id;
  uint8_t sample_count;       // Anzahl samples in einem Packet
  uint8_t reserved;           // Padding Alignment
  SensorSample samples[SAMPLES_PER_PACKET];  // ~88 Bytes (oder so ähnlich)
} __attribute__((packed));

// ========== GLOBALE VARIABLEN ==========

SensorPacket currentPacket;
uint8_t sampleIndex = 0;           // Index im aktuellen Paket (0-1)
uint16_t packetCounter = 0;        // Paket-ID für Tracking

unsigned long lastSampleTime = 0;  // Letzter Sample-Zeitpunkt (µs)
unsigned long lastPacketTime = 0;  // Letzter Sende-Zeitpunkt (µs)

// Statistiken für Monitoring
unsigned long totalSamples = 0;
unsigned long missedDeadlines = 0;
bool isConnected = false;

// ===========================
// ========== SETUP ==========
// ===========================

void setup() {
  Serial.begin(115200);

  // LED als Statusanzeige
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  while (!Serial && millis() < 3000);  // Max 3s auf Serial warten -> im Debug
  
  Serial.println("=== Arduino Nano 33 BLE - HENSOLDTS BESTES PRODUKT JEMALS ===");
  
  // IMU initialisieren
  if (!IMU.begin()) {
    Serial.println("ERROR: IMU Initialisierung fehlgeschlagen!");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);   // LED blinkt schnell -> Auslesen der Sensoren fehlgeschlagen
    }
  }
  Serial.println("✓ IMU initialized");
  Serial.print("  Accel sample rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("  Gyro sample rate: ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  
  // BLE initialisieren
  bleinit();
  
  // Timing initialisieren
  lastSampleTime = micros();
  lastPacketTime = micros();
}

// ============ BLE CONFIG ===========

void bleinit(){
  if (!BLE.begin()) {
    Serial.println("ERROR: BLE initialisierung fehlgeschlagen!");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(1000);    // LED blinkt langsam -> BLE initialisierung fehlgeschlagen
    }
  }
  Serial.println("✓ BLE initialisiert");
  
  // BLE konfigurieren
  BLE.setLocalName("Arduino-Sensor");
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(sensorService);
  
  // Characteristic zum Service hinzufügen
  sensorService.addCharacteristic(sensorCharacteristic);
  BLE.addService(sensorService);
  
  // Min: 7.5ms (0x0006), Max: 15ms (0x000C)
  // Werte in 1.25ms Schritten
  BLE.setConnectionInterval(0x0006, 0x000C);
  
  // Advertising starten
  BLE.advertise();
  Serial.println("✓ BLE advertising gestartet");
  Serial.println("\nWarte auf Verbindung...");
  Serial.print("Device Name: ");
  Serial.println(DEVICE_NAME)
  Serial.println("Service UUID: " SERVICE_UUID);
}

// ===================================
// ============ MAIN LOOP ============
// ===================================

void loop() {
  // Auf BLE-Verbindung warten/prüfen
  BLEDevice central = BLE.central();
  
  if (central) {
    if (!isConnected) {
      // Neu verbunden
      isConnected = true;
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("\n✓ Verbunden mit: " + central.address());
      Serial.println("Starte stream...\n");
      
      // Timing zurücksetzen
      lastSampleTime = micros();
      lastPacketTime = micros();
      sampleIndex = 0;
      packetCounter = 0;
      totalSamples = 0;
      missedDeadlines = 0;
    }
    
    // =====================================
    // =========== SAMPLING LOOP ===========
    // =====================================
    while (central.connected()) {
      unsigned long currentTime = micros();
      
      // Prüfe ob es Zeit für nächstes Sample ist
      if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_US) {
        
        // Timing-Drift prüfen (für Statistik)
        long drift = (currentTime - lastSampleTime) - SAMPLE_INTERVAL_US;
        if (drift > 1000) {  // > 1ms Drift
          missedDeadlines++;
        }
        
        // Sample aufnehmen
        readSensorData(sampleIndex);
        sampleIndex++;
        totalSamples++;
        
        // Nächsten Sample-Zeitpunkt berechnen (kumulativ für präzises Timing)
        lastSampleTime += SAMPLE_INTERVAL_US;
        
        // Paket voll? -> Senden!
        if (sampleIndex >= SAMPLES_PER_PACKET) {
          sendPacket();
          sampleIndex = 0;
        }
      }
      
      // Statistik alle 5 Sekunden ausgeben
      static unsigned long lastStatsTime = 0;
      if (millis() - lastStatsTime >= 5000) {
        printStatistics();
        lastStatsTime = millis();
      }
      
      // Pause um CPU nicht zu blockieren (busy-wait vermeiden)
      delayMicroseconds(100);
    }
    
    // Verbindung getrennt
    isConnected = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("\n✗ Verbindung getrennt: " + central.address());
    Serial.println("Warte auf neue Verbindung...\n");
  }
}

// ==========================================               TODO:
// ========== SENSORDATEN AUSLESEN ==========               Sensordaten glätten lassen
// ==========================================               A$AP  -  Zeit ist Geld
void readSensorData(uint8_t index) {
  // Sample-init
  SensorSample* sample = &currentPacket.samples[index];
  
  // Zeitstempel setzen
  sample->timestamp_ms = millis();
  
  // Beschleunigungssensor auslesen
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(sample->accelX, sample->accelY, sample->accelZ);
  } else {
    // Fallback bei fehlendem Sample
    sample->accelX = sample->accelY = sample->accelZ = 0.0f;
  }
  
  // Gyroskop auslesen
  // TODO: Mittels Madgwick Filter Winkelgeschwindigkeit in Winkel umrechnen
  // TODO: GLÄTTUNG EINBAUEN
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(sample->gyroX, sample->gyroY, sample->gyroZ);
  } else {
    sample->gyroX = sample->gyroY = sample->gyroZ = 0.0f;
  }
  
  // Magnetometer auslesen
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(sample->magX, sample->magY, sample->magZ);
  } else {
    sample->magX = sample->magY = sample->magZ = 0.0f;
  }
}

// =======================================
// ============ PAKETE SENDEN ============
// =======================================

void sendPacket() {
  // Paket-Header befüllen
  currentPacket.packet_id = packetCounter++;
  currentPacket.sample_count = SAMPLES_PER_PACKET;
  currentPacket.reserved = 0;
  
  // Via BLE senden
  bool success = sensorCharacteristic.writeValue(
    (uint8_t*)&currentPacket, 
    sizeof(SensorPacket)
  );
  
  if (!success) {
    Serial.println("⚠ BLE WRITE fehlgeschlagen");
  }
  
  lastPacketTime = micros();
}

// ========================================         INFO: NUR FÜR DEBUG
// ========== STATISTIK AUSGEBEN ==========         TODO: LÖSCHEN WENN FERTIG
// ========================================         TODO: BESTE METHODE RAUSFINDEN (ig notifications)

void printStatistics() {
  float actualSampleRate = totalSamples / 5.0f;  // Samples pro Sekunde
  float missRate = (missedDeadlines * 100.0f) / totalSamples;
  
  Serial.println("--- Statistiken (5s Zeitfenster) ---");
  Serial.print("  Actual sample rate: ");
  Serial.print(actualSampleRate, 2);
  Serial.println(" Hz");
  Serial.print("  Total samples: ");
  Serial.println(totalSamples);
  Serial.print("  Packets sent: ");
  Serial.println(packetCounter);
  Serial.print("  Missed deadlines: ");
  Serial.print(missedDeadlines);
  Serial.print(" (");
  Serial.print(missRate, 2);
  Serial.println("%)");
  Serial.println();
  
  // Counters zurücksetzen
  totalSamples = 0;
  missedDeadlines = 0;
}
