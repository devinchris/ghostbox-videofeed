#include <ArduinoBLE.h>         // Library für BLE Kommunikation

#include "RotationManager.h"    // GHOSTBOX-Library
// -> liest Sensordaten von IMU und Magno 
// -> Kalibriert Sensoren anfänglich
// -> berechnet Rotation via Madgwick AHRS
// -> Gibt berechnete Quaternion, Accel und Magno Werte zurück

#include "SmoothData4D.h"       // GHOSTBOX-Library
// -> Glättet Quaternionen Werte
// -> Filtert Vibrationen & Noise
// -> Optimiert für ARM FPU
// -> Deadzone um kleine Bewegungen zu ignorieren

#define FREQUENCY 25.0f     // Hz   Frequenz mit der Sensrodaten gemessen/gesendet werden
#define MAG_FREQUENCY 19.0f // Hz   Frequenz für Magnetometer Kalibrierung

// BLE Command Bytes -> Jeder Byte ist ein Befehl von Jet -> Raspberry und andersrum
const uint8_t CMD_RUNNING = 0x00;            // Jet bewegt sich und sendet Daten
const uint8_t CMD_IDLE = 0x01;               // Jet ist im IDLE Mode und sendet nichts
const uint8_t CMD_PLATFORM_DETECTED = 0x02;  // Jet war im IDLE und hat detected, dass er auf der Plattform steht
const uint8_t CMD_CALIBRATE_MAG = 0x03;      // Jet kalibriert sein Magnetometer (-> Figure 8 Bewegung!)

const uint8_t INFO_MAGNETMODE = 0x05;        // Info: AHRS Modus (um auf dem LCD anzuzeigen)

const uint8_t CMD_CAL_START = 0x30;          // Start Magnetometer Kalibrierung
const uint8_t CMD_CAL_STOP = 0x31;           // Stop Magnetometer Kalibrierung
const uint8_t CMD_CAL_MATRIX = 0x32;         // Erhalte Kalibrierungsmatrix
const uint8_t CMD_SWITCH_PRESET = 0x33;      // Wechsle Magnetometer Preset (STANDARD, INDIVIDUAL, IMU)

enum State {
  RUNNING,        // Main Loop - sendet Sensordaten
  CALIBRATING,    // Magnetometer Kalibrierung läuft
  DISCONNECTED,   // Keine BLE Verbindung
  IDLE,           // Jet ist im Leerlauf
  PLATFORM        // Jet steht auf der Plattform
};

State currentState = DISCONNECTED;

// Single Sensor Sample (32 Bytes)
// Sendet alle Sensordaten in einem struct via BLE
struct SensorValues {
  uint32_t timestamp_ms;        // in ms
  float accelX, accelY, accelZ; // in m/s
  float q0, q1, q2, q3;         // in Grad
  float magX, magY, magZ;       // in µTesla    
  float tempC;                  // in Celsius
} __attribute__((packed));      // Verhindert Padding

// Sendet nur Magnetometer Daten für Kalibrierung
struct MagnoValues {
  float mx, my, mz;             // in µTesla    
} __attribute__((packed));      // Verhindert Padding

struct CalibrationPayload {
  float softIron[3][3];         // 3x3 Matrix (row-major)
  float hardIron[3];            // Hard-Iron Offsets
} __attribute__((packed));

static const size_t CALIBRATION_FLOATS = 12;      // 3x3 Matrix + 3 Offsets
static const size_t CALIBRATION_PACKET_SIZE = CALIBRATION_FLOATS * sizeof(float);

CalibrationPayload latestCalibrationPayload = {};
bool calibrationPayloadReceived = false;

// === Globals === 
SensorValues sensorData;
MagnoValues magValues;
unsigned long lastSendTime;
float tempC;
int idlecount = 0;
static const int idleCountMax = 100;
bool idle = false;
float accelMag = 0.0f;
float magnetMag = 0.0f;
float gyroMag = 0.0f;
 
// BLE Service erstellen
BLEService myService("19B10000-E8F2-537E-4F6C-D104768A1214");


// === RotationManager ===
RotationManager sensor;
Quaternion _Quaternion;
float gx = 0, gy = 0, gz = 0;
float ax = 0, ay = 0, az = 0;
float mx = 0, my = 0, mz = 0;

float gx_bias[100], gy_bias[100], gz_bias[100];

// === SmoothData4D ===
SmoothQuaternionData smoothing;  


// === BLE Characteristics ===
                                  // Sensorwerte 
BLECharacteristic dataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLEWrite | BLENotify, sizeof(SensorValues));

                                   // Magnetometer Werte
BLECharacteristic magCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLENotify, sizeof(MagnoValues));

                                   // Commands (Idle, Platform, Kalibrierung)
BLECharacteristic controlCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLEWrite | BLENotify, 1); // Steuerungs-Charakteristik

BLECharacteristic commandCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLEWrite | BLEWriteWithoutResponse | BLENotify, CALIBRATION_PACKET_SIZE);

void handleCommandWrite(BLEDevice central, BLECharacteristic characteristic);
void logCalibrationPayload(const CalibrationPayload& payload);
void enterCalibrationMode();
void exitCalibrationMode();
 
void setup() {
  Serial.begin(115200);
 
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
 
  BLE.setConnectionInterval(6, 6);
  BLE.setAdvertisingInterval(160);
  // Suche findet via BLE Namen statt
  BLE.setLocalName("Eurofighter");
  BLE.setDeviceName("Eurofighter");
 
  BLE.setAdvertisedService(myService);
 
  // Add characteristic to service
  myService.addCharacteristic(dataCharacteristic);
  myService.addCharacteristic(magCharacteristic);
  myService.addCharacteristic(controlCharacteristic);
  myService.addCharacteristic(commandCharacteristic);
 
  // Add service
  BLE.addService(myService);
 
  // Set initial value
  dataCharacteristic.writeValue("Ready");
  commandCharacteristic.writeValue((byte*)&latestCalibrationPayload, CALIBRATION_PACKET_SIZE);
  commandCharacteristic.setEventHandler(BLEWritten, handleCommandWrite);
 
  // Starte advertising
  BLE.advertise();
 
  Serial.println("Bluetooth device active, waiting for connections...");

  sensor.init(FREQUENCY);

  // Initialisiere Smoothing mit Target-Frequenz und Alpha-Wert
  // Target-Frequenz sollte der Sensor-Update-Rate entsprechen (z.B. 100 Hz)
  // Alpha: 0.55 = mittlere Glättung (0.01 = sehr stark, 0.99 = fast keine Glättung)
  smoothing.initSmoothing(FREQUENCY, 0.7f);
}
 
void loop() {
  // Central (Raspberry) verbindet sich dann mit dem Nano
  BLEDevice central = BLE.central();

  // Frage nach Updates und neuen Notifications
  BLE.poll();

  // Check ob BLE Verbindung noch besteht
  if(currentState != DISCONNECTED && (!central || !central.connected())){
    currentState = DISCONNECTED;
  }

  // ===============================
  // ======== STATE MACHINE ========
  // ===============================
  switch(currentState) {
    // ==== RECONNECTING STATE ====
    case DISCONNECTED:
      if (central && central.connected()) {
        if(Serial) Serial.print("Connected to central: ");
        if(Serial) Serial.println(central.address());
        currentState = RUNNING;
        lastSendTime = millis();
      }
      break;
    
    // ==== MAGNETOMETER KALIBRIERUNG ====
    case CALIBRATING:
      if(!(millis() - lastSendTime >= 2000 / MAG_FREQUENCY)) {
        break;
      }
      if(central.connected() == false) {
        currentState = DISCONNECTED;
        break;
      }
      if(!IMU.magneticFieldAvailable()) {
        break;
      }
      lastSendTime = millis();
      IMU.readMagneticField(mx, my, mz);
      magValues = {mx, my, mz};
      // Sende nur Magnetometer Daten
      magCharacteristic.writeValue((byte*)&magValues, sizeof(float)*3);
      
      break;

    // =====================
    // ===== MAIN LOOP =====
    // ===================== 
    case RUNNING:
      if(central.connected() == false) {
        if(Serial) Serial.print("Disconnected from central: ");
        if(Serial) Serial.println(central.address());
        currentState = DISCONNECTED;
        break;
      }
      if(!(millis() - lastSendTime >= 1000 / FREQUENCY)) {
        break;
      }
      
      lastSendTime = millis();
      sensor.getCalculatedData(_Quaternion, gx, gy, gz, ax, ay, az, mx, my, mz, gyroMag);
      sensor.getTemperature(tempC);

      smoothing.smoothQuaternion(_Quaternion, millis());

      // SensorData Struct mit aktuellen Werten füllen
      sensorData.timestamp_ms = millis();
      sensorData.accelX = ax;
      sensorData.accelY = ay;
      sensorData.accelZ = az;
      sensorData.q0 = _Quaternion.q0;
      sensorData.q1 = _Quaternion.q1;
      sensorData.q2 = _Quaternion.q2;
      sensorData.q3 = _Quaternion.q3;
      sensorData.magX = mx;
      sensorData.magY = my;
      sensorData.magZ = mz;
      sensorData.tempC = tempC;
      
      dataCharacteristic.writeValue((byte*)&sensorData, sizeof(sensorData));

      // IDLE Detection
      accelMag = sqrt(ax*ax + ay*ay + (az-1)*(az-1));
      if(accelMag < 0.05f && gyroMag < 10.0f) {
        idlecount++;
        if(idlecount > idleCountMax){
          // Go IDLE
          currentState = IDLE;
          idlecount = 0;
        } else if(idlecount > (idleCountMax / 2)){
          sensor.setFilterGain(0.7f); // Korrektur des Gyro Drift/Bias im IDLE
        }
      }
      break;

    // ==== IDLE ====
    case IDLE:
      if(!idle){ 
        // Sendet eine IDLE Nachricht falls noch nicht geschehen
        controlCharacteristic.writeValue(&CMD_IDLE, 1);
        idle = true;
        idlecount = 0;
      }

      sensor.getCalculatedData(_Quaternion, gx, gy, gz, ax, ay, az, mx, my, mz, gyroMag);
      accelMag = sqrt(ax*ax + ay*ay + az*az);
      magnetMag = sqrt(mx*mx + my*my + mz*mz);

      Serial.print(magnetMag);

      // Aufnahme Detection
      if((accelMag - 1) > 0.05 || gyroMag > 15.0f){
        // Bewegung erkannt -> gehe zu state RUNNING
        controlCharacteristic.writeValue(&CMD_RUNNING, 1);
        idle = false;
        currentState = RUNNING;
        break;
      }

      if((magnetMag - 48) > 25 || (magnetMag - 48) < -25){
        // Elektromagnet auf der Platform erkannt -> gehe zu state PLATFORM
        controlCharacteristic.writeValue(&CMD_PLATFORM_DETECTED, 1);
        currentState = PLATFORM;
        delay(700); // Warte kurz, damit drehen der Platte nicht als RUNNING interpretiert wird
      }
      break;

    // ==== PLATFORM ====
    case PLATFORM:
      sensor.getCalculatedData(_Quaternion, gx, gy, gz, ax, ay, az, mx, my, mz, gyroMag);
      accelMag = sqrt(ax*ax + ay*ay + az*az);

      // Aufnahme Detection
      if((accelMag - 1) > 0.05 || gyroMag > 200.0f){   // DEBUG!
        // Bewegung erkannt -> gehe zu state RUNNING
        controlCharacteristic.writeValue(&CMD_RUNNING, 1);
        idle = false;
        currentState = RUNNING;
      }
      break;

  }
}

void handleCommandWrite(BLEDevice central, BLECharacteristic characteristic) {
  int packetLength = characteristic.valueLength();
  if (packetLength <= 0) {
    Serial.println("Command characteristic write with empty payload");
    return;
  }

  if (packetLength == 1) {
    uint8_t opcode = 0;
    if (!characteristic.readValue(&opcode, 1)) {
      Serial.println("Failed to read opcode from command characteristic");
      return;
    }

    switch (opcode) {
      case CMD_CAL_START:
        enterCalibrationMode();
        break;
      case CMD_CAL_STOP:
        exitCalibrationMode();
        break;
      default:
        Serial.print("⚠️  Unbekannter Kalibrierungsbefehl: 0x");
        Serial.println(opcode, HEX);
        break;
    }
    return;
  }

  if (packetLength != (int)CALIBRATION_PACKET_SIZE) {
    Serial.print("Calibration payload length unexpected: ");
    Serial.println(packetLength);
    return;
  }

  CalibrationPayload payload;
  if (!characteristic.readValue((byte*)&payload, CALIBRATION_PACKET_SIZE)) {
    Serial.println("Failed to read calibration payload from characteristic");
    return;
  }

  latestCalibrationPayload = payload;
  calibrationPayloadReceived = true;

  Serial.println("\n===== Calibration payload via BLE =====");
  logCalibrationPayload(payload);
  Serial.println("======================================");

  sensor.applyMagCalibration(payload.softIron, payload.hardIron, true);

  // Sende ein kurzes ACK über die Control-Characteristic
  controlCharacteristic.writeValue(&CMD_CALIBRATE_MAG, 1);
}

void enterCalibrationMode() {
  if (currentState == CALIBRATING) {
    Serial.println("Kalibrierung läuft bereits - ignoriere START");
    return;
  }
  Serial.println("→ Starte Magnetometer-Kalibrierungsmodus");
  currentState = CALIBRATING;
  lastSendTime = 0;
  idle = false;
  controlCharacteristic.writeValue(&CMD_CALIBRATE_MAG, 1);
}

void exitCalibrationMode() {
  if (currentState != CALIBRATING) {
    Serial.println("Kalibrierung ist nicht aktiv - ignoriere STOP");
    return;
  }
  Serial.println("Beende Magnetometer-Kalibrierungsmodus");
  currentState = RUNNING;
  controlCharacteristic.writeValue(&CMD_RUNNING, 1);
}

void logCalibrationPayload(const CalibrationPayload& payload) {
  Serial.println("Soft-Iron Matrix (row-major):");
  for (int row = 0; row < 3; row++) {
    Serial.print("[ ");
    for (int col = 0; col < 3; col++) {
      Serial.print(payload.softIron[row][col], 6);
      Serial.print(col < 2 ? ", " : " ");
    }
    Serial.println("]");
  }

  Serial.println("Hard-Iron Offset (µT):");
  Serial.print("[ ");
  Serial.print(payload.hardIron[0], 6);
  Serial.print(", ");
  Serial.print(payload.hardIron[1], 6);
  Serial.print(", ");
  Serial.print(payload.hardIron[2], 6);
  Serial.println(" ]");
}
