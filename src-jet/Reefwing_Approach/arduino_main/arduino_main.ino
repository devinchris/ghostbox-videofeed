#include <ArduinoBLE.h>
#include "SensorManager.h"
#include "SmoothData4D.h"
 
#define DELAY 100

// Single Sensor Sample (32 Bytes)
struct SensorValues {
  uint32_t timestamp_ms;        // in ms
  float accelX, accelY, accelZ; // in m/s
  float q0, q1, q2, q3;         // in Grad
  float magX, magY, magZ;       // in Tesla    
  float tempC;                  // in Celsius
} __attribute__((packed));      // Verhindert Padding
 
SensorValues sensorData;
const unsigned long sendInterval = 100;
unsigned long lastSendTime;
float tempC;
 
// Create BLE Service
// Raspberry:
BLEService myService("19B10000-E8F2-537E-4F6C-D104768A1214");
// Mac:
// BLEService myService("19B10000-E8F2-537E-4F6C-D104768A1267");


// === SensorManager ===
SensorManager sensor;
Quaternion _Quaternion;
float ax = 0, ay = 0, az = 0;
float mx = 0, my = 0, mz = 0;

// === SmoothData4D ===
SmoothQuaternionData smoothing;  

// =====================

// Create BLE Characteristic with READ, WRITE, and NOTIFY
BLECharacteristic myCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLEWrite | BLENotify, sizeof(SensorValues));
 
void setup() {
  Serial.begin(115200);
  // while (!Serial);
 
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
 
  BLE.setConnectionInterval(6, 6);
  BLE.setAdvertisingInterval(160);
  // Set advertised local name and service
  BLE.setLocalName("Nano");
  BLE.setDeviceName("Nano");
 
  BLE.setAdvertisedService(myService);
 
  // Add characteristic to service
  myService.addCharacteristic(myCharacteristic);
 
  // Add service
  BLE.addService(myService);
 
  // Set initial value
  myCharacteristic.writeValue("Ready");
 
  // Start advertising
  BLE.advertise();
 
  Serial.println("Bluetooth device active, waiting for connections...");

  sensor.init();

  // Initialisiere Smoothing mit Target-Frequenz und Alpha-Wert
  // Target-Frequenz sollte der Sensor-Update-Rate entsprechen (z.B. 100 Hz)
  // Alpha: 0.55 = mittlere Glättung (0.01 = sehr stark, 0.99 = fast keine Glättung)
  smoothing.initSmoothing(10.0f, 0.55f);
}
 
void loop() {
  // Listen for BLE centrals to connect
  BLEDevice central = BLE.central();
 
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
   
    int counter = 0;
   
    // While the central is connected
    while (central.connected()) {
      if(millis() - lastSendTime >= sendInterval){
        lastSendTime = millis();
        
        // Hole Sensordaten vom SensorManager
        sensor.getCalculatedData(_Quaternion, ax, ay, az, mx, my, mz);
        sensor.getTemperature(tempC);
        
        // WICHTIG: Glätte das Quaternion mit SmoothData4D
        // Das Quaternion wird direkt modifiziert (Pass-by-Reference)
        smoothing.smoothQuaternions(_Quaternion, millis());
    
        // Fülle SensorData Struktur mit geglätteten Werten
        sensorData.timestamp_ms = millis();
        sensorData.accelX = ax;
        sensorData.accelY = ay;
        sensorData.accelZ = az;
        sensorData.q0 = _Quaternion.q0;  // Geglättete Werte!
        sensorData.q1 = _Quaternion.q1;
        sensorData.q2 = _Quaternion.q2;
        sensorData.q3 = _Quaternion.q3;
        sensorData.magX = mx;
        sensorData.magY = my;
        sensorData.magZ = mz;
        sensorData.tempC = tempC;
      
        Serial.print("QUAT: "); 
        Serial.print(" q0: "); Serial.print(_Quaternion.q0); Serial.print(" |\t");
        Serial.print("q1: "); Serial.print(_Quaternion.q1); Serial.print(" |\t");
        Serial.print("q2: "); Serial.print(_Quaternion.q2); Serial.print(" |\t");
        Serial.print("q3: "); Serial.print(_Quaternion.q3); Serial.println(" |\t");

        myCharacteristic.writeValue((byte*)&sensorData, sizeof(sensorData));
        //Andi war hier <- What the helly?
      }
     
     //delay(DELAY);  
    }
   
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
/*
void debugMeasurements(Quaternion _Q, float ax, float ay, float az) {
  Serial.println("");
  Serial.print("Q0: "); Serial.print(_Q.q0); Serial.print(" |\t");
  Serial.print("Q1: "); Serial.print(_Q.q1); Serial.print(" |\t");
  Serial.print("Q2: "); Serial.print(_Q.q2); Serial.print(" |\t");
  Serial.print("Q3: "); Serial.print(_Q.q3); Serial.print(" |\t");

  Serial.print("aX: "); Serial.print(ax); Serial.print(" |\t");
  Serial.print("aY: "); Serial.print(ay); Serial.print(" |\t");
  Serial.print("aZ: "); Serial.print(az); Serial.print(" |\t");
}

void debugEulerAngles(float roll, float pitch, float yaw) {
  Serial.println("");
  Serial.print("Roll: "); Serial.print(roll); Serial.print(" |\t");
  Serial.print("Pitch: "); Serial.print(pitch); Serial.print(" |\t");
  Serial.print("Yaw: "); Serial.print(yaw); Serial.print(" |\t");
}
*/