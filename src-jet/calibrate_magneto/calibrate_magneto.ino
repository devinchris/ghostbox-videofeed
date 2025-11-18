#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

BLEService magService("180A"); // Magnetometer Service UUID
BLECharacteristic magChar("2A58", BLERead | BLENotify, 50); // Data characteristic

unsigned long lastSample = 0;
const unsigned long sampleInterval = 50; // 20 Hz (50ms)

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("BLE init failed!");
    while (1);
  }

  BLE.setLocalName("MagSensor");
  BLE.setAdvertisedService(magService);

  magService.addCharacteristic(magChar);
  BLE.addService(magService);

  BLE.advertise();
  Serial.println("BLE advertising Magnetometer Service (UUID 180A)");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      unsigned long now = millis();
      if (now - lastSample >= sampleInterval) {
        lastSample = now;

        float x, y, z;
        if (IMU.magneticFieldAvailable()) {
          IMU.readMagneticField(x, y, z);

          char buffer[50];
          snprintf(buffer, sizeof(buffer), "%.6f,%.6f,%.6f", x, y, z);
          magChar.writeValue(buffer);
          Serial.println(buffer);
        }
      }
    }

    Serial.println("Disconnected");
  }
}
