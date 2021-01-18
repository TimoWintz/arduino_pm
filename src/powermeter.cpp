#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include "HX711.h"
#include <ArduinoBLE.h>

HX711 scale;

uint8_t dataPin = 2;
uint8_t clockPin = 3;
unsigned long prev_ms;

#define CIRCLE_LENGTH 172.5/1000*2*3.14159
#define MESSAGE_DT_MS 1000
#define MEASUREMENT_DT_MS 100
#define SLEEP_DT_MS 60000 // turn off after 60s
#define GYRO_DPS_THRESHOLD 10
#define MIN_POWER 10
#define CALIBRATE 1
#define GRAVITY 9.81
#define CALIBRATION_WEIGHT 12.0

uint32_t start, stop;
volatile float f;

float ax, ay, az;
float x, y, z;
// Define custom BLE service for position (read-only)
BLEService powerService("1818");
BLECharacteristic powerFeature("2A65", BLERead, 4);
BLECharacteristic powerMeasurement("2A63",  BLERead | BLENotify, 8);
BLECharacteristic sensorLocation("2A5D", BLERead, 1);

unsigned char bleBuffer[8];
unsigned char slBuffer[1];
unsigned char fBuffer[4];

short power;
float avg_power;
float elapsed;
unsigned int n_measurements;
unsigned short revolutions = 0;
unsigned short timestamp = 0;
unsigned short flags = 0x20;
byte sensorlocation = 0x0D;
int dt;
char input;
unsigned long lastMotion;

bool halfRevolution = false;
bool fullRevolution = false;
bool ledOn = false;

void setup() {
  // Initialize internal LED (for visual debugging)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize Serial connection
  Serial.begin(115200);



  // Initialize IMU
  if (!IMU.begin()) {
    // Failed to initialize IMU, blink the internal LED
    Serial.println("Failed initializing IMU");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }
  }



  // Initialize BLE
  if (!BLE.begin()) {
    // Failed to initialize BLE, blink the internal LED
    Serial.println("Failed initializing BLE");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  // Set advertised local name and services UUID
  BLE.setDeviceName("Arduino Nano 33 IoT");
  BLE.setLocalName("BLE Experiment");
  BLE.setAdvertisedService(powerService);

  powerService.addCharacteristic(powerFeature);
  powerService.addCharacteristic(powerMeasurement);
  powerService.addCharacteristic(sensorLocation);
  BLE.addService(powerService);

  slBuffer[0] = sensorlocation & 0xff;

  fBuffer[0] = 0x00;
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x08;

  // Start advertising
  BLE.advertise();


  scale.begin(dataPin, clockPin);
  scale.set_scale(4900);
  scale.set_offset(569525);

  lastMotion = millis();
}

void loop() {

  if (millis() - lastMotion > SLEEP_DT_MS) {
    Serial.println("Turning off");
    Serial.close();
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_PWR, LOW);
    NRF_POWER->SYSTEMOFF = 1;
  }

  while (Serial.available()) {
    input = Serial.read();
    if (input == 'c') {
      Serial.println("Zero calibration");
      scale.tare();
      Serial.print("Offset:");
      Serial.println(scale.get_offset());
    }
    else if (input == 'w') {
      Serial.println("Weight calibration");
      float new_scale = scale.get_scale() * scale.get_units(10) / CALIBRATION_WEIGHT / GRAVITY;
      scale.set_scale(new_scale);
      Serial.print("New scale:");
      Serial.println(new_scale);
    }

  }
  // Listen for BLE
  BLEDevice central = BLE.central();
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, (PinStatus) ledOn);


  // If a central is connected
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
  }

  // While central is still connected...
  while (central.connected()) {
    byte ledValue = 0x0;
    avg_power = 0;
    n_measurements = 0;
    elapsed = 0;
    prev_ms = millis();
    while (true) {
      if (halfRevolution) {
        if (ay > 0 && az > 0) {
          Serial.println("Full Rev");
          revolutions += 1;
          halfRevolution = false;
          break;
        }
      } else {
        if (ay < 0 && az < 0) {
          halfRevolution = true;
          Serial.println("Half Rev");
        }
      }

      // Read gyroscope values
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
      }
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
      }

      if (y > GYRO_DPS_THRESHOLD || y < -GYRO_DPS_THRESHOLD) {
        lastMotion = millis();
      }

      f = scale.get_units(1);
      avg_power += f * y;
      n_measurements += 1;
      if (millis() - prev_ms > MESSAGE_DT_MS) {
        break;
      }
      delay(MEASUREMENT_DT_MS);
    }

    avg_power = avg_power / n_measurements / 360 * CIRCLE_LENGTH;
    avg_power = 2 * avg_power; // Left only
    if (avg_power < 0) avg_power = -avg_power;

    power = (short) (avg_power);
    timestamp += ((millis() - prev_ms) * 1024) / 1000;

    Serial.print(f);
    Serial.print(", P="); Serial.print(power);
    Serial.print(", dt="); Serial.print(elapsed);
    Serial.print(", t="); Serial.print(timestamp);
    Serial.print(", rev="); Serial.print(revolutions);
    Serial.println();

    bleBuffer[0] = flags & 0xff;
    bleBuffer[1] = (flags >> 8) & 0xff;
    bleBuffer[2] = power & 0xff;
    bleBuffer[3] = (power >> 8) & 0xff;
    bleBuffer[4] = revolutions & 0xff;
    bleBuffer[5] = (revolutions >> 8) & 0xff;
    bleBuffer[6] = timestamp & 0xff;
    bleBuffer[7] = (timestamp >> 8) & 0xff;

    powerFeature.writeValue(fBuffer, 4);
    powerMeasurement.writeValue(bleBuffer, 8);
    sensorLocation.writeValue(slBuffer, 1);
  }

  // when the central disconnects, print it out:
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
  delay(1000);
}