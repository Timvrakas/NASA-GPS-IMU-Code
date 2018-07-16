#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

static const uint32_t GPSBaud = 9600;
static const uint32_t cycleDelay = 1000;

Adafruit_BNO055 bno = Adafruit_BNO055();
TinyGPSPlus gps;

int IOLED = 6;
int GPSLED = 12;
int button = 8;
int gpsReset = 11;
boolean gpsConnected = false;

uint32_t timer;
int gpsWatchdog;
int gpsPacketCount = 0;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPSBaud);

  Serial.println(F("USB-GPS-IMU Board v1.1"));
  Serial.println(F("Integrated GPS and Orientation Sensor."));
  Serial.print(F("Using TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Tim Vrakas"));
  Serial.println();

  if (!bno.begin())
  {
    while (1) {
      Serial.println(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    }
  }

  delay(1000);
  int8_t temp = bno.getTemp();
  Serial.print(F("Current Temperature: "));
  Serial.print(temp);
  Serial.println(F(" C"));
  Serial.println();

  bno.setExtCrystalUse(true);
  timer = millis();
  gpsWatchdog = millis();

  pinMode(IOLED, OUTPUT);
  pinMode(GPSLED, OUTPUT);
  digitalWrite(GPSLED, HIGH);
  digitalWrite(IOLED, HIGH);

}

void loop()
{
  while (Serial1.available() > 0) //Process data from GPS module
    gps.encode(Serial1.read());

  if (millis() - timer > cycleDelay) { //data print loop
    timer = millis();

    digitalWrite(IOLED, HIGH); //blinky

    StaticJsonBuffer<255> jsonBuffer;

    JsonObject& root = jsonBuffer.createObject();

    root["GPS_alive"] = gpsConnected;
  
    processGPS(root);
    processIMU(root);

    root.printTo(Serial);
    Serial.println();

    digitalWrite(IOLED, LOW); //blinky

    if (millis() - gpsWatchdog > 5000) { //GPS Connection Management
      if (gps.charsProcessed() - gpsPacketCount < 10) {
        Serial.println(F("No GPS detected"));
        digitalWrite(GPSLED, LOW);
        gpsConnected = false;
      } else {
        digitalWrite(GPSLED, HIGH);
        gpsConnected = true;
        gpsWatchdog = millis();
        gpsPacketCount = gps.charsProcessed();
      }
    }
  }
}

void processIMU(JsonObject& root) {

  //IMU Temperature Sensor
  root["temp"] = bno.getTemp();

  //Euler Vector
  imu::Vector<3> imuEuler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  JsonObject& euler = root.createNestedObject("euler");
  euler["x"] = imuEuler.x();
  euler["y"] = imuEuler.y();
  euler["z"] = imuEuler.z();

  // Quaternion
  imu::Quaternion imuQuat = bno.getQuat();
  JsonObject& quat = root.createNestedObject("quat");
  quat["qW"] = imuQuat.w();
  quat["qY"] = imuQuat.y();
  quat["qX"] = imuQuat.x();
  quat["qZ"] = imuQuat.z();

  // IMU Calibratio Status
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  JsonObject& calStatus = root.createNestedObject("imu_cal");
  calStatus["system"] = system;
  calStatus["gyro"] = gyro;
  calStatus["accel"] = accel;
  calStatus["mag"] = mag;

  uint8_t system_status, self_test_result, system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_result, &system_error);
  JsonObject& imuStatus = root.createNestedObject("imu_status");
  imuStatus["system"] = system_status;
  imuStatus["self_test"] = self_test_result;
  imuStatus["error"] = system_error;

}

void processGPS(JsonObject& root)
{

  JsonObject& gpsData = root.createNestedObject("gps");
  gpsData["lat"] = gps.location.lat();
  gpsData["lon"] = gps.location.lng();
  gpsData["alt"] = gps.altitude.meters();
  gpsData["hdop"] = gps.hdop.value();

  JsonObject& date = gpsData.createNestedObject("date");
  date["year"] = gps.date.year();
  date["month"] = gps.date.month();
  date["day"] = gps.date.day();
  date["hour"] = gps.time.hour();
  date["min"] = gps.time.minute();
  date["sec"] = gps.time.second();
  date["centisec"] = gps.time.centisecond();

  if (gps.location.isValid() && gps.hdop.value() <= 500) {
    Serial.print(F("TRUE;"));
  } else {
    Serial.print(F("FALSE;"));
    digitalWrite(GPSLED, !digitalRead(GPSLED));
  }

}
