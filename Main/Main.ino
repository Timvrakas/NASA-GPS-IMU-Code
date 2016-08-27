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

  Serial.println(F("USB-GPS-IMU Board v1"));
  Serial.println(F("USB hub with integrated GPS and Orientation Sensor."));
  Serial.print(F("Using TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Tim Vrakas"));
  Serial.println();

  if (!bno.begin())
  {
    while (1) {
      Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
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

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  pinMode(IOLED, OUTPUT);
  pinMode(GPSLED, OUTPUT);
  digitalWrite(GPSLED, HIGH);
  digitalWrite(IOLED, HIGH);

}

void loop()
{
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());

  if (millis() - timer > cycleDelay) {
    digitalWrite(IOLED, HIGH);
    timer = millis();
    displayGPS();
    displayIMU();
    Serial.println();
    digitalWrite(IOLED, LOW);

    if (millis() - gpsWatchdog > 5000) {
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

void displayIMU() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(euler.x());
  Serial.print(F(";"));
  Serial.print(euler.y());
  Serial.print(F(";"));
  Serial.print(euler.z());
  Serial.print(F(";"));

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print(F("S:"));
  Serial.print(system, DEC);
  Serial.print(F(";G:"));
  Serial.print(gyro, DEC);
  Serial.print(F(";A:"));
  Serial.print(accel, DEC);
  Serial.print(F(";M:"));
  Serial.print(mag, DEC);
}

void displayGPS()
{
  boolean valid = true;
  if (gps.location.isValid())
  {

    Serial.print(gps.location.lat(), 6);
    Serial.print(F(";"));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(";"));
  }
  else
  {
    Serial.print(F("X;X;"));
    valid = false;
  }

  if (gps.altitude.isValid())
  {
    Serial.print(gps.altitude.meters(), 6);
    Serial.print(F(";"));
  }
  else
  {
    Serial.print(F("X;"));
    valid = false;
  }

  if (gps.hdop.isValid())
  {
    Serial.print(gps.hdop.value(), 6);
    Serial.print(F(";"));
  }
  else
  {
    Serial.print(F("X;"));
    valid = false;
  }

  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("X"));
    valid = false;
  }

  Serial.print(F(";"));

  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
    Serial.print(F(";"));
  }
  else
  {
    Serial.print(F("X;"));
    valid = false;
  }

  if (valid && gps.hdop.value() <= 500)
    Serial.print(F("TRUE;"));
  else
    Serial.print(F("FALSE;"));
}
