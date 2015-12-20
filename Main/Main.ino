#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

static const uint32_t GPSBaud = 4800;
#define SAMPLERATE_DELAY (1000)

Adafruit_BNO055 bno = Adafruit_BNO055();
TinyGPSPlus gps;

int greenLED = 5;
int redLED = 6;

int timer;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPSBaud);

  Serial.println(F("USB-GPS-IMU Board v1"));
  Serial.println(F("USB hub with integrated GPS and Orientation Sensor."));
  Serial.print(F("Using TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Tim Vrakas"));
  Serial.println();

  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);
  timer = millis();

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);
}

void loop()
{
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());

  if (gps.location.isUpdated() || (millis() - timer > 1000)) {
    digitalWrite(greenLED, HIGH);
    timer = millis();
    displayGPS();
    displayIMU();
    Serial.println();
    digitalWrite(greenLED, LOW);
    delay(SAMPLERATE_DELAY);
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }

}

void displayIMU() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(";");
  Serial.print(euler.x());
  Serial.print(";");
  Serial.print(euler.y());
  Serial.print(";");
  Serial.print(euler.z());

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("  Calibration:");
  Serial.print(" Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.print(mag, DEC);
}

void displayGPS()
{
  if (gps.location.isValid())
  {

    Serial.print(gps.location.lat(), 6);
    Serial.print(F(";"));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(";"));
  }
  else
  {
    Serial.print(F("INVALID;"));
    Serial.print(F("INVALID;"));
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
    Serial.print(F("INVALID;"));
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
  }
  else
  {
    Serial.print(F("INVALID;"));
  }
}
