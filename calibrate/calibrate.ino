#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

int IOLED = 6;
int GPSLED = 12;
int button = 8;
int gpsReset = 11;

uint16_t cycleDelay = 1000;
uint32_t timer;


void setup()
{
  Serial.begin(115200);

  Serial.println(F("USB-GPS-IMU Board v1.1 [CALIBRATE]"));
  Serial.println(F("Integrated GPS and Orientation Sensor."));
  Serial.println(F("by Tim Vrakas"));
  Serial.println();

  if (!bno.begin())
  {
    while (1) {
      Serial.println(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    }
  }

  bno.setExtCrystalUse(true);
  timer = millis();

  pinMode(IOLED, OUTPUT);
  pinMode(GPSLED, OUTPUT);
  digitalWrite(GPSLED, HIGH);
  digitalWrite(IOLED, HIGH);

  Serial.println("Please Calibrate Sensor: ");

}

void loop()
{
  if (millis() - timer > cycleDelay) { //data print loop
    timer = millis();

    digitalWrite(IOLED, HIGH); //blinky

    //Print Debug Status
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");

    //Print Calibration Data
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);

    if (Serial.read() = 'S') { //Save Cal Data
      if (bno.isFullyCalibrated()) {
        Serial.println("Saving Calibration...");
        adafruit_bno055_offsets_t newCalib;
        bno.getSensorOffsets(newCalib);
        EEPROM.put(0, newCalib);
        Serial.println("Calibration Saved!");
      } else {
        Serial.println("Not Calibrated!");
      }
    }

    digitalWrite(IOLED, LOW); //blinky
  }
}
