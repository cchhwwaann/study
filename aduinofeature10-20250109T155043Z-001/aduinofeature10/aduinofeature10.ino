#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

///
#include <ErriezBMX280.h>
/*!
 * \file ErriezBMX280.ino
 * \brief BMP280/BME280 sensor library for Arduino.
 * \details
 *      BMP280 supports temperature and pressure
 *      BME280 supports temperature, pressure and humidity
 * \details
 *     Source:          https://github.com/Erriez/ErriezBMX280
 *     Documentation:   https://erriez.github.io/ErriezBMX280
 */




// Check I2C device address and correct line below (by default address is 0x29 or 0x28)

///
// Adjust sea level for altitude calculation
#define SEA_LEVEL_PRESSURE_HPA      1026.25

// Create BMX280 object I2C address 0x76 or 0x77
ErriezBMX280 bmx280 = ErriezBMX280(0x76);
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//추가한부분
#define SAMPLERATE_DELAY_MS (10)
unsigned long timer = 0;


double ang_x, ang_y, ang_z;
double gyroX, gyroY;



void setup() {
  // put your setup code here, to run once:
  delay(500);
  Serial.begin(115200); 

   while (!Serial) {
        ;
    }

  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  Wire.begin();
    Wire.setClock(400000);

    // Initialize sensor
    while (!bmx280.begin()) {
        Serial.println(F("Error: Could not detect sensor"));
        delay(3000);
    }

    // Print sensor type
    Serial.print(F("\nSensor type: "));
    switch (bmx280.getChipID()) {
        case CHIP_ID_BMP280:
            Serial.println(F("BMP280\n"));
            break;
        case CHIP_ID_BME280:
            Serial.println(F("BME280\n"));
            break;
        default:
            Serial.println(F("Unknown\n"));
            break;
    }

    bmx280.setSampling(BMX280_MODE_NORMAL,    // SLEEP, FORCED, NORMAL
                       BMX280_SAMPLING_X16,   // Temp:  NONE, X1, X2, X4, X8, X16
                       BMX280_SAMPLING_X16,   // Press: NONE, X1, X2, X4, X8, X16
                       BMX280_SAMPLING_X16,   // Hum:   NONE, X1, X2, X4, X8, X16 (BME280)
                       BMX280_FILTER_X16,     // OFF, X2, X4, X8, X16
                       BMX280_STANDBY_MS_500);// 0_5, 10, 20, 62_5, 125, 250, 500, 1000


}


void loop() {
  // put your main code here, to run repeatedly:
  if ((millis() - timer) > (SAMPLERATE_DELAY_MS - 1)) {
    timer = millis();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    // 각속도 얻어오는 부분
    imu::Vector<3> gyroVector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> laccVector = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    ang_x = euler.x();
    ang_y = euler.y();
    ang_z = euler.z();

      Serial.print(bmx280.readTemperature());
    Serial.print(",");

    if (bmx280.getChipID() == CHIP_ID_BME280) {
        Serial.print(bmx280.readHumidity());
        Serial.print(",");

    Serial.print(bmx280.readPressure() / 100.0F);
    Serial.print(",");

    Serial.print(bmx280.readAltitude(SEA_LEVEL_PRESSURE_HPA));
    Serial.write(',');


    }

    Serial.print(gyroVector.x());
    Serial.write(',');
    Serial.print(gyroVector.y());
    Serial.write(',');
    Serial.print(gyroVector.z());
    Serial.write(',');
    Serial.print(laccVector.x());
    Serial.write(',');
    Serial.print(laccVector.y());
    Serial.write(',');
    Serial.print(laccVector.z());    
    Serial.write('\n');
  }
}