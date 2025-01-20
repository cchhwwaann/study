
/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <feature_10_inferencing.h>//프로젝트 이름으로 수정
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ErriezBMX280.h>


/* Constant defines -------------------------------------------------------- */
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
#define BNO055_SAMPLERATE_DELAY_MS (10)

#define SEA_LEVEL_PRESSURE_HPA      1026.25
ErriezBMX280 bmx280 = ErriezBMX280(0x76);

//추가한부분
#define SAMPLERATE_DELAY_MS (10)
unsigned long timer = 0;


double ang_x, ang_y, ang_z;
double gyroX, gyroY;

#define EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME 10
/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/**
* @brief      Arduino setup function
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};




void setup()
{
    delay(5000);
    // put your setup code here, to run once:
    Serial.begin(115200);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)

 
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    if (!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1)
        ;
    }
  Wire.begin();
    Wire.setClock(100000);
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



    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 10) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }

    // Create the BLE Device
  BLEDevice::init("cchhwwaann");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");





}

/**
 * @brief Return the sign of the number
 * 
 * @param number 
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    //ei_printf("\nStarting inferencing in 2 seconds...\n");

    delay(2000);

    //ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 10) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

        /* Display calibration status for each sensor. */
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);

        imu::Vector<3> gyroVector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Vector<3> laccVector = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

        buffer[ix] = bmx280.readTemperature();
        buffer[ix + 1] = bmx280.readHumidity();
        buffer[ix + 2] = bmx280.readPressure() / 100.0F;
        buffer[ix + 3] = bmx280.readAltitude(SEA_LEVEL_PRESSURE_HPA);
        buffer[ix + 4] = gyroVector.x();
        buffer[ix + 5] = gyroVector.y();
        buffer[ix + 6] = gyroVector.z();
        buffer[ix + 7] = laccVector.x();
        buffer[ix + 8] = laccVector.y();
        buffer[ix + 9] = laccVector.z();
        




        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    //ei_printf("Predictions ");
    //ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        //result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");

    float max_value = -1.0; // 최소값으로 초기화
    const char* max_label = NULL; // 최대값에 대응하는 레이블 초기화

// 최대값 탐색
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {

    if (result.classification[ix].value > max_value) {
        max_value = result.classification[ix].value; // 최대값 갱신
        max_label = result.classification[ix].label; // 해당 레이블 갱신
        }
        
    

    /*for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
    #if EI_CLASSIFIER_HAS_ANOMALY == 1
      ei_printf("    anomaly score: %.3f\n", result.anomaly);*/
//#endif
    if (deviceConnected) {
    static String lastSent = ""; // 이전에 전송된 값 저장
    String input = String(max_label);

    // 값이 변경되었을 때만 전송
    if (input != lastSent) {
        Serial.print("Sending to Smartphone: ");
        Serial.println(input);

        pTxCharacteristic->setValue(input.c_str());
        pTxCharacteristic->notify();
        lastSent = input; // 현재 값을 저장
    }
}

    
		
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
