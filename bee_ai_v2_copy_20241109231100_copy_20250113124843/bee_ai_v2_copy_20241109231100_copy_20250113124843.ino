#include <feature_10_inferencing.h>
static bool debug_nn = false;

#include <lvgl.h>

// uncomment a library for display driver
#define USE_TFT_ESPI_LIBRARY
// #define USE_ARDUINO_GFX_LIBRARY

#include "lv_xiao_round_screen.h"

#include <SD.h>
#include "I2C_BM8563.h"

#define NUM_ADC_SAMPLE 20
#define RP2040_VREF 3300  // The actual voltage on 3V3 pin. (unit: mV)
static lv_obj_t *slider_label;
static lv_obj_t *battery_bar, *battery_label;

I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <PNGdec.h>
PNG png;

// ===================== 코드에서 사용할 전역 변수 값들 =====
// 가속도의 무시를 위한 데드존
const float DEADZONE = 0.01f;
// ========================================================

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
#define BNO055_SAMPLERATE_DELAY_MS (10)

#include "Seeed_BME280.h"

BME280 bme280;

int32_t battery_level_percent(void) {
  int32_t mvolts = 0;
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
  for (int8_t i = 0; i < NUM_ADC_SAMPLE; i++) {
    mvolts += analogReadMilliVolts(D0);
  }
  mvolts /= NUM_ADC_SAMPLE;
#elif defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE) || defined(ARDUINO_SEEED_XIAO_NRF52840)
  int32_t adc_raw = 0;
  for (int8_t i = 0; i < NUM_ADC_SAMPLE; i++) {
    adc_raw += analogRead(D0);
  }
  adc_raw /= NUM_ADC_SAMPLE;
  mvolts = 2400 * adc_raw / (1 << 12);
#elif defined(ARDUINO_SEEED_XIAO_RP2040)
  int32_t adc_raw = 0;
  for (int8_t i = 0; i < NUM_ADC_SAMPLE; i++) {
    adc_raw += analogRead(D0);
  }
  adc_raw /= NUM_ADC_SAMPLE;
  mvolts = RP2040_VREF * adc_raw / (1 << 12);
#endif
  int32_t level = (mvolts - 1850) * 100 / 250;  // 1850 ~ 2100
  level = (level < 0) ? 0 : ((level > 100) ? 100 : level);
  return level;
}

void rtc_clock_cb(lv_timer_t *timer) {
  lv_obj_t *rtc_clock = (lv_obj_t *)timer->user_data;
  I2C_BM8563_TimeTypeDef tStruct;
  rtc.getTime(&tStruct);
  char rtc_time[10];
  sprintf(rtc_time, "%d:%d:%d", tStruct.hours, tStruct.minutes, tStruct.seconds);
  lv_label_set_text(rtc_clock, rtc_time);
}

static void event_handler(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED) {
    LV_LOG_USER("State: %s\n", lv_obj_has_state(obj, LV_STATE_CHECKED) ? "On" : "Off");
  }
}

static void hardware_polled_cb(lv_timer_t *timer) {
  /*lv_obj_t *tf_state = (lv_obj_t *)timer->user_data;
  if (SD.begin(D2)) {
    lv_obj_add_state(tf_state, LV_STATE_CHECKED);
    delay(200);
    SD.end();
  } else {
    lv_obj_clear_state(tf_state, LV_STATE_CHECKED);
  }*/
  lv_label_set_text_fmt(battery_label, "%" LV_PRId32 "%", battery_level_percent());
}

static void slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  lv_label_set_text_fmt(slider_label, "%" LV_PRId32, lv_slider_get_value(slider));
  lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);
}

void lv_hardware_test(void) {
  lv_obj_t *slider = lv_slider_create(lv_scr_act());
  lv_obj_set_width(slider, 120);
  lv_obj_set_pos(slider, 60, 120);
  lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  slider_label = lv_label_create(lv_scr_act());
  lv_label_set_text(slider_label, "0");
  lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);

  lv_obj_t *tf_label = lv_label_create(lv_scr_act());
  lv_label_set_text(tf_label, "tf-card");
  lv_obj_set_pos(tf_label, 90, 170);
  lv_obj_t *tf_state = lv_switch_create(lv_scr_act());
  lv_obj_set_pos(tf_state, 90, 190);
  lv_obj_add_state(tf_state, LV_STATE_CHECKED | LV_STATE_DISABLED);
  lv_obj_add_event_cb(tf_state, event_handler, LV_EVENT_ALL, NULL);
  lv_timer_create(hardware_polled_cb, 7000, tf_state);

  rtc.begin();
  I2C_BM8563_TimeTypeDef tStruct;
  rtc.getTime(&tStruct);
  char rtc_time[10];
  sprintf(rtc_time, "%d:%d:%d", tStruct.hours, tStruct.minutes, tStruct.seconds);
  lv_obj_t *rtc_clock = lv_label_create(lv_scr_act());
  lv_label_set_text(rtc_clock, rtc_time);
  lv_obj_set_pos(rtc_clock, 55, 45);
  lv_timer_create(rtc_clock_cb, 1000, rtc_clock);


  analogReadResolution(12);
#if defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE) || defined(ARDUINO_SEEED_XIAO_NRF52840)
  analogReference(AR_INTERNAL2V4);  // 0.6V ref  1/4 Gain
#endif

  lv_obj_t *battery_outline = lv_obj_create(lv_scr_act());
  lv_obj_set_style_border_width(battery_outline, 2, 0);
  lv_obj_set_style_pad_all(battery_outline, 0, 0);
  lv_obj_set_style_radius(battery_outline, 8, 0);
  lv_obj_clear_flag(battery_outline, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(battery_outline, 40, 20);
  lv_obj_set_pos(battery_outline, 128, 42);

  battery_bar = lv_bar_create(battery_outline);
  lv_obj_set_size(battery_bar, 40, 20);
  lv_obj_align_to(battery_bar, battery_outline, LV_ALIGN_CENTER, 0, 0);
  lv_bar_set_range(battery_bar, 0, 100);
  lv_bar_set_value(battery_bar, 80, LV_ANIM_OFF);
  // lv_obj_set_style_bg_color(battery_bar, lv_palette_main(LV_PALETTE_GREEN), 0);

  battery_label = lv_label_create(battery_outline);
  lv_obj_align_to(battery_label, battery_outline, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text_fmt(battery_label, "80%");
}

lv_obj_t *move_label = NULL;

File myfile;

void * myOpen(const char *filename, int32_t *size) {
  Serial.printf("Attempting to open %s\n", filename);
  myfile = SD.open(filename);
  *size = myfile.size();
  return &myfile;
}
void myClose(void *handle) {
  if (myfile) myfile.close();
}
int32_t myRead(PNGFILE *handle, uint8_t *buffer, int32_t length) {
  if (!myfile) return 0;
  return myfile.read(buffer, length);
}
int32_t mySeek(PNGFILE *handle, int32_t position) {
  if (!myfile) return 0;
  return myfile.seek(position);
}

// Function to draw pixels to the display
void PNGDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[240];

  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(0, 0+pDraw->y, pDraw->iWidth, 1, lineBuffer);
}



void setup() {
  delay(5000);
  Serial.begin(115200);  //prepare for possible serial debug
  Serial.println("XIAO round screen - LVGL_Arduino");

  lv_init();

  lv_xiao_disp_init();
  if(SD.begin(D2)){
    Serial.println("SD init!");
  }

  int rc= png.open("/test.png", myOpen, myClose, myRead, mySeek, PNGDraw);
  if(rc == PNG_SUCCESS){
    tft.startWrite();
    rc = png.decode(NULL,0);
    tft.endWrite();
    png.close();
  }

  lv_xiao_touch_init();

  lv_hardware_test();
  move_label = lv_label_create(lv_scr_act());
  lv_obj_set_pos(move_label, 90, 140);

  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  if (!bme280.init()) {
    Serial.println("Device error!");
  }

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 10) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
    return;
  }
}

unsigned long timer = 0;
unsigned long timer1 = 0;
unsigned long timer2 = 0;
unsigned long timer3 = 0;
float pressure;

double filteredX, filteredY;

// set alpha 0.0f to turn off ema filter
const float alpha = 0.8f;

double filtereIX, filtereIY;

double ang_x, ang_y, ang_z;
double gyroX, gyroY;

float buffer_j[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
size_t ix = 0;

bool inference_flag = false;

void loop() {
  if ((millis() - timer) > (BNO055_SAMPLERATE_DELAY_MS - 1)) {
    timer = millis();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    // 각속도 얻어오는 부분
    imu::Vector<3> gyroVector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> laccVector = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    if ((millis() - timer3) > 999) {
        buffer_j[ix] = bme280.getTemperature();
        buffer_j[ix + 1] = bme280.getPressure();
        buffer_j[ix + 2] = bme280.calcAltitude(pressure);
        buffer_j[ix + 3] = bme280.getHumidity();
        buffer_j[ix + 4] = gyroVector.x();
        buffer_j[ix + 5] = gyroVector.y();
        buffer_j[ix + 6] = gyroVector.z();
        buffer_j[ix + 7] = laccVector.x();
        buffer_j[ix + 8] = laccVector.y();
        buffer_j[ix + 9] = laccVector.z();
      if (ix + 9 == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1) {
        inference_flag = true;
      } else
        ix += 10;//ix = ix + 6
    }
  }

  if (inference_flag) {

    inference_flag = false;
    ix = 0;
    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer_j, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
      ei_printf("Failed to create signal from buffer (%d)\n", err);
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
      ei_printf("ERR: Failed to run classifier (%d)\n", err);
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    int j_i = 0;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
      if (result.classification[ix].value > result.classification[j_i].value)
        j_i = ix;
    }

    char rtc_time[16];
    sprintf(rtc_time, "%s: %.2f", result.classification[j_i].label, result.classification[j_i].value);
    lv_label_set_text(move_label, rtc_time);
    timer3 = millis();
  }

  if ((millis() - timer1) > (4)) {
    timer1 = millis();
    lv_timer_handler();  //let the GUI do its work
  }

  if ((millis() - timer2) > (999)) {
    timer2 = millis();

    float pressure;

    //get and print temperatures
    Serial.print("Temp: ");
    Serial.print(bme280.getTemperature());
    Serial.println("C");  //The unit for  Celsius because original arduino don't support speical symbols

    //get and print atmospheric pressure data
    Serial.print("Pressure: ");
    pressure = bme280.getPressure();
    Serial.print(pressure / 100);
    Serial.println("hPa");

    //get and print altitude data
    Serial.print("Altitude: ");
    Serial.print(bme280.calcAltitude(pressure));
    Serial.println("m");

    //get and print humidity data
    Serial.print("Humidity: ");
    Serial.print(bme280.getHumidity());
    Serial.println("%");
  }
}
