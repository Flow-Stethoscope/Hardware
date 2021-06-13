#include <Arduino.h>
// #include "I2SSampler.h"
#include "BluetoothSerial.h"
#include <driver/i2s.h>
#include "esp_adc_cal.h"
#include <queue>

BluetoothSerial SerialBT;

// calibration values for the adc
#define DEFAULT_VREF 1100
esp_adc_cal_characteristics_t *adc_chars;


std::queue<uint16_t>* bufferQueue;
bool flag = 0;

void initBT();
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void startRecording();
void finishRecording();
void recordData(void *arg);
void transmitData(void *arg);
void deviceCancel(void *arg);
void logData(void *arg);

TaskHandle_t taskHandler1 = NULL;
TaskHandle_t taskHandler2 = NULL;
unsigned long int timer = 0;
unsigned long int counter = 0;
unsigned long int startTime = 0;

//Sets up all the adc, bte, and serial
void setup() {
  Serial.begin(115200);
  initBT();

  //------------------------Init ADC------------------------

  //Range 0-4096
  adc1_config_width(ADC_WIDTH_BIT_12);
  // full voltage range
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);


  // check to see what calibration is available
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
  {
    Serial.println("Using voltage ref stored in eFuse");
  }
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
  {
    Serial.println("Using two point values from eFuse");
  }
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_DEFAULT_VREF) == ESP_OK)
  {
    Serial.println("Using default VREF");
  }
  //Characterize ADC
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

  //------------------------Init ADC------------------------
  }

//Initalizes bluetooth
void initBT(){
  if(!SerialBT.begin("ESP32-Recorder")){
    Serial.println("An error occurred initalizing Bluetooth");
    ESP.restart();
    return;
  } else{
    Serial.println("Bluetooth Initialized");
  }

  SerialBT.register_callback(btCallback);
  Serial.println("==========================================================");
  Serial.println("The device started, now you can pair it with bluetooth-003");
  Serial.println("==========================================================");
}

//Gets Called whenever bluetooth happens
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected!");
  }
  else if(event == ESP_SPP_DATA_IND_EVT){
    Serial.printf("ESP_SPP_DATA_IND_EVT len=%d handle=%d\n", param->data_ind.len, param->data_ind.handle);
    int dataLen = param->data_ind.len + 1;
    char textArray[dataLen];
    strncpy(textArray, (const char*)param->data_ind.data, dataLen);
    textArray[dataLen - 1] = 0;
    String textString = textArray;
    Serial.print("*** Text String: ");
    Serial.print(textString);
    Serial.println("\n");

    //If device wants to start or stop call the function
    if(textString.equals("START")){
      startRecording();
    }else if(textString.equals("STOP")){
      finishRecording();
    }
  }
}

//Starts recording by creating the seperate record and transmit loops as well as init the queue
void startRecording(){
  Serial.println("*** Recording Start ***");
  bufferQueue = new std::queue<uint16_t>;
  xTaskCreate(recordData, "recordData", 1024 * 2, NULL, 3, &taskHandler1);
  xTaskCreate(transmitData, "transmitData", 1024 * 2, NULL, 2, &taskHandler2);
  //xTaskCreate(logData, "logData", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

//Finishes by deleting the recording and creating a new loop that waits for the data to finish sending and then cleans everything up
void finishRecording(){
  Serial.println("*** Recording End ***");
  vTaskDelete(taskHandler1);
  flag = 0;
  xTaskCreate(deviceCancel, "deviceCancel", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

//Async log function b/c serial is taxing
void logData(void *arg){
  vTaskDelay(2000);
  while(1){
      Serial.print((micros() - timer) / 1000.00);
      Serial.print(", ");
      Serial.print(  counter / (float)((millis() - startTime)/1000.00)  );
      Serial.print(", ");
      Serial.print(counter);
      Serial.print(", ");
      Serial.print((float)((millis() - startTime)/1000.00));
      Serial.print(", ");
      Serial.println(bufferQueue->size());

    vTaskDelay(2000);
  }
  vTaskDelete(NULL);
}

//A new loop that waits for the data to finish sending and then cleans everything up
void deviceCancel(void *arg){
  vTaskDelay(2000);
  while(1){
    if(flag == 0){
      Serial.println("***Transmition Done***");
      vTaskDelete(taskHandler2);
      delete bufferQueue;
      break;
    }
    vTaskDelay(1000);
  }
  vTaskDelete(NULL);
}

//Reads the data from the adc
void recordData(void *arg){
  Serial.println("Test1");
  timer = micros();
  startTime = millis();
  
  counter = 0;
  while(1){
      timer = micros();
      int sample = adc1_get_raw(ADC1_CHANNEL_7);
      counter++;
      bufferQueue->push((uint32_t)sample);
    vTaskDelay(portTICK_PERIOD_MS);

  }
  vTaskDelete(NULL);
}

//Transmits the data over bluetooth
void transmitData(void *arg){
  Serial.println("Test2");
  //unsigned long int startTime2 = millis();
  unsigned long int time2 = micros();

  while (1) {
    if(bufferQueue->size() > 0){
    flag = 1;
    int sample = (uint32_t)bufferQueue->front();  
    SerialBT.println(sample);
    bufferQueue->pop();
    time2 = micros();
    }
    else{
      flag = 0;
    }
    vTaskDelay(portTICK_PERIOD_MS/2);
  }
  vTaskDelete(NULL);
}

//Not needed but requried to compile
void loop()
{
}
