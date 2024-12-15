#include <LSM6DS3.h>
#include <RTClib.h>
#include <bluefruit.h>
#include "FreeRTOS.h"
#include "Wire.h"

// Pin definitions
// Define 8 channel analog multiplexer control pins
#define MUX1 D8
#define MUX2 D7
#define MUX3 D6
// Define 2 negative voltage switch control pins
#define PHASEP D10
#define PHASEN D9
// Define 3 circuit control pins
#define CTEST D1
#define CSLEEP D2
#define CRESET D3
// Define one ADC pin
#define ADC_SIG A0

//************************ Signal ************************
//Device name
const String deviceName = "6R";
// Base frequency for D8
const int baseFrequency = 1;  // One frame frequency 1024->16s, 64->1s
const int fsrNumber = 16;
//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

//************************ Cache ************************
const int buffBase = 200;    //Set storage size
const int pack_size = 5;   //Set BLE transmission package size
unsigned long startTime = 0;    //Reset millisecond
volatile unsigned long milliBuffer[buffBase];   //Storage for millisecond
volatile short imuBuffer[6 * buffBase];   //Storage for IMU reading
volatile short fsrBuffer[16 * buffBase];   //Storage for IMU reading
int bufferIndex = 0;    //Index for buffer of IMU reading
int adcIndex = 0;   //Index for buffer of adc reading
int milliIndex = 0;   //Index for miliBuffer send
int imuIndex = 0;   //Index for imuBuffer send
int fsrIndex = 0;   //Index for fsrBuffer send
bool imuOverflow = false;
bool adcOverflow = false;
SemaphoreHandle_t bufferSemaphore;


//************************ RTC ************************
RTC_Millis rtc;
// Initial date and time values
volatile int year = 2023;
volatile int month = 9;
volatile int day = 30;
volatile int hour = 15;
volatile int minute = 0;
volatile int second = 0;
// Reset flag
volatile bool timeReset = false;
// Update date and time every second
TickType_t Second_Time_Delay = 1024; 

//************************ BLE Service ************************
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery
char central_name_global[32] = { 0 };
String receivedString;         // Variable to store the received string
String lastProcessedString;    // Variable to store the last processed string
// Global variable to track BLE connection status
volatile bool bleConnected = false;

// Define a task function for the IMU reading
void SensorTask(void *pvParameters) {
  (void) pvParameters;

  while (true) { // A Task shall never return or exit.
    if (bleConnected && timeReset){
      // Read accelerometer data
      milliBuffer[bufferIndex] = millis() - startTime;
      imuBuffer[(bufferIndex * 6)] = myIMU.readRawAccelX();
      imuBuffer[(bufferIndex * 6) + 1] = myIMU.readRawAccelY();
      imuBuffer[(bufferIndex * 6) + 2] = myIMU.readRawAccelZ();
      // Read raw gyroscope data
      imuBuffer[(bufferIndex * 6) + 3] = myIMU.readRawGyroX();
      imuBuffer[(bufferIndex * 6) + 4] = myIMU.readRawGyroY();
      imuBuffer[(bufferIndex * 6) + 5] = myIMU.readRawGyroZ();
      // // Read thermometer data
      // sensorData[6] = myIMU.readTempC();

      bufferIndex++;
      if (bufferIndex % pack_size == 0) {
        imuOverflow = true; // A send package is full, set overflow flag
      }
      if (bufferIndex >= buffBase){
        bufferIndex = 0;    //Write buffer from beginning
      }
    }
    vTaskDelay(pdMS_TO_TICKS(baseFrequency * 16)); // Delay for a period of time
  }
}


void UnifiedSignalTask(void *pvParameters) {
  unsigned int count = 0;  // Counter to manage different delays

  for (;;) {
    if (bleConnected && timeReset) {
      // Execute operations for PHASEN, LED_GREEN, and PHASEP at eight times the base frequency
      if (count % 8 == 0) {
        digitalWrite(PHASEN, !digitalRead(PHASEN));  // Toggle PHASEN pin
        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));  // Toggle LED_GREEN pin
        digitalWrite(PHASEP, !digitalRead(PHASEP));  // Toggle PHASEP pin
      }

      // Execute operations for MUX3 and LED_RED at four times the base frequency
      if (count % 4 == 0) {
        digitalWrite(MUX3, !digitalRead(MUX3));  // Toggle MUX3 pin
        digitalWrite(LED_RED, !digitalRead(LED_RED));  // Toggle LED_RED pin
      }
      
      // Execute operations for MUX2 at twice the base frequency
      if (count % 2 == 0) {
        digitalWrite(MUX2, !digitalRead(MUX2));  // Toggle MUX2 pin
      }

      // Execute operations for MUX1 at the base frequency
      digitalWrite(MUX1, !digitalRead(MUX1));  // Toggle MUX1 pin
    
      count++;  // Increment the counter
      // Reset the counter if it exceeds or equals the least common multiple of delays to avoid overflow
      if (count >= 8) {
        count = 0;
      }
      // Get the current index from the pins
      int index = getBufferIndex();
      // Read from ADC and store in buffer at the position indicated by the pins
      fsrBuffer[adcIndex * 16 + index] = analogRead(ADC_SIG);
      // Set the overflow flag if we've just written to the last index
      if ((index + 1) % 16 == 0){
        adcIndex++;
      }

      if (adcIndex % (pack_size) == 0) {
        adcOverflow = true; // A send package is full, set overflow flag
      }
      if (adcIndex >= buffBase){
        adcIndex = 0;    //Write buffer from beginning
      }
        vTaskDelay(pdMS_TO_TICKS(baseFrequency));  // Delay of the base frequency
    }
    else{
      vTaskDelay(pdMS_TO_TICKS(baseFrequency * 16));  // Delay of the base frequency
    }

  }
}

// // This task will read ADC value and place it into the buffer based on the index
// void TaskADCReader(void *pvParameters) {
//   (void) pvParameters; // Just to avoid compiler warnings

//   for (;;) {
//     if (bleConnected && timeReset){

//     }
//     // Wait before reading the next value
//     vTaskDelay(pdMS_TO_TICKS(baseFrequency));
//   }
// }

// The RTC thread
void TaskDateTime(void *pvParameters) {
    (void) pvParameters; // Silence unused parameter warning

    while (true) {
      updateClock();
      // Delay for 1 second to match our software clock update.
      vTaskDelay(Second_Time_Delay);
    }
}

// This task checks for buffer overflow and prints the buffer if overflow occurs forward data from HW Serial to BLEUART
void ble_uart_task(void *pvParameters)
{
    (void) pvParameters; // Just to avoid compiler warnings

  for (;;) {
    if (bleConnected && timeReset){
      // Wait for the overflow flag to be set
      if (imuOverflow && adcOverflow) {
        // Take the semaphore to ensure no conflict on buffer access
        if (xSemaphoreTake(bufferSemaphore, (TickType_t)10) == pdTRUE) {

          uint8_t buf[300];
          int index = 0;


          // Add device name (assuming it's a null-terminated string)
          for (int i = 0; deviceName[i] != '\0'; i++) {
            buf[index++] = deviceName[i];
          }
          // Add a delimiter (e.g., a comma)
          buf[index++] = ','; 

          // Add 5*1 milli buffer
          for (int i = 0; i < pack_size; i++) {
            buf[index++] = (milliBuffer[milliIndex + i] >> 24) & 0xFF; // Most significant byte
            buf[index++] = (milliBuffer[milliIndex + i] >> 16) & 0xFF;
            buf[index++] = (milliBuffer[milliIndex + i] >> 8) & 0xFF;
            buf[index++] = milliBuffer[milliIndex + i] & 0xFF; // Least significant byte
          }
          milliIndex += pack_size;
          if (milliIndex >= buffBase) {
            milliIndex = 0;
          }

          // Add 5*6 imu buffer
          for (int i = 0; i < (6 * pack_size); i++) {
            buf[index++] = (imuBuffer[imuIndex + i] >> 8) & 0xFF; // Most significant byte
            buf[index++] = imuBuffer[imuIndex + i] & 0xFF; // Least significant byte
          }
          imuIndex += 6 * pack_size;
          if (imuIndex >= 6 * buffBase) {
            imuIndex = 0;
          }

          // Add 5*16 imu buffer
          for (int i = 0; i < (16 * pack_size); i++) {
            buf[index++] = (fsrBuffer[fsrIndex + i] >> 8) & 0xFF; // Most significant byte
            buf[index++] = fsrBuffer[fsrIndex + i] & 0xFF; // Least significant byte
          }
          fsrIndex += 16 * pack_size;
          if (fsrIndex >= 16 * buffBase) {
            fsrIndex = 0;
          }

          bleuart.write(buf, index);
        
          // Reset the overflow flag
          imuOverflow = false;
          adcOverflow = false;
          
          // Give the semaphore back
          xSemaphoreGive(bufferSemaphore);
        }
      }
    }
    
    // Slight delay to prevent this task from hogging the CPU
    vTaskDelay(pdMS_TO_TICKS(baseFrequency * 16));
  }
}

void ble_receive_task(void *pvParameters)
{
  while(true) {
    if (bleConnected){
      // Check if there's data available
      if (bleuart.available()) 
      {
        receivedString = bleuart.readString();
        // TODO: Handle or process the receivedString if needed
      }
    }
    vTaskDelay(pdMS_TO_TICKS(baseFrequency * 16)); // Short delay to prevent busy-waiting
  }
} 

void processReceivedStringTask(void *pvParameters) {
  while(true) {
    if (receivedString != lastProcessedString) {
      if (isInCorrectFormat(receivedString)) {
        int year   = receivedString.substring(0, 4).toInt();
        int month  = receivedString.substring(5, 7).toInt();
        int day    = receivedString.substring(8, 10).toInt();
        int hour   = receivedString.substring(11, 13).toInt();
        int minute = receivedString.substring(14, 16).toInt();
        int second = receivedString.substring(17, 19).toInt();

        DateTime newTime(year, month, day, hour, minute, second);
        rtc.adjust(newTime);
        startTime = millis();
        timeReset = true;
      }
      lastProcessedString = receivedString;
    }
    vTaskDelay(pdMS_TO_TICKS(baseFrequency * 16));
  }
}

void setup() {
  // Initialize digital pins as outputs
  pinMode(MUX1, OUTPUT);
  pinMode(MUX2, OUTPUT);
  pinMode(MUX3, OUTPUT);
  pinMode(PHASEN, OUTPUT);
  pinMode(PHASEP, OUTPUT);
  pinMode(CTEST, OUTPUT);
  pinMode(CSLEEP, OUTPUT);
  pinMode(CRESET, OUTPUT);
  pinMode(VBAT_ENABLE, OUTPUT);
  // Initialize analog pins as input
  pinMode(ADC_SIG, INPUT);

  // Initialize pin phases
  digitalWrite(MUX1, HIGH);
  digitalWrite(MUX2, HIGH);
  digitalWrite(MUX3, HIGH);
  digitalWrite(PHASEN, HIGH);
  digitalWrite(PHASEP, LOW);
  digitalWrite(CTEST, LOW);
  digitalWrite(CSLEEP, LOW);
  digitalWrite(CRESET, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  
  //Configure IMU
  myIMU.begin();

  // initialize BLE
  setupBLE();

  delay(1000);
  digitalWrite(CRESET, LOW);

  // initialize RTC
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  rtc.adjust(DateTime(year, month, day, hour, minute, second));

  // Initialize the semaphore
  bufferSemaphore = xSemaphoreCreateMutex();

  xTaskCreate(UnifiedSignalTask, "Pin switch", 128, NULL, 9, NULL);
  // Create the ADC reader task
  //xTaskCreate(TaskADCReader, "ADC Reader", 128, NULL, 9, NULL);
  // Create the IMU reading task
  xTaskCreate(SensorTask,    "IMU Read", 512,  NULL, 7, NULL);
  // Create the BLE send task
  xTaskCreate(ble_uart_task, "BLE UART Task", 512, NULL, 6, NULL);
  // Create RTC task
  xTaskCreate(TaskDateTime, "RTC Task", 256, NULL, 5, NULL); 
  // Create BLE receive tasks
  xTaskCreate(ble_receive_task, "BLE RE Task", 128, NULL, 4, NULL);
  xTaskCreate(processReceivedStringTask, "Process Received String Task", 128, NULL, 3, NULL);
}


void loop() {
  // Empty. Things are managed by tasks.
}


// Start Advertising Setting
void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

// Setup the BLE LED to be enabled on CONNECT
void setupBLE(void) 
{
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName(deviceName.c_str()); // useful testing with multiple central connections getMcuUniqueID()
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("University of Queensland");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  bleConnected = true;
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  strncpy(central_name_global, central_name, 32);
  // Set the global flag to true
  
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  bleConnected = false;
  (void) conn_handle;
  (void) reason;
  // Set the global flag to flase
  imuOverflow = false;
  adcOverflow = false;
  timeReset = false;

}

bool isInCorrectFormat(const String &str) {
    // Simple check for the format "YYYY/MM/DD HH:MM:SS"
    if (str.length() != 19) return false;
    if (str.charAt(4) != '/' || str.charAt(7) != '/' || 
        str.charAt(10) != ' ' || str.charAt(13) != ':' || str.charAt(16) != ':') return false;
    // Additional checks like valid month, day, hour, etc., can be added if needed.
    return true;
}

// This function converts the pin readings into a number
int getBufferIndex() {
  int index = 0;
  index += digitalRead(PHASEN) * 8; // D9 is the MSB
  index += digitalRead(MUX3) * 4; 
  index += digitalRead(MUX2) * 2;
  index += digitalRead(MUX1);     // D8 is the LSB
  return index;
}

// This function updates the software-based clock every second
void updateClock() 
{
    DateTime now = rtc.now();
    year = now.year();
    month = now.month();
    day = now.day();
    hour = now.hour();
    minute = now.minute();
    second = now.second();
}