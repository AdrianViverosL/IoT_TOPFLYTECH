/**
 * IoT SYSTEM ESP32
 * Gets the Temperature, Humidity, Battery and Scene from a TSTH1-B
 * TOPFLYTech's BLE server and it fowards the data to the internet 
 * Use Blynk Cloud Connection
 * Generate a Trama of 12 bytes for SerialCom 
 *                    TSTH1-B
 * ID       MAC      Battery  Temperature   Hum   Scene
 * 0    1 2 3 4 5 6     7        8  9       10     11
 *                     TSR1
 * ID       MAC      Relay Status    NULL   
 * 0    1 2 3 4 5 6        7       8 9 10 11
 * 
 * Author Solovino
 * Property of DECA Company
 * Special thanks to Neil Kolban & Chegewara 
 * for their work and support on BLE for ESP32
 **/

#define uS_TO_S_FACTOR 1000000    //Constants for deep sleep 
#define TIME_TO_SLEEP 60
#define BLYNK_PRINT Serial

#define TX2 14
#define RX2 12

#include <Arduino.h>
#include "BLEDevice.h"
#include <BlynkSimpleEsp32.h>
//#include <TinyGsmClient.h>
#include <WiFi.h>
//#include <WiFiClient.h>
#include <HardwareSerial.h>
#include <WiFiUdp.h>

TaskHandle_t fetchData;
TaskHandle_t UploadData;

SemaphoreHandle_t sema_control;

HardwareSerial MySerial(2);
//BlynkTimer timer;
WidgetLED scene(V8);
WiFiUDP udp;

//Auth Token Blynk App
char auth[] = "PnzC0e83XmXJI4FjKcsWVbr45HsL2LUx";

//WiFi credentials
char ssid[] = "FamVivLu_2.4Gnormal";
char pass[] = "PedroBigotes";

const char * udpAddress = "192.168.0.27";
const int udpPort = 7778;
 

// UUID's for the remote service and its characteristic. 
static BLEUUID serviceUUID("27760001-999c-4d6a-9fc4-c7272be10900");
static BLEUUID    charUUID("27763561-999c-4d6a-9fc4-c7272be10900");

//BLE objects declaration
static BLERemoteCharacteristic* pRemoteCharacteristic; 
static BLEAdvertisedDevice *myDevice;
static BLEClient* pClient;

//Logic Control Flags
static boolean doConnect = false; 
static boolean connected = false;
static boolean established = false;
RTC_DATA_ATTR int control = 0;  

//Other Variables
const float scale = 100.0;
int lecture;
int count = 0;
String clue = "";
int srchlmt = 0;

//Data received variables
int batterylvl;
float tempf;
int humidity;


//Buffer Initialization 
byte databuffer[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const size_t databufferLength = sizeof(databuffer) / sizeof(databuffer[0]);

//Board ID for indetification in the data basis
int id = 2581;

/*--------------------------------------------------*/
/*-------- BLE FunctionS/Classes/Callbacks ---------*/
/*--------------------------------------------------*/

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
  
 /* Serial.print("Notify callback for characteristic ");
  for(int i=0; i<length; i++){
    Serial.print(pData[i]);
    Serial.print(" ");                     
  }
  Serial.println();
  */
  
  if(length == 10){

    databuffer[0] = 68; //Payload Header meaning a successfull connection with the BLE server.
    //databuffer[0] = '73';

    //Battery leve for debugging
    Serial.print("Battery Level: ");
    Serial.print(pData[2]);
    Serial.println("%");
    batterylvl = pData[2];
    //char battlvl = pData[2];

    databuffer[11] = pData[2];
    //databuffer[7] = battlvl;
    Blynk.virtualWrite(V7, batterylvl);
  
    //Two bytes [3][4] from the reply trama to Temperature
    int16_t tempc;
    tempc = ((pData[3] << 8) + pData[4]);
    tempf= tempc/scale;

    databuffer[12] = pData[3];
    databuffer[13] = pData[4];

    //databuffer[8] = char(pData[3]);
    //databuffer[9] = char(pData[4]);
    Blynk.virtualWrite(V5, tempf);
    
    //Temperature value for debugging
    Serial.print("Temperature: ");
    Serial.print(tempf,2);
    Serial.println("ºC");
  
    //Humidity value for debugging
    Serial.print("Humidity: ");
    Serial.print(pData[5]);
    Serial.println("%");
    humidity = pData[5];
    databuffer[14] = pData[5];
    //databuffer[10] = char(pData[5]);
    Blynk.virtualWrite(V6, humidity);

    //Scene status 
    Serial.print("Scene: ");
    if(pData[7]== 1){
      Serial.println("Light");
      scene.on();
      databuffer[15] = 1;
    }else{
      Serial.println("Dark");
      scene.off();
      databuffer[15] = 0;
    }
    
    Serial.print("Payload: ");
    for(int j=0; j<16; j++){
      Serial.print(databuffer[j]);
      Serial.print(" ");
     }
    Serial.println();
    lecture++;         
  }
  else if(length == 4){
    databuffer[0] = 68;
    //Relay status 
    Serial.print("Relay Status: ");
    if(pData[2]== 1){
      Serial.println("ON");
      databuffer[11] = 1;
    }else{
      Serial.println("OFF");
      databuffer[11] = 0;
    }
    
    databuffer[12] = 0;
    databuffer[13] = 0;
    databuffer[14] = 0;
    databuffer[15] = 0;

    Serial.print("Payload: ");
    for(int j=0; j<16; j++){
      Serial.print(databuffer[j]);
      Serial.print(" ");
     }
    Serial.println();
  }
  else{
    Serial.println("Error at Reply String");
    //established = false;
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) {
    connected = true;
    Serial.println("   -- onConnect");
  }

  void onDisconnect(BLEClient* pClient) {
    connected = false;
    Serial.println("   -- onDisconnect");
  }
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice){  
   
    Serial.print("-");
    String advDeviceName = String(advertisedDevice.getName().c_str());
    //bool isTempSensor = advDeviceName.startsWith("TST");
    //bool isRelay = advDeviceName.startsWith("TSR");

    bool isMyDevice = advDeviceName.startsWith(clue);
    if(isMyDevice){
      Serial.println();
      Serial.print("Found a device! RSSI: ");
      Serial.println(advertisedDevice.getRSSI());
        
      advertisedDevice.getScan()->stop();

      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  } // onResult
}; // MyAdvertisedDeviceCallbacks

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getName().c_str());
    //deviceName = String(myDevice->getName().c_str());
    
    //BLEClient*  pClient  = BLEDevice::createClient();
    pClient = BLEDevice::createClient();
    //Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());
    //Serial.println(" - Connecting to BLE server");
    
    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    
    if(connected == true){
      // Obtain a reference to the service we are after in the remote BLE server.
      BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
      if (pRemoteService == nullptr) {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
      }
      //Serial.println(" - Found our service");
  
      // Obtain a reference to the characteristic in the service of the remote BLE server.
      pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
      if (pRemoteCharacteristic == nullptr) {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charUUID.toString().c_str());
        pClient->disconnect();
        return false;
      }
      //Serial.println(" - Found our characteristic");
  
      if(pRemoteCharacteristic->canNotify()){
        //Serial.println("Registered for notifications");
        pRemoteCharacteristic->registerForNotify(notifyCallback);
      }else{
        Serial.println("Not possible to register notifications");
        return false;
      }
      established = true;
      return true;
    }
    else{
      Serial.println("BLE Server Suddenly disconnected");
      return false;
    }
}

/*--------------------------------------------------*/
/*---------------- Sensor & Device -----------------*/
/*------------------- Functions --------------------*/
/*--------------------------------------------------*/

//CRC function from the manufacturer datasheet [CHECKSUM]
uint8_t xCal_crc(uint8_t *ptr, uint8_t len){
  uint8_t crc;
  uint8_t i;
  crc = 0xff;
  while(len--){
    crc ^= *ptr++;
    for(i=0; i<8; i++){
      if(crc & 0x80){
        crc = (crc<<1)^0x31; //G(x)=x8+x5+x4+1
      }else{
        crc <<=1;
      }
    }
  }
  return crc;
}

bool udpSend(int array_size){
  //This functions receives the size of the buffer and send it to the server using UDP
  int pkt_size = array_size;
  udp.beginPacket(udpAddress, udpPort);
  udp.write(databuffer, pkt_size);
  udp.endPacket();
  memset(databuffer, 0, pkt_size);
  return true;
}

void insertMACintoBuffer(){
  //Obtain the native address and store it into an array of bytes
  //byte myDevice_address[6];
  byte* myDevice_address = NULL;
  myDevice_address = new byte[6];
  memcpy(myDevice_address, myDevice->getAddress().getNative(), 6);

  databuffer[5] = myDevice_address[0];
  databuffer[6] = myDevice_address[1];
  databuffer[7] = myDevice_address[2];
  databuffer[8] = myDevice_address[3];
  databuffer[9] = myDevice_address[4];
  databuffer[10] = myDevice_address[5];
}

void controlRelay(){

    if(Serial.available()){
      String incomingData = Serial.readStringUntil('\n');     
      if(incomingData.equals("a")){
        Serial.println("Turnning ON");
        uint8_t preTrama2[] = {0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x6c, 0x01};
        uint8_t crcVal2 = xCal_crc((uint8_t*)preTrama2, 8);
        const uint8_t trama2[] = {0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x6c, 0x01, crcVal2};
        pRemoteCharacteristic->writeValue((uint8_t*)trama2, 9, true);
      }else if(incomingData.equals("b")){
        Serial.println("Turnning OFF");
        uint8_t preTrama3[] = {0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x6c, 0x00};
        uint8_t crcVal3 = xCal_crc((uint8_t*)preTrama3, 8);
        const uint8_t trama3[] = {0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x6c, 0x00, crcVal3};
        pRemoteCharacteristic->writeValue((uint8_t*)trama3, 9, true);
      }else if(incomingData.equals("d")){
        Serial.println("Disconnecting from Relay");
        //connected = false; 
        pClient->disconnect();
        established = false; 
        }    
      }
}

void readTempSensor(){
  clue = "TSTH"; 
  if(doConnect == true){
      srchlmt = 0;
      if(connectToServer()){

        insertMACintoBuffer();
        databuffer[3] = 2;    //Assing number of device
        databuffer[4] = 73;   //Adding confirmation of successfull conection with the device
        Serial.println("Connection to Temp & Humidity sensor successful");       
        while(established){
          if(lecture < 1){
            uint8_t preTrama[] = {0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x04};
            uint8_t crcVal = xCal_crc((uint8_t*)preTrama, 7);
            const uint8_t trama[] = {0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x04, crcVal};
            pRemoteCharacteristic->writeValue((uint8_t*)trama, 8, true); 
          }
          else{
            Serial.println("Disconnecting from the BLE server");      
            lecture = 0;

            pClient->disconnect(); 

            Serial.print("Sending data through serial port........");
            
            MySerial.write(databuffer, sizeof(databuffer));
            Serial.write(databuffer, sizeof(databuffer));
            MySerial.flush();  //wait the outgoing transmition to be complete
            delay(500);
            Serial.println("         Done");

            //udpSend(sizeof(databuffer));
           
            delete myDevice;
            myDevice = NULL;
            established = false;
            control = 2;
            }
        } 
      }
      else{
        Serial.println("Failed to connect to the server");
      }
    doConnect = false; 
  }
  else{
    BLEDevice::getScan()->start(1,false);
    Serial.println(srchlmt);
    if(srchlmt == 10){
      Serial.println("Search limit reached, Device not available");
      databuffer[0] = 68;
      databuffer[3] = 2;
      databuffer[4] = 84;
      
      Serial.print("Sending data through serial port........");
      MySerial.write(databuffer, databufferLength);
      Serial.println("         Done");

      //Send frame to the server using UDP
      //udpSend(5);

      srchlmt = 0;
      control = 2;
    }
    srchlmt++;
  }
}

void checkRelay(){
  clue = "TSR";
  if(doConnect == true){
    srchlmt = 0;
      if(connectToServer()){

        insertMACintoBuffer();
        databuffer[3] = 1;    //Assing number of device
        databuffer[4] = 73;   //Adding confirmation of successfull conection with the device
        Serial.println("Connection to the Relay successful");
        if(established){
          uint8_t preTramaRe[] = {0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x6d};
          uint8_t crcValRe = xCal_crc((uint8_t*)preTramaRe, 7);
          const uint8_t tramaRe[] = {0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x6d, crcValRe};
          pRemoteCharacteristic->writeValue((uint8_t*)tramaRe, 8, true);
          control = 1;
          pClient->disconnect(); 
        }
      }
      else{
        Serial.println("Failed to connect to the server");
        return;
      }
    doConnect = false; 
  }
  else{
    BLEDevice::getScan()->start(1,false);
    Serial.println(srchlmt);
    if(srchlmt == 1){
      Serial.println("Search limit reached, Relay not available");

      databuffer[0] = 68;
      databuffer[3] = 1;
      databuffer[4] = 84;

      //Serial 2 communication with GPS Suntech if needed
      Serial.print("Sending data through serial port........");
      Serial.write(databuffer, sizeof(databuffer));
      MySerial.write(databuffer, sizeof(databuffer));
      MySerial.flush();
      Serial.println("         Done");

      /**
      if(udpSend(5)){
        Serial.println("Data packet UDP sended");  
      }
      else{
        Serial.println("Data packet UDP Failed");
      }
    **/
      srchlmt = 0; 
      control = 1;
      xSemaphoreGive(sema_control);
    }
    srchlmt++;
  }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void FetchData(void * parameter)  // This is a task.
{
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(100);  //760 OG value
  pBLEScan->setWindow(99);     //700 OG value
  pBLEScan->setActiveScan(true);
  for (;;) // A Task shall never return or exit.
  {
    databuffer[1] = highByte(id);
    databuffer[2] = lowByte(id); 
    switch (control){
      case 0:
        Serial.println("Searching Relay Device");
        vTaskDelay(2500);
        checkRelay();
        vTaskDelay(1000);
        break;
      case 1:
        Serial.println("Searching Temperature Sensor");
        vTaskDelay(2500);
        readTempSensor();
        break;
      case 2:
        control = 0;
        Serial.println("   ----- Taking a nap -----");
        esp_deep_sleep_start();
        break;
    }
    
  } 
}

void TaskWifi(void * parameter)  // This is a task.
{  
  /**
   * Blynk.connectWiFi(ssid, pass);
  Blynk.config(auth, "blynk-cloud.com", 80);
  Blynk.connect();**/

  Blynk.begin(auth, ssid, pass);
  //This initializes udp and transfer buffer
  //udp.begin(udpPort);

  for(;;)
  { 
    Blynk.run();
  }
}

void setup() {

  Serial.begin(115200);
  MySerial.begin(115200, SERIAL_8N1, 34, 32);
  Serial.println("Starting IoT Truck System...");

  //Set up Deep Sleep Mode
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  xTaskCreatePinnedToCore(
    FetchData,
    "Search & Collect data from the Devices",
    4096,
    NULL,
    0,
    &fetchData,
    0
    );

  xTaskCreatePinnedToCore(
    TaskWifi,
    "Upload to Blynk",
    2048,
    NULL,
    1,
    &UploadData,
    1
    );
  
  sema_control = xSemaphoreCreateBinary();
}

void loop() {

}


