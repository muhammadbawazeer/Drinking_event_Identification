// Modules and Libraries

#include "weight.h"  // not used
#include "SSD1306Wire.h"
#include "IMU.h" 

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


#define battery_code "1"
#define drinking_code "2"
bool drinking;
String message;
int consumed;
char accepted_message[20];

// Initialise ESP32 cores
TaskHandle_t Task1, Task2;


// Initialise Sensors
SSD1306Wire  display(0x3c, 19, 21);
IMU mpu;


//////////////////////////BLE Configuration////////////////////////////////

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLe callback function
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

/////////////////////////declare available function////////////////////


void init();
void start_ble();
void drawTextAlignmentDemo(String msg);

// attribute
long start_display;

//////////////////////////////////////////////////////////////////////


void setup() {

  Serial.begin(115200);
  Serial.println("init starting");
  init();
  delay(10);
  Serial.println("init done");
 
  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, 0, 1, &Task1, 0);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, 0, 2, &Task2, 1);
  
}

void init()
{   
//  scale.begin(25, 33);
  display.init();
  
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  delay(5);
  drawTextAlignmentDemo("FirstDrink");
  display.display();
  delay(3000);
  display.clear();
  drawTextAlignmentDemo("Ready");
  display.display();
  
}

void loop() {
  /* Update all the values */


}


void Task1code( void *pvParameters)
{ 
  start_ble();
 
  while(1){
  // notify changed value
  if (deviceConnected) {
      if (drinking){
        message.toCharArray(accepted_message, 20);
        pCharacteristic->setValue(accepted_message);
        pCharacteristic->notify();
        delay(5); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
        Serial.print("message sent: "); Serial.println(accepted_message);
        drinking = false;
        }
      else{
        Serial.println("non drinking");
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
   delay(5);
  }
  
}




void Task2code( void *pvParameters)
{ 
  mpu.init(22, 23);

  while(1){
    if (start_display)
      if (millis() - start_display > 3000)
        {
          display.init();
          display.clear();
          delay(5);
          mpu.init(22, 23);
        }
    
    mpu.read_IMU();
  
    if (mpu.Angle.Y > mpu.Min_angle_threshold){
      Serial.println("tracking activity triggered");
      drinking = mpu.track_activity(mpu.Angle.Y);
      
      if (drinking){
        consumed = random(25,70); // currently we set volume to random, we are still conducting a research on regression for liquid intake
        
        message = drinking_code + String(":") + String(consumed);
        
        display.init();
        display.flipScreenVertically();
        display.setFont(ArialMT_Plain_24);
        drawTextAlignmentDemo(String(consumed));
        display.display();
        delay(3);
        start_display = millis();
        
        mpu.init(22, 23);
        }
      }
      delay(5);
    }
}  


////////////////////////////////////////////////////////////////////////////////////////////////////


void drawTextAlignmentDemo(String msg) {

  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 22, msg);

}



void start_ble(){
  
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

