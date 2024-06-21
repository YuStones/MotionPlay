//Trimmed from I2Cdev by Jeff Rowberg

//Initialize library
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <esp_now.h>
#include <WiFi.h>

//Payload ratio
int QuatRatio = 1000;
float AccelRatio = 0.1;

MPU6050 mpu;
#define INTERRUPT_PIN 19
#define LED_PIN 2
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;

uint8_t broadcastAddress[] = {0xA8, 0x42, 0xE3, 0x91, 0x31, 0xA4};

typedef struct struct_message {
  int quatW;
  int quatX;
  int quatY;
  int quatZ;
} struct_message;

struct_message data;
esp_now_peer_info_t peerInfo;

// Interrupt
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup(){
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(250000);

  //Initialize MPU
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  //Test connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  //Initialize DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();
  
  //Gyro offsets (min sensitivity), changes for each implementation
  mpu.setXGyroOffset(104.00000);
  mpu.setYGyroOffset(-27.00000);
  mpu.setZGyroOffset(-41.00000);
  mpu.setZAccelOffset(1788);
  
  if (devStatus == 0) {
    //Calibration
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    //Setup DMP
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    //Attach interrupt
    Serial.print("Enabling interrupt detection (Arduino external interrupt ");
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(")...");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt...")); //Flagging for main loop(), not sure what it does
    dmpReady = true;

    //Fetch FIFO packet size
    packetSize = mpu.dmpGetFIFOPacketSize();
  }else {
    // Error feedback
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  pinMode(LED_PIN, OUTPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop(){
  if(!dmpReady) return; //Wait for DMP to setup

  //Fetch FIFO packet
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      //Send Quaternion through serial
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      struct payloadQuat{
        int w;
        int x;
        int y;
        int z;
      };
      payloadQuat quat;
      payloadQuat signQ;

      signQ.x = -1 * (q.x < 0) + 1 * (q.x > 0);
      signQ.y = -1 * (q.y < 0) + 1 * (q.y > 0);
      signQ.z = -1 * (q.z < 0) + 1 * (q.z > 0);

      quat.w = q.w * QuatRatio + 1410000;
      quat.x = (abs(q.x) * QuatRatio + 1420000) * signQ.x;
      quat.y = (abs(q.y) * QuatRatio + 1430000) * signQ.y;
      quat.z = (abs(q.z) * QuatRatio + 1440000) * signQ.z;

      //Serial.println(quat.w);
      //Serial.println(quat.x);
      //Serial.println(quat.y);
      //Serial.println(quat.z);

      data.quatW = quat.w;
      data.quatX = quat.x;
      data.quatY = quat.y;
      data.quatZ = quat.z;

      //Real acceleration with gravity
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      struct payloadAccel{
        int x;
        int y;
        int z;
      };
      payloadAccel accel;
      payloadAccel signA;

      signA.x = -1 * (aaReal.x < 0) + 1 * (aaReal.x > 0);
      signA.y = -1 * (aaReal.y < 0) + 1 * (aaReal.y > 0);
      signA.z = -1 * (aaReal.z < 0) + 1 * (aaReal.z > 0);

      accel.x = (int)(abs(aaReal.x) * AccelRatio + 1310000) * signA.x;
      accel.y = (int)(abs(aaReal.y) * AccelRatio + 1320000) * signA.y;
      accel.z = (int)(abs(aaReal.z) * AccelRatio + 1330000) * signA.z;

      // Serial.println(accel.x);
      // Serial.println(accel.y);
      // Serial.println(accel.z);
      delay(100);
  }

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }else{
    Serial.println("Error sending the data");
  }
}