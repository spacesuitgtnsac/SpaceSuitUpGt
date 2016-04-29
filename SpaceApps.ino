
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define touch 13
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
short conteo=0;
bool flag=false;

char comando;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
    while (!Serial);
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    }
    pinMode(touch, INPUT);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
    display.display();
    delay(2000);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
     display.clearDisplay();
     if(digitalRead(touch)==1){
      flag=true;
     }
     if(flag){
      if(digitalRead(touch)==0){
        conteo+=1;
        flag=false;
        if(conteo==4){
          conteo=0;
        }
      }
     }
     
    
      
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);        
        fifoCount -= packetSize;

         mpu.dmpGetQuaternion(&q, fifoBuffer);
         mpu.dmpGetGravity(&gravity, &q);
         mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
         ypr[1]=ypr[1] * 180/M_PI;
         ypr[2]=ypr[2] * 180/M_PI;

         switch(conteo){
          case 0:
            display.setCursor(32,10);
            display.println("HOLD");
            display.display();
            break;
          case 1:
            display.setCursor(32,10);
            display.println("ROBOT");
            display.display();
            if(ypr[2]>45){
              comando='x';
            }
            else if(ypr[2]<-45){
              comando='w';
            }
            else if(ypr[1]>45){
              comando='a';
            }
            else if(ypr[1]<-45){
              comando='d';
            }
            else{
              comando='s';
            }
            break;
          case 2:
            display.setCursor(40,10);
            display.println("ARM");
            display.display();
            if(ypr[2]>45){
              comando='m';
            }
            else if(ypr[2]<-45){
              comando='r';
            }
            else{
              comando='s';
            }
            break; 
          case 3:
            display.setCursor(32,10);
            display.println("CAMERA");
            display.display();
            if(ypr[2]>45){
              comando='g';
            }
            else if(ypr[2]<-45){
              comando='f';
            }
            else{
              comando='s';
            }
            break; 
           
          }

         Serial.print(comando);          
    }
}
