#include <EEPROM.h>

#include "I2Cdev.h"
#include "MPU6050_Wrapper.h"
#include "TogglePin.h"
#include "DeathTimer.h"
#include "MPU6050.h"

// Arduino Wire libra ry is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_PITCHROLL


#define OUTPUT_READABLE_REALACCEL
#define EEPROM_SIZE 1024


#ifdef OUTPUT_TEAPOT
// Teapot demo can only output from one MPU6050
const bool useSecondMpu = false;
MPU6050_Array mpus(1);
#else
const bool useSecondMpu = true;
MPU6050_Array mpus(useSecondMpu ? 2 : 1);
#endif

#define AD0_PIN_0 15  // Connect this pin to the AD0 pin on MPU #0
#define AD0_PIN_1 4  // Connect this pin to the AD0 pin on MPU #1

#define LED_NOTIFY 23  


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define OUTPUT_SERIAL Serial

#define MAX_COUNT 25

uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 gyro;

int arrayData[MAX_COUNT][12];
int arrayMpu0[6];
int arrayMpu1[6];

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

TogglePin activityLed(LED_PIN, 100);
DeathTimer deathTimer(5000L);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
const int enPin = 16;
const int bkPin = 17;

const int speedControlPin = 19;
const int freq = 3;
const int ledChannel = 0;
const int resolution = 8;

boolean flag = false;
#define SENSOR_COUNT 2
float storedValues[MAX_COUNT][2][6]; // Array to temporarily store roll, pitch, and yaw values for each sensor
const int stepsPerRevolution = 50;
int stepCount = 0;  // Initialize step counter variable
uint64_t eeprom_address = 0;
volatile bool mpuFlag = false;
boolean notPrinted = true;
hw_timer_t * timer = NULL;   // Declare timer as a global variable

void IRAM_ATTR saveDataInterrupt() {
  stepCount++;
  flag = true;
}

void setup() {
  pinMode(LED_NOTIFY, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(bkPin, OUTPUT);
  pinMode(speedControlPin, OUTPUT);

  digitalWrite(enPin, HIGH);
  digitalWrite(bkPin, HIGH);

  digitalWrite(LED_NOTIFY, HIGH);


  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  while (!Serial)


    // initialize device
    Serial.println(F("Initializing I2C devices..."));
  mpus.add(AD0_PIN_0);
  if (useSecondMpu) {
    mpus.add(AD0_PIN_1);
    // mpus.add(AD0_PIN_2);
  }

  mpus.initialize();

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  if (mpus.testConnection()) {
    Serial.println(F("MPU6050 connection successful"));
  } else {
    digitalWrite(enPin, HIGH);
    digitalWrite(bkPin, HIGH);
    mpus.halt(F("MPU6050 connection failed, halting"));
    
  }

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ; // empty buffer
  while (!Serial.available())
    activityLed.update(); // flash led while waiting for data
  while (Serial.available() && Serial.read())
    ; // empty buffer again
  activityLed.setPeriod(500); // slow down led to 2Hz

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  mpus.dmpInitialize();

  digitalWrite(LED_NOTIFY, LOW);
  delay(10);

  digitalWrite(enPin, LOW);
  digitalWrite(bkPin, LOW);

 
  // Configure timer interrupt for 8ms interval
  timer = timerBegin(0, 80, true);  // 80 prescaler for 1 MHz base clock
  timerAttachInterrupt(timer, &saveDataInterrupt, true);
  timerAlarmWrite(timer, 10000, true); // 8000 counts for 8ms interval
  timerAlarmEnable(timer);  // Enable the timer interrupt


  // supply your own gyro offsets here, scaled for min sensitivity
  MPU6050_Wrapper* currentMPU = mpus.select(0);
  currentMPU->_mpu.setXGyroOffset(220);
  currentMPU->_mpu.setYGyroOffset(76);
  currentMPU->_mpu.setZGyroOffset(-85);
  currentMPU->_mpu.setZAccelOffset(1353); // 1688 factory default for my test chip

  if (useSecondMpu) {
    currentMPU = mpus.select(1);
    currentMPU->_mpu.setXGyroOffset(220);
    currentMPU->_mpu.setYGyroOffset(76);
    currentMPU->_mpu.setZGyroOffset(-85);
    currentMPU->_mpu.setZAccelOffset(448); // 1688 factory default for my test chip
  }
  mpus.programDmp(0);
  if (useSecondMpu)
    mpus.programDmp(1);

}


void sensorFunc() {
  static uint8_t mpu = 0;
  static MPU6050_Wrapper* currentMPU = NULL;
  if (useSecondMpu) {
    for (int i = 0; i < 2; i++) {
      mpu = (mpu + 1) % 2; // failed attempt at round robin
      currentMPU = mpus.select(mpu);
      if (currentMPU->isDue()) {
        handleMPUevent(mpu);
      }
    }
  } else {
    mpu = 0;
    currentMPU = mpus.select(mpu);
    if (currentMPU->isDue()) {
      handleMPUevent(mpu);
    }
  }
  activityLed.update();
  deathTimer.update();
}


void handleMPUevent(uint8_t mpu) {

  MPU6050_Wrapper* currentMPU = mpus.select(mpu);
  // reset interrupt flag and get INT_STATUS byte
  currentMPU->getIntStatus();

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  if (currentMPU->_mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

    // read and dump a packet if the queue contains more than one
    while (currentMPU->_fifoCount >= 2 * currentMPU->_packetSize) {
      currentMPU->getFIFOBytes(fifoBuffer);
    }

    // read a packet from FIFO
    currentMPU->getFIFOBytes(fifoBuffer);

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetAccel(&aa, fifoBuffer);
    currentMPU->_mpu.dmpGetGyro(&gyro, fifoBuffer);
    currentMPU->_mpu.dmpGetGravity(&gravity, &q);
    currentMPU->_mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    if (mpu) {
      arrayMpu1[0] = aa.x;
      arrayMpu1[1] = aa.y;
      arrayMpu1[2] = aa.z;
      arrayMpu1[3] = gyro.x;
      arrayMpu1[4] = gyro.y;
      arrayMpu1[5] = gyro.z;
    } else {
      arrayMpu0[0] = aa.x;
      arrayMpu0[1] = aa.y;
      arrayMpu0[2] = aa.z;
      arrayMpu0[3] = gyro.x;
      arrayMpu0[4] = gyro.y;
      arrayMpu0[5] = gyro.z;
    }

  }

#endif

}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop() {
  if (notPrinted) {
    if (stepCount <  MAX_COUNT) {
      if (flag == true) {
        sensorFunc();
        for(int i = 0; i<6; i++){
          arrayData[stepCount][i] = arrayMpu0[i];
          arrayData[stepCount][6 + i] = arrayMpu1[i];
          }
        flag = false;
      }
        //stepCount++;  
    } else {
      //stop motor
      digitalWrite(enPin, HIGH);
      digitalWrite(bkPin, HIGH);
      
      digitalWrite(LED_NOTIFY, HIGH);
      serialPrintStoredValues();
      notPrinted = false; 
    }
    ///delay(1);
  }
}



void serialPrintStoredValues() {

  for (int i = 0; i < MAX_COUNT; i++) {
    for (int j = 0; j < 12; j++) {
      if ( j < 11) {
        OUTPUT_SERIAL.print(arrayData[i][j]); OUTPUT_SERIAL.print(", ");
      }
      else {
        OUTPUT_SERIAL.println(arrayData[i][j]);
      }

    }
  }
}
