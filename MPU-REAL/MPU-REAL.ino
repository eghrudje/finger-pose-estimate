
#include "I2Cdev.h"
#include "MPU6050_Wrapper.h"
#include "TogglePin.h"
#include "DeathTimer.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif



//#define OUTPUT_READABLE_EULER

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_PITCHROLL


//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

//#define OUTPUT_TEAPOT


#ifdef OUTPUT_TEAPOT
// Teapot demo can only output from one MPU6050
const bool useSecondMpu = false;
MPU6050_Array mpus(1);
#else
const bool useSecondMpu = true;
MPU6050_Array mpus(useSecondMpu ? 3 : 1);
#endif

#define AD0_PIN_0 2  // Connect this pin to the AD0 pin on MPU #0
#define AD0_PIN_1 4  // Connect this pin to the AD0 pin on MPU #1
#define AD0_PIN_2 7


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define OUTPUT_SERIAL Serial
//#define OUTPUT_SERIAL Serial2
//#define OUTPUT_SERIAL softSerial


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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

TogglePin activityLed(LED_PIN, 100);
DeathTimer deathTimer(5000L);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);

  while (!Serial)
    

  // initialize device 
  Serial.println(F("Initializing I2C devices..."));
  mpus.add(AD0_PIN_0);
  if (useSecondMpu) {
    mpus.add(AD0_PIN_1);
    mpus.add(AD0_PIN_2);
    }

  mpus.initialize();

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  if (mpus.testConnection()) {
    Serial.println(F("MPU6050 connection successful"));
  } else {
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

  // supply your own gyro offsets here, scaled for min sensitivity
  MPU6050_Wrapper* currentMPU = mpus.select(0);
  currentMPU->_mpu.setXGyroOffset(220);
  currentMPU->_mpu.setYGyroOffset(76);
  currentMPU->_mpu.setZGyroOffset(-85);
  currentMPU->_mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  currentMPU = mpus.select(1);
  currentMPU->_mpu.setXGyroOffset(220);
  currentMPU->_mpu.setYGyroOffset(76);
  currentMPU->_mpu.setZGyroOffset(-85);
  currentMPU->_mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  if (useSecondMpu) {
    currentMPU = mpus.select(2);
    currentMPU->_mpu.setXGyroOffset(220);
    currentMPU->_mpu.setYGyroOffset(76);
    currentMPU->_mpu.setZGyroOffset(-85);
    currentMPU->_mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  }
  mpus.programDmp(0);
  if (useSecondMpu)
    mpus.programDmp(1);
    mpus.programDmp(2);

}


  
  void sensorFunc() {
     static uint8_t mpu = 0;
     static MPU6050_Wrapper* currentMPU = NULL;
  if (useSecondMpu) {
    for (int i=0;i<3;i++) {
      mpu=(mpu+1)%3; // failed attempt at round robin
      currentMPU = mpus.select(mpu);
      if (currentMPU->isDue()) {
        handleMPUevent(mpu);
      }
    }
  } else {
    mpu=0;
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
      // read and dump one sample
     // Serial.print("DUMP"); // this trace will be removed soon
      currentMPU->getFIFOBytes(fifoBuffer);
    }

    // read a packet from FIFO
    currentMPU->getFIFOBytes(fifoBuffer);
/*

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetEuler(euler, &q);
    OUTPUT_SERIAL.print("euler:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(euler[0] * 180 / M_PI);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(euler[1] * 180 / M_PI);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.println(euler[2] * 180 / M_PI);
#endif

#if defined(OUTPUT_READABLE_YAWPITCHROLL) or defined(OUTPUT_READABLE_PITCHROLL)
    // display Euler angles in degrees
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetGravity(&gravity, &q);
    currentMPU->_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if defined(OUTPUT_READABLE_YAWPITCHROLL)
    OUTPUT_SERIAL.print("y");
#endif
    OUTPUT_SERIAL.print("pr:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
#if defined(OUTPUT_READABLE_YAWPITCHROLL)
    OUTPUT_SERIAL.print(ypr[0]);
    OUTPUT_SERIAL.print("\t");
#endif
    OUTPUT_SERIAL.print(ypr[1]);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.println(ypr[2]);
//    OUTPUT_SERIAL.println(ypr[2] * 180 / M_PI);
    
#endif
*/
#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetAccel(&aa, fifoBuffer);
    currentMPU->_mpu.dmpGetGyro(&gyro, fifoBuffer);
    currentMPU->_mpu.dmpGetGravity(&gravity, &q);
    currentMPU->_mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //OUTPUT_SERIAL.print("areal:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(aaReal.x);
    OUTPUT_SERIAL.print(",");
    OUTPUT_SERIAL.print(aaReal.y);
    OUTPUT_SERIAL.print(",");
    OUTPUT_SERIAL.print(aaReal.z);
    OUTPUT_SERIAL.print(",");
    OUTPUT_SERIAL.print(gyro.x);
    OUTPUT_SERIAL.print(",");
    OUTPUT_SERIAL.print(gyro.y);
    OUTPUT_SERIAL.print(",");
    OUTPUT_SERIAL.println(gyro.z);
#endif
/*
#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetAccel(&aa, fifoBuffer);
    currentMPU->_mpu.dmpGetGravity(&gravity, &q);
    currentMPU->_mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    currentMPU->_mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    OUTPUT_SERIAL.print("aworld:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(aaWorld.x);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(aaWorld.y);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.println(aaWorld.z);
#endif

*/
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop() {

  sensorFunc();
}
