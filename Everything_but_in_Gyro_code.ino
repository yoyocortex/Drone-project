#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define OUTPUT_READABLE_YAWPITCHROLL //yaw, pitch and roll output
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
#define LED 13 //define pine 13 as an led pin

MPU6050 mpu; // define gyro and accelerometer module
Servo motA, motB, motC, motD; // four motors
char terminator = ('n'); 
int output = 0, less_power = 0, more_power = 0;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// PID controll One
double Setpoint, Input, Output_PID;
double Kp = 1.2, Ki = 0.05, Kd = 0.75;
PID myPID(&Input, &Output_PID, &Setpoint, Kp, Ki, Kd, DIRECT);

// PID controll Two
double Setpoint2, Input2, Output_PID2;
double Kp2=1, Ki2=0, Kd2=0; 
PID myPID2(&Input2, &Output_PID2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

float motor_speed = 1000;
void setup() {
  Serial.begin(115200); //Seral for serial.print 
  Serial1.begin(9600); // Bluetooth module serial
  //while (!Serial2);

  pinMode(LED, OUTPUT);
  pinMode(8, OUTPUT);
  motA.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //attach the motor to pins and pulse length
  motB.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //attach the motor to pins and pulse length
  motC.attach(11, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //attach the motor to pins and pulse length
  motD.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //attach the motor to pins and pulse length
  motA.writeMicroseconds(MIN_PULSE_LENGTH); //set the motor minimum pulse length
  motB.writeMicroseconds(MIN_PULSE_LENGTH); //set the motor minimum pulse length
  motC.writeMicroseconds(MIN_PULSE_LENGTH); //set the motor minimum pulse length
  motD.writeMicroseconds(MIN_PULSE_LENGTH); //set the motor minimum pulse length

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize(); // initialize the MPU6050
  devStatus = mpu.dmpInitialize();

// Offset values
  mpu.setXGyroOffset(38);
  mpu.setYGyroOffset(-11);
  mpu.setZGyroOffset(-6);
  mpu.setZAccelOffset(1603);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

// PID One setup
  Setpoint = 0;
  Setpoint2 = 0;
  myPID.SetOutputLimits(-512, 512);
  myPID.SetMode(AUTOMATIC);

// PID Two setup
  myPID2.SetOutputLimits(-512, 512);
  myPID2.SetMode(AUTOMATIC);

  digitalWrite(LED, HIGH);
  output = 1000;
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) {
    return;
  }

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt) {
    //run the code between readings of mpu6050
    if (Serial1.available() > 0) {  // if something is recieved 
      String x_char = Serial1.readStringUntil(terminator);  // read the string untill terminator
      if (x_char == "A") {
        motA.writeMicroseconds(MIN_PULSE_LENGTH);
        motB.writeMicroseconds(MIN_PULSE_LENGTH);
        motC.writeMicroseconds(MIN_PULSE_LENGTH);
        motD.writeMicroseconds(MIN_PULSE_LENGTH);
        //Serial.println("STOP");
        while (1) {
          digitalWrite(8, HIGH);
          delay(1000);
          digitalWrite(8, LOW);
          delay(1000);
        }
      }
      int x_val = x_char.toInt(); // recived data from bluetooth transform to ints
      int mapped = map(x_val, 0, 1023, 1023, 2046); // map the value
      if (mapped > 1550 and mapped != 65) { 
        output++;
        digitalWrite(LED, HIGH);
        more_power++;
        if (more_power >= 25) {
          if (mapped > 1550 and mapped != 65) {
            output = output + 5;
          }
          if (mapped < 1550 and mapped != 65) {
            more_power = 0;
          }
        }
      }
      if (mapped < 1450 and mapped != 65) {
        output--;
        digitalWrite(LED, LOW);
        less_power++;
        if (less_power >= 25) {
          if (mapped < 1450 and mapped != 65) {
            output = output - 5;
          }
          if (mapped > 1450 and mapped != 65) {
            less_power = 0;
          }
        }
      }
      motA.writeMicroseconds(output); // set the ouput value as the speed of the motor
      motB.writeMicroseconds(output); // set the ouput value as the speed of the motor
      motC.writeMicroseconds(output); // set the ouput value as the speed of the motor
      motD.writeMicroseconds(output); // set the ouput value as the speed of the motor
    }
  }
  //Serial.println("5");
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Input = ypr[1] * 180 / M_PI; // pitch value - y value
    Input2 = ypr[2] * 180 / M_PI; // roll value - x value
    myPID.Compute();
    myPID2.Compute();
    motA.writeMicroseconds(output - Output_PID + Output_PID2);
    motB.writeMicroseconds(output + Output_PID + Output_PID2);
    motC.writeMicroseconds(output + Output_PID - Output_PID2);
    motD.writeMicroseconds(output - Output_PID - Output_PID2);
    Serial.print(ypr[1]* 180/M_PI);
    Serial.print("\t");
    Serial.print(Output_PID);
    Serial.print("\t");
    Serial.print(ypr[2] * 180/M_PI);
    Serial.print("\t");
    Serial.print(Output_PID2);
    Serial.print("\t");
    Serial.println(output);
#endif
  }
}
