/*
 * Joint Controller
 * Joint controller node that runs on every actuator.
 * Author :: Rae Jeong
 * Email  :: raychanjeong@gmail.com
 */
#include <Encoder.h> 
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <ros.h> // Working in rosserial catkin. For future ROS implemantation
#include <Arduino.h>
#include <Wire.h>
#include <WireData.h>

/*
 * I2C Commands available on each joint controller.
 *
 */

#define I2C_COMMAND_NULL                        0
#define I2C_COMMAND_JOINT_SET_SETPOINT          1
#define I2C_COMMAND_JOINT_GET_SETPOINT          2
#define I2C_COMMAND_JOINT_GET_POSITION          3
#define I2C_COMMAND_JOINT_SET_KP                4
#define I2C_COMMAND_JOINT_GET_KP                5
#define I2C_COMMAND_JOINT_GET_CALIBRATION_STATE 6
#define I2C_COMMAND_JOINT_GET_DIRECTION         7
#define I2C_COMMAND_JOINT_SET_DIRECTION         8
#define I2C_COMMAND_JOINT_GET_CAL_DIRECTION     9
#define I2C_COMMAND_JOINT_SET_CAL_DIRECTION     10
#define I2C_COMMAND_JOINT_CALIBRATE             11
#define I2C_COMMAND_JOINT_HALT                  12
#define I2C_COMMAND_JOINT_HOME                  13
#define I2C_COMMAND_JOINT_MOTOR_OFF             14

/*
 * Default values, and directions.
 *
 */

#define DIRECTION_CLOCKWISE                     1
#define DIRECTION_COUNTERCLOCKWISE              0

#define DEFAULT_KP                              3.0

/*
 * Address
 */

// 7 bit I2C/TWI addresses are in the range of 0x08 to 0x77

const uint8_t myAddress = 0x08;

/*
 * Function prototypes
 */
boolean limitSwitch();
void calibration();
float mapFloat(float x, float in_min, float in_max,
	       float out_min, float out_max);
float encoderRead();
void i2cReceive(int byteCount);
void i2cRequest();


/* 
 * Pin assignments
 */
const uint8_t limit_switch_pin = 7; 
const uint8_t enc_pin_a = 2;
const uint8_t enc_pin_b = 3;
const uint8_t my_serial_pin_rx = 8;
const uint8_t my_serial_pin_tx = 9;


/*
 * motorControllerCommand
 * Gobal variable to store the value sent to the motor controller.
 * maxForward: 255 maxReverse: 0 motorStop: 127
 * maxForward, maxReverse and motorStop variables are used in the calibration.
 * Direction was chosen based on the encoder direction.
 */
const uint8_t max_forward = 255;
const uint8_t max_reverse = 0;
const uint8_t motor_stop = 127; 


/*
 * PID variables
 */
double setpoint;
double input;
double output;

double new_setpoint = 0.0;


/*
 * Reading Encoder data from pin 2 and 3. Note that we use 2 and 3 on an UNO
 * because these are the interupt pins.
 */ 
Encoder myEnc(enc_pin_a, enc_pin_b);


/*
 * Software Serial for communicating with the Syren 25A motor controller at
 * baud rate of 9600, using pin 4 and 5.
 */
SoftwareSerial mySerial(my_serial_pin_rx, my_serial_pin_tx); 

/*
 * PID controller for position control
 */
PID myPID(&input, &output, &setpoint, DEFAULT_KP, 0, 0, DIRECT);


/*
 * System globals
 */
uint8_t i2cCommand = I2C_COMMAND_NULL;
uint8_t direction = DIRECTION_CLOCKWISE;
uint8_t calibrationDirection = DIRECTION_CLOCKWISE;
boolean calibrated = false;
double new_kP = DEFAULT_KP;
double old_kP = DEFAULT_KP;


void setup()
{
  Wire.begin(myAddress);
  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);

  pinMode(limit_switch_pin, INPUT); 

  mySerial.begin(9600); // Serial commuication to motor controller start.

  setpoint = 0.0;

  calibration(); // Running the calibration code on every start up

  input = encoderRead();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-127, 127);
  myPID.SetSampleTime(20);
}


void loop()
{
  input = encoderRead();

  if(new_kP != old_kP)
  {
      myPID.SetTunings(new_kP,0,0);
      old_kP = new_kP;
  }

  myPID.Compute();
  double motorInput;
  if (output < -1)
  {
    motorInput = (112.0/127.0)*output -15;
  }
  else if (output > 1)
  {
    motorInput = (112.0/127.0)*output +15;
  }
  else
  {
    motorInput = output;
  }

  motorInput = motorInput+127.0;

  mySerial.write((int)motorInput);
  delay(120);
}


/*
 * When called, limitSwitch() returns true when the limit switch is pressed 
 * and false when the limit switch is not pressed.
 */
boolean limitSwitch()
{
  return !digitalRead(limit_switch_pin);
}


/*
 * calibration() fucntion that is called every time the system starts up
 * Is responsible for hitting the limit switch and resetting the encoder to zero
 */
void calibration()
{
  /*
   * If the motor starts on the limitSwitch, go reverse until the limit switch
   * is no longer triggered.
   */
  if(limitSwitch())
  { 
    while(limitSwitch()) 
    {
      mySerial.write(max_reverse);
    }
  }

  delay(100);
  
  // Go forward until the motor triggers the limit switch.
  while(!limitSwitch())
  {
    mySerial.write(max_forward);
  }

  // Stop the motor
  mySerial.write(motor_stop);

  /*
   * Reset the encoder. The delay is there becasue the motor does not stop
   * instantaneously.
   */
  delay(500);
  myEnc.write(0);
}


/*
 * mapFloat is meant to execute mapping for floats
 */
float mapFloat(float x, float in_min,
	       float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/*
 * encoderRead() function is used to read the encoder value in degrees
 */
float encoderRead()
{
  return mapFloat(myEnc.read(), 0, 5260, 0, 360);
}


/*
 * i2c request handler
 */
void i2cRequest()
{
  switch (i2cCommand)
  {
    case I2C_COMMAND_JOINT_GET_SETPOINT:
      wireWriteData(setpoint);
      break;
    case I2C_COMMAND_JOINT_GET_POSITION:
      wireWriteData(input);
      break;
    case I2C_COMMAND_JOINT_GET_KP:
	wireWriteData(old_kP);
      break;
    case I2C_COMMAND_JOINT_GET_CALIBRATION_STATE:
      wireWriteData(calibrated);
      break;
    case I2C_COMMAND_JOINT_GET_DIRECTION:
      wireWriteData(direction);
      break;
    case I2C_COMMAND_JOINT_GET_CAL_DIRECTION:
      wireWriteData(calibrationDirection);
      break;
    default:
      break;
  }
}


/*
 * Get command and optional data from I2C
 */
void i2cReceive(int byteCount)
{
  i2cCommand = Wire.read();

  switch (i2cCommand)
  {
    case I2C_COMMAND_JOINT_SET_SETPOINT:
      wireReadData(setpoint);
      i2cCommand = I2C_COMMAND_NULL;  // clear the command, since we have completed all processing (set)
      break;

    case I2C_COMMAND_JOINT_SET_KP:
	wireReadData(new_kP);
      // TODO: set KP value in PID
      i2cCommand = I2C_COMMAND_NULL;
      break;

    case I2C_COMMAND_JOINT_SET_DIRECTION:
      wireReadData(direction);
      // TODO: any post processing for direction change
      i2cCommand = I2C_COMMAND_NULL;
      break;

    case I2C_COMMAND_JOINT_SET_CAL_DIRECTION:
      wireReadData(calibrationDirection);
      // TODO: any post processing for calibration direction change
      i2cCommand = I2C_COMMAND_NULL;
      break;

    case I2C_COMMAND_JOINT_GET_SETPOINT:  // Each of the GETs are handled in the Wire request handler
    case I2C_COMMAND_JOINT_GET_POSITION:
    case I2C_COMMAND_JOINT_GET_KP:
    case I2C_COMMAND_JOINT_GET_CALIBRATION_STATE:
    case I2C_COMMAND_JOINT_GET_DIRECTION:
    case I2C_COMMAND_JOINT_GET_CAL_DIRECTION:
      break;

    case I2C_COMMAND_JOINT_CALIBRATE:
    case I2C_COMMAND_JOINT_HALT:
    case I2C_COMMAND_JOINT_HOME:
    case I2C_COMMAND_JOINT_MOTOR_OFF:
    default:
      i2cCommand = I2C_COMMAND_NULL;
      break;
  }
}

