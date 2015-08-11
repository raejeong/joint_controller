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
#include <I2C_Anything.h>

/*
 * Address
 */

const uint8_t ADDRESS = 1;

/*
 * Function prototypes
 */
boolean limitSwitch();
void calibration();
float mapFloat(float x, float in_min, float in_max,
	       float out_min, float out_max);
float encoderRead();
void setSetPoint(int how_many);
void sendPosition();


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
double set_point;
double input;
double output;

double new_set_point = 0;


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
PID myPID(&input, &output, &set_point, 3, 0, 0, DIRECT);


void setup()
{
  Wire.begin(ADDRESS);
  Wire.onReceive(setSetPoint);
  Wire.onRequest(sendPosition);

  pinMode(limit_switch_pin, INPUT); 

  mySerial.begin(9600); // Serial commuication to motor controller start.

  set_point = 0.0;

  calibration(); // Running the calibration code on every start up

  input = encoderRead();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-127, 127);
  myPID.SetSampleTime(20);
}


void loop()
{
  input = encoderRead();
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
 * Setting a new set_point for position of the joint
 */
void setSetPoint(int how_many)
{
  if(how_many >= (sizeof new_set_point))
  {
    I2C_readAnything(new_set_point);
    set_point = new_set_point;
  }
}


/*
 * sending position, have to find a good way to send float
 */
void sendPosition()
{
  Wire.write((int)input);
}

