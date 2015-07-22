/*
 * Joint Controller
 * Joint controller node that runs on every actuator.
 * Author :: Rae Jeong
 * Email  :: raychanjeong@gmail.com
 */

#include <Encoder.h> 
#include <AltSoftSerial.h>
#include <PID_v1.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>


/*
 * Function prototypes
 */
void positionCb(const std_msgs::Float32& position_msg);
boolean limitSwitch();
void calibration();
float mapFloat(float x, float in_min, float in_max,
	       float out_min, float out_max);
float encoderRead();


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


/*
 * Reading Encoder data from pin 2 and 3. Note that we use 2 and 3 on an UNO
 * because these are the interupt pins.
 */ 
Encoder myEnc(enc_pin_a, enc_pin_b);


/*
 * Software Serial for communicating with the Syren 25A motor controller at
 * baud rate of 9600, using pin 4 and 5.
 */
AltSoftSerial mySerial(my_serial_pin_rx, my_serial_pin_tx); 


/*
 * PID controller for position control
 */
PID myPID(&input, &output, &set_point, 1, 1, 1, DIRECT);


/*
 * ROS Node
 */
ros::NodeHandle nh;

/*
 * JointState message for publishing
 */
sensor_msgs::JointState joint_state_msg;

/*
 * Publisher and Subscribers respectively: joint_state_publisher and position
 * listener with postionCb as the call back function
 */
ros::Publisher joint_state_publisher("joint_state", &joint_state_msg);
ros::Subscriber<std_msgs::Float32> position_listener("position", &positionCb);


void setup()
{
  /*
   * By default, the JointState message has it's variables as an array with 0
   * legnth. Usually we use resize() to change the length of the array but in
   * the Ardino ros implemation because its memory and computational
   * limitations, there is a variable that stores the length. Hense,
   * position_length.
   */
  joint_state_msg.position_length = 1;

  set_point  = 0.0; // Default position

  mySerial.begin(19200); // Serial commuication to motor controller start.

  pinMode(limit_switch_pin, INPUT); 

  input = encoderRead();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(max_reverse-motor_stop, max_forward-motor_stop);
  
  calibration(); // Running the calibration code on every start up

  nh.initNode();
  nh.advertise(joint_state_publisher);
  nh.subscribe(position_listener);

}


void loop()
{
  input = encoderRead();

  joint_state_msg.position[0] = input;
  joint_state_publisher.publish(&joint_state_msg);
  
  myPID.Compute();
  mySerial.write(output+motor_stop);

  delay(10);

  nh.spinOnce();
}


/*
 * Called when there is new msg on the position topic. Updates the position
 * value with the new position msg value
 */
void positionCb( const std_msgs::Float32& position_msg)
{
  set_point = position_msg.data;
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
    delay(100);
  }
  
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
