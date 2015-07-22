/*
 * publisher_subscriber_test
 * Test for sensor_msgs::JointState publisher and std_msgs::Float32 subscriber in a same node
 * Rae Jeong :: raychanjeong@gmail.com :: MakeLab
 */

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

sensor_msgs::JointState joint_state_msg; // JointState msg for publishing

float position; 

/*
 * Called when there is new msg on the position topic. Updates the position value with the new position msg value
 */
void positionCb( const std_msgs::Float32& position_msg) {
  position = position_msg.data;
}

ros::Publisher joint_state_publisher("joint_state", &joint_state_msg); // joint_state publisher
ros::Subscriber<std_msgs::Float32> position_listener("position", &positionCb ); // position subscriber with positionCb as the call back function

void setup()
{
  nh.initNode();
  nh.advertise(joint_state_publisher);
  nh.subscribe(position_listener);

  joint_state_msg.position_length = 1; // By default, the JointState msg has it's variables as an array with 0 legnth. Usually we use resize() to change the length of the array but in the Ardino ros implemation because its memory and computational limitations, there is a variable that stores the length. Hense, position_length.
  position  = 0.0; // Default position
}

void loop()
{
  joint_state_msg.position[0] = position;
  joint_state_publisher.publish( &joint_state_msg );
  nh.spinOnce();

  delay(1000);
}