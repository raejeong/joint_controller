#include <stdio.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <WireData.h>

// I2C Commands
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

// PID ISR defines
#define ISR_PERIOD            0.002
#define ISR_FREQ              500 // (1/ISR_PERIOD)
#define CLOCK_SPEED           16000000.0
#define PRESCALER             64.0

// PID Default Constants
#define DEFAULT_POS_KP        1.5
#define DEFAULT_POS_KI        0.0
#define DEFAULT_POS_KD        (-0.01)

#define DEFAULT_VELO_KP       0
#define DEFAULT_VELO_KD       0
#define DEFAULT_VELO_KI       0

// Pin Constants
#define ENCODER_PIN           A0
#define SERIAL_RX_PIN         8
#define SERIAL_TX_PIN         9

//Serial Constants
#define HARDWARE_BAUD_RATE    9600
#define SOFTWARE_BAUD_RATE    9600

// Direction
#define POSITIVE              0
#define NEGATIVE              1

// Other Constants
#define POSITION_LOG_SIZE     5
#define PRINT_PERIOD          500   // printing loop period in ms

// Encapsulate all the data needed for computePID()
struct PID {
  
  float kP, kI, kD;
  float error, integral, derivative, prev_error;
  float dt;

};

// 7 bit I2C/TWI addresses are in the range of 0x08 to 0x77
extern const uint8_t myAddress = 0x08;

// Position Data
float setpoint = 700;             
int position_log[POSITION_LOG_SIZE];
int position_log_index = 0;
float running_position_avg = 0;

// Motor Outputs
int motor_torque = 0;
float output = 0;

// Timestamps
unsigned long last_ISR_time_stamp;
unsigned long last_loop_time_stamp; 

unsigned long debug_time_stamp_1;
unsigned long debug_time_stamp_2;

// I2C Data
uint8_t i2cCommand = I2C_COMMAND_NULL;


// Software Serial Object for Comm. with Driver
SoftwareSerial driverSerial(SERIAL_RX_PIN, SERIAL_TX_PIN); 

// PID Objects to store coefficients
struct PID position_PID;

// functions prototypes
void i2cReceive(int byteCount);
void i2cRequest();
int getMotorInput(float PID_raw);
void set_timer1_prescaler();
void setup_Timer1(int scaler);
float computePID(struct PID *pid, float error, int deltaT);
void i2cRequest();
void i2cReceive(int byteCount);

  
  
  
  
void setup()
{
  Wire.begin(myAddress);
  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);

  // Starts Serial to console
  //Serial.begin(HARDWARE_BAUD_RATE);
  
  //Starts Serial to communicate with driver
  driverSerial.begin(SOFTWARE_BAUD_RATE);
  
  // Set GPIO Pins
  pinMode(ENCODER_PIN, INPUT);
  pinMode(SERIAL_RX_PIN, INPUT);
  pinMode(SERIAL_TX_PIN, OUTPUT);

  // Read in current position of joint
  position_log[position_log_index] = analogRead(A0);
  
  // Set some default PID Constants
  position_PID.kP = DEFAULT_POS_KP;
  position_PID.kI = DEFAULT_POS_KI;
  position_PID.kD = DEFAULT_POS_KD;

  
  // Call function to setup Timer1 for Compare Match Interrupts
  setup_Timer1(PRESCALER);
  //Serial.println(OCR1A);

  // Initialize the position log array to 0
  for (int i = 0; i < POSITION_LOG_SIZE; i++) { position_log[i] = 0; }
  
  // Send 0 speed directive to motor, just in case
  driverSerial.write(127);
  
  // record current time stamp
  last_loop_time_stamp = millis();
  
}

void loop()
{
  
  //Serial.println( millis() - last_loop_time_stamp );

  // if its been enough time since last print, enter print sequence
  //if (millis() - last_loop_time_stamp >= PRINT_PERIOD)
  //{
    // Store Timestamp
    //last_loop_time_stamp = millis();
   
    // If there's incoming data, assume its a setpoint and store it
   // if (Serial.available()) { setpoint = Serial.parseFloat(); }

    // Print current status for debugging
   // Serial.print( running_position_avg ); //running_position_avg  );
    //Serial.print("    ");
   // Serial.print(setpoint);
   // Serial.print("    ");
    //Serial.print(motor_torque);
    //Serial.print("    ");
    //Serial.print(output + 127); 
    //Serial.print("    ");
    //Serial.print(1 / position_PID.dt, 9);
    //Serial.print("    ");
 
  //}
}





// Applies PID Feedback calculations
float computePID( struct PID *pid, float error, int deltaT)
{
  float dt = (deltaT/1000000.0);
  pid->dt = dt;
  pid->derivative = (pid->prev_error - error)/dt;
  pid->prev_error = pid->error;
  pid->error = error;
  pid->integral += error*dt;
  
  return (pid->kP)*(pid->error) 
          + (pid->kI)*(pid->integral) 
          + (pid->kD)*(pid->derivative);
}


// Interrupt Service Routine to calculate PID Output at a set frequency
ISR(TIMER1_COMPA_vect)
{
  
  debug_time_stamp_1 = micros();
  
  // Remove the weight of the oldest element from average
  running_position_avg -= 1.0*position_log[position_log_index]/POSITION_LOG_SIZE;
  
  // Read in new element, and overwrite oldest element
  position_log[position_log_index] = analogRead(A0);
  
  // Add in weight of newest element to average
  running_position_avg += 1.0*position_log[position_log_index]/POSITION_LOG_SIZE;
  
  // increment position log index // reset to 0 if it hits 10
  position_log_index = (position_log_index+1)%POSITION_LOG_SIZE;
  
  // Get output from PID Controller based on current conditions
  output = computePID(  &position_PID, 
                        running_position_avg - setpoint,
                        micros() - last_ISR_time_stamp );
  
  
  
  // Store the timestamp of this interrupt sequence
  last_ISR_time_stamp = micros();
  
  
  
  // Send the data to the motor driver
  motor_torque = getMotorInput(output);
  driverSerial.write( motor_torque );   // might be able to combine into 1 line
  
  
  debug_time_stamp_2 = micros() - debug_time_stamp_1;
  
}

int getMotorInput(float PID_raw)
{
  // If the raw PID value is outside bounds, cap it
  if      (PID_raw > 127)   { return 255; }
  else if (PID_raw < -127)  { return 0; }
  
  // If rae PID value is greater than a tolerance, scale it
  // 1 - 127 is mapped to 15 - 127
  else if (PID_raw < -1)    { return (int)((112.0/127.0)*PID_raw-15)+127.0; }
  else if (PID_raw > 1)     { return (int)((112.0/127.0)*PID_raw+15)+127; }
  
  // if raw PID value is too small, return 127
  else                      { return 127; }
}

// Select a Prescaler for Timer1
void set_timer1_prescaler()
{ 
  switch ((int)PRESCALER)
  {
    case 1:   TCCR1B |= ( 1 << CS10 );                  return;
    case 8:   TCCR1B |= ( 1 << CS11 );                  return;
    case 64:  TCCR1B |= ( 1 << CS10 ) | ( 1 << CS11 );  return;
    case 256: TCCR1B |= ( 1 << CS12 );                  return;
    case 1024:TCCR1B |= ( 1 << CS12 ) | ( 1 << CS10 );  return;
    default:  TCCR1B |= ( 1 << CS10 ) | ( 1 << CS10 );  return;
  }
}

// Intialize Timer1 for PID Feedback Controller
void setup_Timer1(int scaler)
{
  // Clear Timer1 Settings
  TCCR1A = 0;
  TCCR1B = 0;
  
  // Initialize Timer1 Counter to 0
  TCNT1  = 0;
  
  // Set the value of the compare match register
  OCR1A = CLOCK_SPEED / (ISR_FREQ * PRESCALER) - 1;
  
  // Set Timer1 to clear counter on compare match
  TCCR1B |= (1 << WGM12);
  
  // Set Timer1 Prescaler to 64 
  set_timer1_prescaler();
  
  // enable timer compare interrupts on Timer1
  TIMSK1 |= (1 << OCIE1A);
}

void i2cRequest()
{
  switch (i2cCommand)
  {
    // Sends current target position
      case I2C_COMMAND_JOINT_GET_SETPOINT:
        wireWriteData(setpoint);
        break;

    // Sends current position
      case I2C_COMMAND_JOINT_GET_POSITION:
        wireWriteData(running_position_avg);
        break;
    
    // Sends current KP of Position PID Loop
      case I2C_COMMAND_JOINT_GET_KP:
        wireWriteData(position_PID.kP);
        break;

    // Sends direction joint is travelling // i think!? - Swapnil
      case I2C_COMMAND_JOINT_GET_DIRECTION:
        wireWriteData(motor_torque >= 127 ? POSITIVE : NEGATIVE);
        break;

    // If I2C Command is none of the above specified, write nothing
    default:
      break;
  }
}

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
      wireReadData(position_PID.kP);
      // TODO: set KP value in PID
      i2cCommand = I2C_COMMAND_NULL;
      break;

    // The Following cases GETs are handled in the Wire request handler
      case I2C_COMMAND_JOINT_GET_SETPOINT:  
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
    
    // If the line is crap, store NULL into i2cCommand
      default:
        i2cCommand = I2C_COMMAND_NULL;
        break;
  }
}


