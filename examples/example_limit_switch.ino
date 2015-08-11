#include <SoftwareSerial.h>
boolean limitSwitch();
void calibration();
const uint8_t limit_switch_pin = 7; 
const uint8_t max_forward = 255;
const uint8_t max_reverse = 0;
const uint8_t motor_stop = 127;

const uint8_t my_serial_pin_rx = 8;
const uint8_t my_serial_pin_tx = 9;

 SoftwareSerial mySerial(my_serial_pin_rx, my_serial_pin_tx); 

void setup()
{
    pinMode(limit_switch_pin, INPUT); 
    mySerial.begin(9600);
    //calibration();
}

void loop()
{
}

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
}