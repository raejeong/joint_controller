#include <Encoder.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <SoftwareSerial.h>


boolean limitSwitch();
void calibration();
float mapFloat(float x, float in_min, float in_max,
	       float out_min, float out_max);
float encoderRead();

const uint8_t limit_switch_pin = 7; 
const uint8_t enc_pin_a = 2;
const uint8_t enc_pin_b = 3;
const uint8_t my_serial_pin_rx = 8;
const uint8_t my_serial_pin_tx = 9;

Encoder myEnc(enc_pin_a, enc_pin_b);

const uint8_t max_forward = 255;
const uint8_t max_reverse = 0;
const uint8_t motor_stop = 127; 

SoftwareSerial mySerial(my_serial_pin_rx, my_serial_pin_tx); 


byte ATuneModeRemember=2;
double input=80, output=50, setpoint=180;
double kp=2.0,ki=0.5,kd=2.0;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

//set to false to connect to the real world
boolean useSimulation = false;

void setup()
{
  pinMode(limit_switch_pin, INPUT); 
  mySerial.begin(9600); // Serial commuication to motor controller start.

  calibration(); // Running the calibration code on every start up
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(max_reverse-motor_stop, max_forward-motor_stop);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(9600);

}

void loop()
{

  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    input = map(myEnc.read(),0,5260,0,360);
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else
  {
     mySerial.write(output); 
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
  delay(100);
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

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
