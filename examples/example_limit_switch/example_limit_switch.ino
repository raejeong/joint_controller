boolean limitSwitch();

void calibration();

const uint8_t limit_switch_pin = 7; 


void setup()
{
    pinMode(limit_switch_pin, INPUT); 
    Serial.begin(9600);
}

void loop()
{
    if(limitSwitch())
    {
	Serial.println(1);
    }
    else
    {
	Serial.println(0);
    }
    delay(500);
}

boolean limitSwitch()
{
  return !digitalRead(limit_switch_pin);
}
