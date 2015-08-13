#include <Encoder.h>
#include <SoftwareSerial.h>


const uint8_t enc_pin_a = 2;
const uint8_t enc_pin_b = 3;
const uint8_t my_serial_pin_rx = 8;
const uint8_t my_serial_pin_tx = 9;

Encoder myEnc(enc_pin_a, enc_pin_b);
SoftwareSerial mySerial(my_serial_pin_rx, my_serial_pin_tx);


void setup()
{
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  mySerial.write(255);
  Serial.println(myEnc.read());
}
