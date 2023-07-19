/*
  djrm, HSI course reader with ARINC display
  19 July 2023, arduino pico rp2040
  two core implementation
  ADC / display on first core
  Encoder / ARINC on second core

  Sketch to test accuracy of decoded values by comparing to heading display on HSI
  Heading display has an offset of about 10 degrees for some reason 
  Offset value compensated for in displayed value
  Overall accuracy appears to be within 1 degree overall.

  1) display synchro angle on display
  decode synchro transmitter with reference connected to A0, A1, A2 DAC inputs 
  display decoded angle on TFT display as meter needle on compass card
  
  2) transmit adjusted value to ARINC display
  read encoder value to get required offset (default -9.9 degrees)
  add offset to decoded transmitter value
  send value to ARINC display on HSI 

 Make sure all the display driver and pin connections are correct by
 editing the User_Setup.h file in the TFT_eSPI library folder.

 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################
 
 */

#include <Arduino.h>
#include <math.h>             // initial build using floating point, intent to rewrite with fixed point
#include <SPI.h>
#include <TFT_eSPI.h>         // Hardware-specific library
#include <ADCInput.h>         // buffered ADC read stream
#include <RotaryEncoder.h>
#include "Arinc.h"
#include "Compass.h"
#include "Encoder.h"

uint32_t targetTime = 0;    // millis value for next display event
long target2_time = 0;    // second core event timer value
long ARINC_value;     // value sent to ARINC display
int  encoder_pos;
volatile float theta=0.0;
float angle;                // value sent to display

#define INITIAL_ENCODER_VALUE -99 // used to subtract 9.9 degree offset on mechanical course reading

#define ADC_READ_FREQ    8000   // frequency of adc reading (per each channel)
#define ADC_READ_BUFFERS 2500
#define ADC_OFFSET       2064   // value with no input present

ADCInput ADCInputs(A0, A1, A2);

void waitForSerial(unsigned long timeout_millis) {
  unsigned long start = millis();
  while (!Serial) {
    if (millis() - start > timeout_millis)
      break;
  }
}

void setup(void) {

  Serial.begin(115200);
  waitForSerial(3000);
  ADCInputs.setFrequency(ADC_READ_FREQ);
  ADCInputs.setBuffers(3, ADC_READ_BUFFERS);
  ADCInputs.begin();

  targetTime = millis() + 1000; 

}

void setup1() {
  // put your setup code here, to run once:
  // pins used by encoder
  pinMode(PIN_Hi_429, OUTPUT);
  pinMode(PIN_Lo_429, OUTPUT);
  pinMode(PIN_Debug,  OUTPUT);

  encoder_init(INITIAL_ENCODER_VALUE);
  compass_init();
  draw_needle(0);

}

void loop() {
  int sinVal;
  int cosVal;
  int refVal;

  float r2mr1;
  float s1ms3;
  float s3ms2;
  //float s2ms1;

  float sinin, cosin, delta, demod;
  int refsqwv;

  // run repeatedly:  read the ADC. ( 0 to 4095 )
  // make value signed +- 2048 by applying offset
  refVal=ADCInputs.read()-ADC_OFFSET-1; 
  cosVal=ADCInputs.read()-ADC_OFFSET;
  sinVal=ADCInputs.read()-ADC_OFFSET+4;

  // note that s2ms1 isn't needed
  // convert to float in range -2PI to +2PI
//r2mr1 =  refVal / (2048.0 / 2 * M_PI); // Vr2mr1 = -Vr1mr2 = -(Vredwht - Vblkwht)
  s1ms3 =  sinVal / (2048.0 / 2 * M_PI); // Vs1ms3 = Vylw    - Vblu
  s3ms2 =  cosVal / (2048.0 / 2 * M_PI); // Vs3ms2 = Vblu    - Vblk
//s2ms1 =                                   Vs2ms1 = Vblk    - Vylw  

#if 0
  // convert reference waveform into square wave by getting its sign
  if (r2mr1 < 0)
      refsqwv = -1;
  else if (r2mr1 >= 0)
      refsqwv = +1;
  else
      refsqwv = 0;
#else
  if (refVal > 0)
      refsqwv = 1;
  else
      refsqwv = -1;
#endif

#if 0
  if(refsqwv>0)
    digitalWriteFast(LED_BUILTIN, HIGH);
  else if (refsqwv<0)
    digitalWriteFast(LED_BUILTIN, LOW);                                
#endif

#if 1 
  // scott t transform the synchro inputs
  sinin = s1ms3;
  // cosin = 2/sqrt(3) * (s3ms2 + 0.5 * s1ms3)
  cosin = 1.1547 * (s3ms2 - 0.5 * s1ms3);
#else  
  // or do nothing here if using resolver inputs
  sinin = s1ms3;
  cosin = s3ms2;
#endif  

  // compute error
  delta = sinin*cos(theta) - cosin*sin(theta);

  // demodulate AC error term
  demod = refsqwv * delta;

  // apply gain term to demodulated error term and integrate
  // theta = theta + 1/64*demod
  theta = theta + 0.015625*demod;

  // wrap from -pi to +pi
  theta = fmod((theta),(M_PI));

/*
// angle display update every 250mS
  if (targetTime < millis()) 
  {
    targetTime += 250;

#if 0                                   
//    Serial.print("-2048, 2048, ");
    Serial.print("-10, 10, ");
//    Serial.print("ref=");
    Serial.print(refVal);
    Serial.print(", ");
//    Serial.print("sin=");
    Serial.print(cosVal);
    Serial.print(", ");
//    Serial.print("cos=");
    Serial.print(sinVal);
    Serial.print(", ");
#endif

#if 0
    Serial.print(r2mr1,2);
    Serial.print(",");
    Serial.print(s1ms3,2);
    Serial.print(",");
    Serial.print(s3ms2,2);
    Serial.print(",");
#endif

#if 0
//    Serial.print("sinin=");
    Serial.print(sinin,2);
    Serial.print(", ");
//    Serial.print("cosin=");
    Serial.print(cosin,2);
    Serial.print(", ");
#endif

#if 0
    Serial.print("delta=");
    Serial.print(delta,2);
    Serial.print(", ");
    Serial.print("demod=");
    Serial.print(demod,2);
    Serial.print(", ");
#endif

#if 0 // moved to second core
    angle = theta*180/M_PI ;
    angle = fmod(angle+180,360);
    Serial.print("theta=");
    Serial.print(theta,2);
    Serial.print(theta_,2);
    Serial.print(", ");
    Serial.print("angle=");
    Serial.print("0, 360, ");
    Serial.print(angle,2);
    Serial.println();
#endif

  }
*/
}

void loop1() {
  // put your main code here, to run repeatedly: 
  unsigned long data, ARINC_data;
  word label, sdi, ssm;
  
  encoder_pos = encoder_process();

  label = 0201;   // octal message label
  sdi   = 0;      // source - destination identifiers
  ssm   = 0;      // sign status matrix

  target2_time = millis();
  if(target2_time % 250L == 0)
  {
    angle = theta*180/M_PI ;
    angle = fmod(angle+180,360);

    ARINC_value=int(36000 + (angle*100) + (encoder_pos*10)) % 36000;
     ARINC_data = ARINC429_BuildCommand(label, sdi, ARINC_value, ssm);
//    ARINC429_PrintCommand(ARINC_data);
    ARINC429_SendCommand(ARINC_data);
//    Serial.print("Enc=");
//    Serial.println(encoder_pos);
//    Serial.print("angle=");
//    Serial.print(angle,1);

    draw_needle(angle);   // draw meter pointer
  }

}



