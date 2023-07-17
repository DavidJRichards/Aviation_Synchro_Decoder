/*
 * Read synchro transmitter angle, display angle on serial console or plotter
 * djrm: July 16 2023, this version appears to work!
 * Target rp2040 with three channel ADC inputs fed from referencew and two synchro outputs, third op common.
 * Tracking Synchro-to-Digital Converter
 * method used copied from here: https://bikerglen.com/blog/building-a-synchro-to-digital-converter/
 *
*/

#include <ADCInput.h>         // buffered ADC read stream
#include <math.h>             // initial build using floating point, intent to rewrite with fixed point

#define ADC_READ_FREQ 10000   // frequency of adc reading (per each channel)
#define ADC_OFFSET 2064       // vause with no input present
#define PIN_LCD_CE 17         // used to disable LCD on pico explorer board

ADCInput ADCInputs(A0, A1, A2);

uint32_t targetTime = 0;                    // for next timeout
float angle;

void setup() {
  // run once:
  Serial.begin(115200);
  //pinMode(PIN_DEBUG, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // disable onboard SPI display
  pinMode(PIN_LCD_CE, OUTPUT);  
//  digitalWrite(PIN_LCD_CE, HIGH);
  
  ADCInputs.begin(ADC_READ_FREQ);

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
  static float theta=0.0;

  int refsqwv;

  // run repeatedly:  read the ADC. ( 0 to 4095 )
  // make value signed +- 2048 by applying offset
  refVal=ADCInputs.read()-ADC_OFFSET; 
  sinVal=ADCInputs.read()-ADC_OFFSET;
  cosVal=ADCInputs.read()-ADC_OFFSET;

  // note that s2ms1 isn't actually used
  // convert to float in range -2PI to +2PI
  r2mr1 = - refVal / (2048.0 / 2 * M_PI); // Vr2mr1 = -Vr1mr2 = -(Vredwht - Vblkwht)
  s1ms3 =   sinVal / (2048.0 / 2 * M_PI); // Vs1ms3 = Vylw    - Vblu
  s3ms2 =   cosVal / (2048.0 / 2 * M_PI); // Vs3ms2 = Vblu    - Vblk
//  s2ms1 =  channels[3] / 32768.0 # Vs2ms1 = Vblk    - Vylw  

  // convert reference waveform into square wave by getting its sign
  if (r2mr1 < 0)
      refsqwv = -1;
  else if (r2mr1 > 0)
      refsqwv = +1;
  else
      refsqwv = 0;

  if(refsqwv>0)
    digitalWriteFast(LED_BUILTIN, HIGH);
  else if (refsqwv<0)
    digitalWriteFast(LED_BUILTIN, LOW);                                

#if 1
  // show led when ref present
  if(refVal>0)
    digitalWriteFast(LED_BUILTIN, HIGH);
  else if(refVal<0)
    digitalWriteFast(LED_BUILTIN, LOW);                                
#endif

#if 1
  // scott t transform the inputs
  sinin = s1ms3;
  // cosin = 2/sqrt(3) * (s3ms2 + 0.5 * s1ms3)
  cosin = 1.1547 * (s3ms2 + 0.5 * s1ms3);
#else  
  // or do nothing if resolver inputs
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


  // angle display update every 250mS
  if (targetTime < millis()) 
  {
    targetTime += 50;
#if 0                                   
//    Serial.print("ref=");
    Serial.print(refVal);
    Serial.print(", ");
//    Serial.print("sin=");
    Serial.print(sinVal);
    Serial.print(", ");
//    Serial.print("cos=");
    Serial.print(cosVal);
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
    angle = theta*180/M_PI ;
    angle = fmod(angle+180,360);
//    Serial.print("theta=");
//    Serial.print(theta,2);
//    Serial.print(theta_,2);
//    Serial.print(", ");
//    Serial.print("angle=");
    Serial.print("0, 360, ");
    Serial.print(angle,2);
    Serial.println();
  }

}

// end of pico_measure_synchro
