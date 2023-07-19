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

#include <SPI.h>
#include <TFT_eSPI.h>         // Hardware-specific library
#include <ADCInput.h>         // buffered ADC read stream
#include <math.h>             // initial build using floating point, intent to rewrite with fixed point
#include <Arduino.h>
#include <RotaryEncoder.h>

typedef union {
//    byte ar249_B[4];
//    word ar429_W[2];
    unsigned long ar429_L;
    struct 
    {
      unsigned long label:  8;  // 2,3,3 LSB
      unsigned long sdi:    2;  // source / destination identifier
      unsigned long data:  19;  // 
      unsigned long ssm:    2;  // sign / status matrix
      unsigned long parity: 1;  // odd parity MSB
    };
} ARINC429;

#define PIN_IN1 22
#define PIN_IN2 20
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

constexpr float m = 10;
// at 200ms or slower, there should be no acceleration. (factor 1)
constexpr float longCutoff = 50;
// at 5 ms, we want to have maximum acceleration (factor m)
constexpr float shortCutoff = 5;
// To derive the calc. constants, compute as follows:
// On an x(ms) - y(factor) plane resolve a linear formular factor(ms) = a * ms + b;
// where  f(4)=10 and f(200)=1
constexpr float a = (m - 1) / (shortCutoff - longCutoff);
constexpr float b = 1 - longCutoff * a;
// a global variables to hold the last position
static int lastPos, newPos;

#define INITIAL_ENCODER_VALUE -99; // used to subtract 9.9 degree offset on mechanical course reading

#define PIN_Hi_429  6
#define PIN_Lo_429  5
#define PIN_Debug   25

long target2_time;    // second core event timer value
long ARINC_value;     // value sent to ARINC display

ARINC429 dAR429;


// function copied from https://stackoverflow.com/questions/13247647/convert-integer-from-pure-binary-to-bcd
long bin2BCD(long binary) { // double dabble: 8 decimal digits in 32 bits BCD
  if (!binary) return 0;
  long bit = 0x4000000; //  99999999 max binary
  while (!(binary & bit)) bit >>= 1;  // skip to MSB

  long bcd = 0;
  long carry = 0;
  while (1) {
    bcd <<= 1;
    bcd += carry; // carry 6s to next BCD digits (10 + 6 = 0x10 = LSB of next BCD digit)
    if (bit & binary) bcd |= 1;
    if (!(bit >>= 1)) return bcd;
    carry = ((bcd + 0x33333333) & 0x88888888) >> 1; // carrys: 8s -> 4s
    carry += carry >> 1; // carrys 6s  
  }
}

word ARINC429_Permute8Bits(word label8bits)
{
  int i;
  word mask, result;

  result = 0;
  for(i = 0, mask = 0x80; i<8; i += 1, mask >>= 1)
    result |= ( (label8bits & mask) ? (1 << i) : 0 );
  return result;
}

word ARINC429_ComputeMSBParity(unsigned long ar429_L)
{
  int bitnum;
  int parity=0;
  unsigned long bitmask = 1L;

  // check all bits except parity bit
  for(bitnum = 0, bitmask = 1L; bitnum<31; bitnum += 1, bitmask <<= 1L)
  {
    if(ar429_L & bitmask)
      parity++; 
  }
  return (parity%2) ? 0 : 1;
}

unsigned long ARINC429_BuildCommand( unsigned char label, unsigned char sdi, unsigned long data, unsigned char ssm)
{

  dAR429.data = bin2BCD(data); // 19bits data
  dAR429.label = ARINC429_Permute8Bits(label);
  dAR429.sdi = sdi & 0x03;  
  dAR429.ssm = ssm & 0x03;  
  dAR429.parity = ARINC429_ComputeMSBParity(dAR429.ar429_L); // parity: MSB  

  return dAR429.ar429_L;
 }

void printbits(unsigned long value, word start, word finish, char delim)
{
  unsigned long bitnum;
  unsigned long bitmask;

  bitmask = 1L << finish;
  for(bitnum = finish; bitnum+1 > start; bitnum -- , bitmask >>= 1)
    Serial.print(value & bitmask ? "1" : "0");
  
  Serial.print(delim);
}

void ARINC429_PrintCommand( unsigned long ar429_L)
{
  int bitnum;
  unsigned long bitmask = 1L;

  dAR429.ar429_L = ar429_L;

  Serial.print("parity: ");
  Serial.println(dAR429.parity, HEX);
  Serial.print("ssm: ");
  Serial.println(dAR429.ssm, HEX);
  Serial.print("BCD data: ");
  Serial.println(dAR429.data, HEX);
  Serial.print("sdi: ");
  Serial.println(dAR429.sdi, HEX);
  Serial.print("Label (oct): ");
  Serial.println(ARINC429_Permute8Bits(dAR429.label), OCT);

  Serial.print("Bitstream: ");
  printbits(dAR429.parity,0,0,',');
  printbits(dAR429.ssm,1,2,',');
  Serial.print('[');
  printbits(dAR429.data,16,18,',');
  printbits(dAR429.data,12,15,',');
  printbits(dAR429.data,8,11,',');
  printbits(dAR429.data,4,7,',');
  printbits(dAR429.data,0,3,']');
  Serial.print(',');
  printbits(dAR429.sdi,0,1,',');
  printbits(dAR429.label,0,7,'\n');

}


void transmit_bit(char bitvalue)
{
  if(bitvalue)
  {
    digitalWrite(PIN_Hi_429, HIGH);
    digitalWrite(PIN_Lo_429, LOW);
  }
  else
  {
    digitalWrite(PIN_Hi_429, LOW);
    digitalWrite(PIN_Lo_429, HIGH);
  }
  delayMicroseconds(24);
  digitalWrite(PIN_Hi_429, LOW);
  digitalWrite(PIN_Lo_429, LOW);
  delayMicroseconds(40);
}

void ARINC429_SendCommand( unsigned long ar429_L)
{
  int bitnum;
  unsigned long bitmask = 1L;

  digitalWrite(PIN_Debug,HIGH);
  // for all bits
  for(bitnum = 0, bitmask = 1L; bitnum<32; bitnum ++, bitmask <<= 1)
  {
    transmit_bit(ar429_L & bitmask ? 1 : 0);
  }  
  digitalWrite(PIN_Debug,LOW);
}

#define ADC_READ_FREQ    8000   // frequency of adc reading (per each channel)
#define ADC_READ_BUFFERS 2500
#define ADC_OFFSET       2064   // value with no input present

ADCInput ADCInputs(A0, A1, A2);
TFT_eSPI tft = TFT_eSPI();      // Invoke custom library

#define TFT_GREY 0x5AEB
float sx = 0, sy = 1;       // Saved H, M, S x & y multipliers
uint16_t osx=120, osy=120;  // Saved H, M, S x & y coords
uint16_t x0=0, x1=0, yy0=0, yy1=0;
uint8_t ss=0;

uint32_t targetTime = 0;    // millis value for next display event
float angle;                // value sent to display

void setup(void) {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_GREY);  // Adding a background colour erases previous text automatically

  Serial.begin(115200);
//  while(!Serial);
  ADCInputs.setFrequency(ADC_READ_FREQ);
  ADCInputs.setBuffers(3, ADC_READ_BUFFERS);
  ADCInputs.begin();

  // Draw clock face
  tft.fillCircle(120, 120, 118, TFT_WHITE);
  tft.fillCircle(120, 120, 110, TFT_BLACK);

  // Draw 12 lines
  for(int i = 0; i<360; i+= 5) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*110+120;
    yy0 = sy*110+120;
    x1 = sx*108+120;
    yy1 = sy*108+120;

    tft.drawLine(x0, yy0, x1, yy1, TFT_WHITE);
  }
  // Draw 12 lines
  for(int i = 0; i<360; i+= 30) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*114+120;
    yy0 = sy*114+120;
    x1 = sx*100+120;
    yy1 = sy*100+120;

    tft.drawLine(x0, yy0, x1, yy1, TFT_WHITE);
  }


  // Draw 60 dots
  for(int i = 0; i<360; i+= 10) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*102+120;
    yy0 = sy*102+120;
     // Draw minute markers
    tft.drawPixel(x0, yy0, TFT_WHITE);

    // Draw main quadrant dots
    if(i==0 || i==180) tft.fillCircle(x0, yy0, 2, TFT_WHITE);
    if(i==90 || i==270) tft.fillCircle(x0, yy0, 2, TFT_WHITE);
  }

  tft.fillCircle(120, 121, 3, TFT_WHITE);

  targetTime = millis() + 1000; 
  draw_needle(0);

}

void draw_needle(float sdeg)
{
      // Pre-compute hand degrees, x & y coords for a fast screen update
    sx = cos((sdeg-90)*0.0174532925);    
    sy = sin((sdeg-90)*0.0174532925);

    // Redraw new hand positions

        tft.drawLine(osx-1, osy, 120-1, 121, TFT_BLACK);
//    tft.drawLine(osx, osy-1, 120, 121-1, TFT_BLACK);
//    tft.drawLine(osx, osy, 120, 121, TFT_BLACK);
    tft.drawLine(osx+1, osy, 120+1, 121, TFT_BLACK);
//      tft.drawLine(osx, osy+1, 120, 121+1, TFT_BLACK);

    osx = sx*99+121;    
    osy = sy*99+121;
    tft.drawLine(osx-1, osy, 120-1, 121, TFT_RED);
//    tft.drawLine(osx, osy-1, 120, 121-1, TFT_RED);
//    tft.drawLine(osx, osy, 120, 121, TFT_MAGENTA);
    tft.drawLine(osx+1, osy, 120+1, 121, TFT_RED);
//    tft.drawLine(osx, osy+1, 120, 121+1, TFT_RED);

    tft.fillCircle(120, 121, 3, TFT_RED);

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

  unsigned long data, ARINC_data;
  word label, sdi, ssm;


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

    angle = theta*180/M_PI ;
    angle = fmod(angle+180,360);
#if 0
    Serial.print("theta=");
    Serial.print(theta,2);
    Serial.print(theta_,2);
    Serial.print(", ");
    Serial.print("angle=");
#endif
    Serial.print("0, 360, ");
    Serial.print(angle,2);
    Serial.println();
    draw_needle(angle);

  }

}

void setup1() {
  // put your setup code here, to run once:
  // pins used by encoder
  pinMode(PIN_Hi_429, OUTPUT);
  pinMode(PIN_Lo_429, OUTPUT);
  pinMode(PIN_Debug,  OUTPUT);
 
  lastPos = newPos= INITIAL_ENCODER_VALUE;
  encoder.setPosition(lastPos);
}


void loop1() {
  // put your main code here, to run repeatedly: 
  unsigned long data, ARINC_data;
  word label, sdi, ssm;

  encoder.tick();
  newPos = encoder.getPosition();
  if (lastPos != newPos) {

    // accelerate when there was a previous rotation in the same direction.

    unsigned long ms = encoder.getMillisBetweenRotations();

    if (ms < longCutoff) {
      // do some acceleration using factors a and b

      // limit to maximum acceleration
      if (ms < shortCutoff) {
        ms = shortCutoff;
      }

      float ticksActual_float = a * ms + b;
//      Serial.print("  f= ");
//      Serial.println(ticksActual_float);

      long deltaTicks = (long)ticksActual_float * (newPos - lastPos);
//      Serial.print("  d= ");
//      Serial.println(deltaTicks);

      newPos = newPos + deltaTicks;
      encoder.setPosition(newPos);
    }

//    Serial.println(newPos);
//    Serial.print("  ms: ");
//    Serial.println(ms);
    lastPos = newPos;
  } // if

  label = 0201;  // octal
  sdi   = 0;
  ssm   = 0;

#if 1
  target2_time = millis();
  if(target2_time % 250L == 0)
  {
    //data = abs(lastPos * 10);
    ARINC_value=int(36000 + (angle*100) + (lastPos*10)) % 36000;
     ARINC_data = ARINC429_BuildCommand(label, sdi, ARINC_value, ssm);
//    ARINC429_PrintCommand(ARINC_data);
    ARINC429_SendCommand(ARINC_data);
  }
#endif

}



