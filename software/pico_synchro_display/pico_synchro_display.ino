/*

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

#define ADC_READ_FREQ 10000   // frequency of adc reading (per each channel)
#define ADC_OFFSET 2064       // vause with no input present

ADCInput ADCInputs(A0, A1, A2);
TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

#define TFT_GREY 0x5AEB
float sx = 0, sy = 1;    // Saved H, M, S x & y multipliers
uint16_t osx=120, osy=120;  // Saved H, M, S x & y coords
uint16_t x0=0, x1=0, yy0=0, yy1=0;
uint8_t ss=0;

uint32_t targetTime = 0;                    // for next timeout
float angle;

void setup(void) {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_GREY);  // Adding a background colour erases previous text automatically
  
  Serial.begin(115200);
//  while(!Serial);
  ADCInputs.begin(ADC_READ_FREQ);

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
    tft.drawLine(osx, osy, 120, 121, TFT_BLACK);
    osx = sx*99+121;    
    osy = sy*99+121;
    tft.drawLine(osx, osy, 120, 121, TFT_RED);
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

  // run repeatedly:  read the ADC. ( 0 to 4095 )
  // make value signed +- 2048 by applying offset
  refVal=ADCInputs.read()-ADC_OFFSET; 
  cosVal=ADCInputs.read()-ADC_OFFSET;
  sinVal=ADCInputs.read()-ADC_OFFSET;

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

#if 0
  if(refsqwv>0)
    digitalWriteFast(LED_BUILTIN, HIGH);
  else if (refsqwv<0)
    digitalWriteFast(LED_BUILTIN, LOW);                                
#endif

#if 0
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
    targetTime += 250;

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
#if 1
//    Serial.print("theta=");
//    Serial.print(theta,2);
//    Serial.print(theta_,2);
//    Serial.print(", ");
//    Serial.print("angle=");
    Serial.print("0, 360, ");
    Serial.print(angle,2);
    Serial.println();
#endif

    draw_needle(angle);

    }
}
