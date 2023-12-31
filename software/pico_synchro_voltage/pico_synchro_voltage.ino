/*
 *
 * File: Measure_rms.ino
 * Purpose: TrueRMS library example project
 * Version: 1.0.3
 * Date: 23-05-2019
 * last update: 30-09-2020
 * 
 * URL: https://github.com/MartinStokroos/TrueRMS
 * License: MIT License
 *
 *
 * This example illustrates the measurement of the rms value of a signal at the ADC input. To test, apply a sine- or a 
 * square wave of 50Hz/60Hz with 1V amplitude, biased on 2.5Vdc to input ADC0. The dcBias reading should be stable around 
 * the value 512 decimal (the middle of the ADC range) and the rms value of the sine wave should read about 0.71V and for a 
 * square wave about 1.00V.
 * 
 * The number of samples used to capture the input signal, must be a whole number. The sample window, expressed in 
 * number of samples, must have a length equal to at least one cycle of the input signal. If this is not the case, 
 * slow fluctuations in the rms and power readings will occure.
 * 
*/

#include <ADCInput.h>
#include "Synchro.h"
//#include <digitalWriteFast.h> // It uses digitalWriteFast only for the purpose of debugging!
                              // https://code.google.com/archive/p/digitalwritefast/downloads
ADCInput ADCInputs(A0, A1, A2);

#define LPERIOD 1000    // loop period time in us. In this case 1.0ms
#define ADC_INPUT 0     // define the used ADC input channel
#define RMS_WINDOW 200   // rms window of 40 samples, means 2 periods @50Hz
//#define RMS_WINDOW 50   // rms window of 50 samples, means 3 periods @60Hz

#define PIN_DEBUG 4

uint32_t targetTime = 0;       
unsigned long nextLoop;
int adcVal;
int sinVal;
int cosVal;
int refVal;
int refSqwv;

int cnt=0;
float VoltRange = 5.00; // The full scale value is set to 5.00 Volts but can be changed when using an
                        // input scaling circuit in front of the ADC.

Receiver synchro; // create an instance of Rms.


void setup() {
  // run once:
  Serial.begin(115200);
//  pinMode(PIN_DEBUG, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  ADCInputs.setFrequency(10000);
  ADCInputs.setBuffers(3, 2500);
  ADCInputs.begin();

  // configure for automatic base-line restoration and continuous scan mode:
  synchro.begin(VoltRange, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  synchro.start(); //start measuring
}



void loop() {
  // run repeatedly:
  refVal=ADCInputs.read();
  sinVal=ADCInputs.read();
  cosVal=ADCInputs.read();

//  digitalWriteFast(PIN_DEBUG, HIGH);
  synchro.update(refVal, sinVal, cosVal); // for BLR_ON or for DC(+AC) signals with BLR_OFF
//  digitalWriteFast(PIN_DEBUG, LOW);                                

  // show led when ref present
  if(synchro.refSign<0)
    digitalWriteFast(LED_BUILTIN, HIGH);
  else
    digitalWriteFast(LED_BUILTIN, LOW);                                

  if (targetTime < millis()) 
  {
    targetTime += 250;
    synchro.publish();
    Serial.print("0, -5, +5 ");
    Serial.print(synchro.rmsVal,2);
    Serial.print(", ");
    Serial.print(synchro.sinVal,2);
    Serial.print(", ");
    Serial.print(synchro.cosVal,2);
//    Serial.print(", ");
//    Serial.println(synchro.dcBias);
    Serial.println();
    //readRms.start();  // Restart the acquisition after publishing if the mode is single scan.
  }

}

// end of pico_read_synchro
