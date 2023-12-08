#include "arduinoFFT.h"

arduinoFFT FFT;
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 2000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
float heighestf;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup()
{
  pinMode(13,OUTPUT);
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
}


void checkfft(){
    /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  Serial.println("Peak ");
  //println("Computed magnitudes:");
  //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);  
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  //Serial
  heighestf = FFT.MajorPeak();
  Serial.println(heighestf, 6); //Print out what frequency is the most dominant.
  //while(1); /* Run Once */
  delay(100); /* Repeat after delay */
}

void loop()
{
  checkfft();
  if(heighestf>950 && heighestf<1050){
    digitalWrite(13,HIGH);
  }else{
    digitalWrite(13,LOW);
  }
}

