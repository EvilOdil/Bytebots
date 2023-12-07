#include<Filter.h>

// define necessary parameters
#define MIC_PIN   A0

#define NOISE 780
#define TOP 255

int maxtemp=0;
int max=0;

// define the variables needed for the audio levels
int lvl = 0, minLvl = 0, maxLvl = 50; // tweak the min and max as needed

// instantiate the filter class for smoothing the raw audio signal
ExponentialFilter<long> ADCFilter(5,0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
       // interval at which to blink (milliseconds)

void loop() {
soundlevel();

}


int soundlevel(){
  // put your main code here, to run repeatedly:
  // read the audio signal and filter it
  int n, height;
  n = analogRead(MIC_PIN);
  // remove the MX9814 bias of 1.25VDC
  n = abs(1023 - n);
  // hard limit noise/hum
  n = (n <= NOISE) ? 0 : abs(n - NOISE);
  // apply the exponential filter to smooth the raw signal
  ADCFilter.Filter(n);
  lvl = ADCFilter.Current();
//  // plot the raw versus filtered signals
//  Serial.print(n);
//  Serial.print(" ");
//  Serial.println(lvl);
  height = TOP * (lvl - minLvl) / (long)(maxLvl - minLvl);
  if(height < 0L) height = 0;
  else if(height > TOP) height = TOP;
  Serial.println(height);
  return height;
}

