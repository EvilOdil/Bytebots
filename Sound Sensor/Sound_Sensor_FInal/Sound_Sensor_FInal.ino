#include <Filter.h>


// define necessary parameters
#define MIC_PIN A0

#define NOISE 780
#define TOP 255
#define LED_PIN 13

int max = 0;
int tempmax=0;
int avg = 0;
int tempavg=0;

// define the variables needed for the audio levels
int lvl = 0, minLvl = 0, maxLvl = 50; // tweak the min and max as needed

// instantiate the filter class for smoothing the raw audio signal
ExponentialFilter<long> ADCFilter(5, 0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 0; i < 40; i++) {
    int temp = soundlevel();
    tempavg=tempavg+temp;
    if (temp > tempmax) {
      tempmax = temp;
      Serial.print("Temp ");
      Serial.print(tempmax);
      Serial.print(" ");
      Serial.println(max);
    }
    delay(100);
  }
  avg=tempavg/40;
  Serial.print(avg);
  Serial.print(" ");
  max=tempmax+avg+5;
  Serial.print("Max ");
  Serial.println(max);
  pinMode(LED_PIN, OUTPUT); // Set LED pin as an output
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW); // Make LED pin high
}

void loop() {
  // Capture the result of soundlevel() in a variable if needed
  int result = soundlevel();
  Serial.println(result);
  if(result>=max){
    digitalWrite(LED_PIN, HIGH);
  }else{
    digitalWrite(LED_PIN, LOW);
  }
  // Perform any other loop logic here
}

int soundlevel() {
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
  // plot the raw versus filtered signals
  // Serial.print(n);
  // Serial.print(" ");
  // Serial.println(lvl);
  height = TOP * (lvl - minLvl) / (long)(maxLvl - minLvl);
  if (height < 0L)
    height = 0;
  else if (height > TOP)
    height = TOP;
  int finalheight = height - max;
  //Serial.print(height);
  //Serial.print(" ");
  //Serial.println(finalheight);
  return finalheight;
}
