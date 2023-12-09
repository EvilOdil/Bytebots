// connect motor controller pins to Arduino digital pins
// motor one
#include <QTRSensors.h>
#include "arduinoFFT.h"

arduinoFFT FFT;
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A15
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

QTRSensors qtr;


//[-60,-52,-44,-36,-28,-20,-12,-4,4,12,20,28,36,44,52,60]
// [ -0.60, -0.52, -0.44, -0.36, -0.28, -0.20, -0.12, -0.04, 0.04, 0.12, 0.20, 0.28, 0.36, 0.44, 0.52, 0.60 ]
//-0.60, -0.52, -0.44, -0.40, -0.32, -0.24, -0.18, -0.04, 0.04, 0.18, 0.24, 0.32, 0.40, 0.44, 0.52, 0.60
const uint8_t SensorCount = 16;
float sensorW[16] = { -1.38, -1.12, -0.83, -0.68, -0.56, -0.44, -0.20, -0.08, 0.08, 0.20, 0.44, 0.56, 0.68, 0.83, 1.12, 1.38 };
uint16_t sensorValues[SensorCount];
double weightedVal[SensorCount];
double dVal[SensorCount];
double digital_thres = 500;

double position = 0;
double P, I, D, PID, PreErr = 0;
double offset = 3;

double motorSpeedA;
double motorSpeedB;
double baseSpeed = 80;
double Kp = 1.1;//1.1
double Kd = 1.9;

int enA = 12;//10
int in1 = 10;//8
int in2 = 11;//9
int M1 = 15;
// motor two
int enB = 7;//5
int in3 = 9;//6
int in4 = 8;//7
int M2 = 17;

bool linefollow=true;
bool music= false;
void setup()

{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
  // set all the motor control pins to outputs
  pinMode(M1, OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46 }, SensorCount);
  qtr.setEmitterPin(2);

  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  //`goForward(5000);
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
  //delay(1000); /* Repeat after delay */
}

void sensorRead() {
  double weightedSum = 0;
  double actualSum = 0;
  qtr.read(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i] = map(sensorValues[i], 0, 2500, 1000, 0);
    weightedVal[i] = sensorW[i] * sensorValues[i];
    actualSum += sensorValues[i];
    weightedSum += weightedVal[i];
    if(sensorValues[i] > digital_thres){
      dVal[i] = 1;
    }
    else {
      dVal[i] = 0;
    }
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println("");

  position = weightedSum / actualSum;
  //Serial.println(position*100);


  //delay(50);
}
void checkmusic(){
  if(heighestf>950){
    //music = true;
    linefollow=false;
  }else{
    //music=false;
    linefollow=true;
  }

}

void PID_control() {
  // Serial.println("Kp " + String(Kp,4) + "   KI " + String(Ki,4) + "   KD " + String(Kd,4) );
  //Serial.Println(positionLine);
  P = position * 100;
  D = P - PreErr;
  PID = Kp * P + Kd * D;

  PreErr = P;



  double MSpeedA = baseSpeed + offset - PID;  //
  double MSpeedB = baseSpeed - offset + PID;  //offset has been added to balance motor speeds


  // // constraints for speed

  if (MSpeedA > 140) {
    MSpeedA = 140;
  }

  if (MSpeedB > 140) {
    MSpeedB = 140;
  }

  if (MSpeedA < 40) {
    MSpeedA = 40;
  }

  if (MSpeedB < 30) {
    MSpeedB = 30;
  }

  //Serial.println("Position   " + String(position*100) + "  D  " + String(D) +  "  PID  "+ String(PID));
  // Serial.println("A   " + String(MSpeedA-offset)+ "   B   " +String(MSpeedB+offset));
  //enable pwm
  analogWrite(enA, MSpeedA);
  analogWrite(enB, MSpeedB);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(50);
}

void goForward(double forward_delay) {
  // this function will run the motors in
  //both directions at a fixed speed
  // turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 70 + offset);//70
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 70 - offset);//70
  delay(forward_delay);
  // now change motor directions
}

void TurnRight(double turn_delay){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 70 + offset);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 70 - offset);
  delay(turn_delay);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
}

void TurnLeft(double turn_delay){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 70 + offset);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 70 - offset);
  delay(turn_delay);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
}

void stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

void LineFollow(){
   sensorRead();
    if(dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1){
      goForward(200);
      //delay(500);
      sensorRead();
      
      
      if( dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1){
        //stop();
        //delay(4000);
       // WhiteBox = WhiteBox + 1;
        linefollow=false;
        stop();
        delay(4000);
       // break;
      }else{
        TurnLeft(500); // not PID control
      }}
    else if (dVal[0] == 1 && dVal[1] == 1 && dVal[2] == 1 && dVal[3] == 1 && dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1){ 
      goForward(400);  //Turn Right
      TurnRight(500);
    }

    else if (dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 && dVal[12] == 1&& dVal[13] == 1 && dVal[14] == 1 && dVal[15] == 1){
      goForward(400);  //Turn Left
      TurnLeft(500);

    }
    
    //else if(dVal[0] == 0 && dVal[1] == 0 && dVal[2] == 0 && dVal[3] == 0 && dVal[4] == 0 && dVal[5] == 0 && dVal[6] == 0 && dVal[7] == 0 && dVal[8] == 0 && dVal[9] == 0 && dVal[10] == 0 && dVal[11] == 0 && dVal[12] == 0 && dVal[13] == 0 && dVal[14] == 0 && dVal[15] == 0){
      //TurnRight(300);//400
      //sensorRead();
      //if(dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 ||dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1){
      //  PID_control();
        
      //}else{
      //TurnLeft(600);//8004
      //sensorRead();
      //if(dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 ||  dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1){
        //PID_control();
      //}}
    
    else{
      PID_control();
    }
    delay(20);
  }

void loop() {
  checkfft();
  checkmusic();
  if(linefollow){
    LineFollow();
  }else{
    stop();
    delay(100);
  }
    

    Serial.println("follow");
    //delay(2000);
  //}
}