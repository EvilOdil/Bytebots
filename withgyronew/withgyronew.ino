
// connect motor controller pins to Arduino digital pins
// motor one
#include <AccelStepper.h>
#include "Adafruit_VL53L0X.h"
#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>
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


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

QTRSensors qtr;


#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32



int sensor3, sensor2, sensor1,Sensor2;


// set the pins to shutdown
#define SHT_LOX1 52
#define SHT_LOX2 51
#define SHT_LOX3 53

#define NO_LWF_DIST 120 //15 distance threshold to follow left wall
#define NO_RWF_DIST 120 //15 distance threshold to follow left wall
#define LWF_LIMIT 200 // 4.1 // center line for left wall 5
#define RWF_LIMIT 200 // 11 // center line for right wall 5
// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
////Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();


// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;


#define motorPin1  48      // IN1
#define motorPin2  47      // IN2 //49
#define motorPin3  49     // IN3 //50
#define motorPin4  50    // IN4 //51
#define MotorInterfaceType 4
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);


int WhiteBox = 0;
int Start_1 = 0;
int Start_2 = 0;
int Start_3 = 0;
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;


double x;
double y;
double z;

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
double Kp = 1.1; //1.1 //5.8
double Kd = 1.4;


double wallPosition = 0;
float wallpositionLeft = 0;
float wallpositionRight = 0;
//float wallposition2 = 0;
float finalwallposition =0;
double wallP, wallI, wallD, wallPID, wallPreErr = 0;
double wallKp = 1.2;//1.1
double wallKd = 1.8;//2.8
double wallbaseSpeed = 60;


int wallcount=0;
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
bool wallfollow=false;
bool music= false;

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  //digitalWrite(SHT_LOX2, LOW);
  //digitalWrite(SHT_LOX3, LOW);

  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
 // digitalWrite(SHT_LOX2, HIGH);
  //digitalWrite(SHT_LOX3, HIGH);

  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  //digitalWrite(SHT_LOX2, LOW);
  //digitalWrite(SHT_LOX3, LOW);


  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  //digitalWrite(SHT_LOX2, HIGH);
  //delay(10);

  //initing LOX2
  /*
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX2
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1);
  }

*/

}
/*
int read_sensor2() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);

  // print sensor two reading
  if (measure2.RangeStatus != 4) {
    sensor2 = measure2.RangeMilliMeter;
    //Serial.print(sensor2);
    //Serial.print("mm");
  } else {
    sensor2=200;
  }
  
  return sensor2;
}

int read_sensor3() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  //lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);


  if (measure3.RangeStatus != 4) {
    sensor3 = measure3.RangeMilliMeter;
    //Serial.print(sensor3);
    //Serial.print("mm");
  } else {
    sensor3=200;
  }


  return sensor3;
}*/

int read_sensor1() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  //lox2.rangingTest(&measure2, false);
  //lox3.rangingTest(&measure3, false);


  if (measure1.RangeStatus != 4) {
    sensor1 = measure1.RangeMilliMeter;
    Serial.print(sensor1);
    //Serial.print("mm");
  } else {
    sensor1=200;
  }


  return sensor1;
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



void display_text(String text, int x, int y){
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(x, y);
  // Display static text
  display.println(text);
  display.display(); 
}

//void printWallSensors(){
  //int left = read_sensor2();
  //int right = read_sensor3();

  //display_text(String(left)+ " " + String(right), 0, 0);

  
//}

void lift(){
  stepper.setCurrentPosition(0);
    while (stepper.currentPosition() != -2000) { 
      stepper.setSpeed(-500);
      stepper.runSpeed();
  }

}

void drop(){
stepper.setCurrentPosition(0);
    while (stepper.currentPosition() != 3000) { 
      stepper.setSpeed(500);
      stepper.runSpeed();
  }

}


void setup()

{
  // set all the motor control pins to outputs
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  pinMode(M1, OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  stepper.setMaxSpeed(1000);
  stepper.setCurrentPosition(0); //set current pos to 0


  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46 }, SensorCount);
  qtr.setEmitterPin(2);

  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);
  Serial.begin(115200);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  //`goForward(5000);
  while (!Serial) {
    delay(1);
  }

  pinMode(SHT_LOX1, OUTPUT);
  //pinMode(SHT_LOX2, OUTPUT);
  //pinMode(SHT_LOX3, OUTPUT);

  digitalWrite(SHT_LOX1, LOW);
  //digitalWrite(SHT_LOX2, LOW);
  //digitalWrite(SHT_LOX3, LOW);

  //setID();
  display_text(String(WhiteBox), 0, 10);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

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

void Wall_Avoid(){
  int frontDist=read_sensor1();
      linefollow =true;
    wallfollow = false;
  if (frontDist<180){
    stop();
    delay(100);
    linefollow =false;
    wallfollow = true;
    wallcount=wallcount+1;
    if(wallcount==2){
      linefollow = false;
    //while (rightDist < RWF_LIMIT){
      wallfollow = true;
      TurnLeft(450);
    //  rightDist = read_sensor3();
    //}
    sensorRead();
    while (dVal[10] == 0 && dVal[11] == 0  && dVal[4] == 0 && dVal[5] == 0 && dVal[6] == 0 && dVal[7] == 0 && dVal[8] == 0 && dVal[9] == 0){
      goForward(100);
      sensorRead();
    }
    //delay(300);
    goForward(500);
    TurnLeft(400); 
    linefollow =true;
    wallfollow = false;

    }else if(wallcount==3){
       linefollow = false;
    //while (leftDist < LWF_LIMIT){
      wallfollow = true;
      TurnRight(450);
     // leftDist = read_sensor2();
   // }
   sensorRead();
    while ( dVal[4] == 0 && dVal[5] == 0 && dVal[6] == 0 && dVal[7] == 0 && dVal[8] == 0 && dVal[9] == 0 && dVal[10] == 0 && dVal[11] == 0 ){
      goForward(100);
      sensorRead();

      
    
    }
    stop();
    delay(400);
    linefollow = true;
    wallfollow = false;
    TurnRight(600);

    }
  }    
  
}
      
  

void Wall_PID_control() {




  wallP = position * 100;
  wallD = wallP - wallPreErr;
  wallPID = wallKp * wallP + wallKd * wallD;

  wallPreErr = wallP;



  double MSpeedA = wallbaseSpeed + offset - wallPID;  //
  double MSpeedB = wallbaseSpeed - offset + wallPID;  //offset has been added to balance motor speeds


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

void rush(double forward_delay) {
  // this function will run the motors in
  //both directions at a fixed speed
  // turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 180 + offset);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 180 - offset);
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
   // while (true) {

    sensorRead();
    if(dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1){
      goForward(200);
      //delay(500);
      sensorRead();
      
      
      if( dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1){
        //stop();
        //delay(4000);
        WhiteBox = WhiteBox + 1;
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
    
    else if(dVal[0] == 0 && dVal[1] == 0 && dVal[2] == 0 && dVal[3] == 0 && dVal[4] == 0 && dVal[5] == 0 && dVal[6] == 0 && dVal[7] == 0 && dVal[8] == 0 && dVal[9] == 0 && dVal[10] == 0 && dVal[11] == 0 && dVal[12] == 0 && dVal[13] == 0 && dVal[14] == 0 && dVal[15] == 0){
      TurnRight(300);//400
      sensorRead();
      if(dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 ||dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1){
        PID_control();
        
      }else{
      TurnLeft(600);//8004
      sensorRead();
      if(dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 ||  dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1){
        PID_control();
      }}
    
    }else{
      PID_control();
    }
    delay(20);
  }

//}


void Wall_LineFollow(){
   // while (true) {

    sensorRead();
    if(dVal[0] == 1 && dVal[1] == 1 && dVal[2] == 1 && dVal[3] == 1 && dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 && dVal[12] == 1 && dVal[13] == 1 && dVal[14] == 1 && dVal[15] == 1){
      goForward(200);
      delay(500);
      sensorRead();
      
      
      if(dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 ){
        //stop();
        //delay(4000);
        WhiteBox = WhiteBox + 1;
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
    
    else if(dVal[0] == 0 && dVal[1] == 0 && dVal[2] == 0 && dVal[3] == 0 && dVal[4] == 0 && dVal[5] == 0 && dVal[6] == 0 && dVal[7] == 0 && dVal[8] == 0 && dVal[9] == 0 && dVal[10] == 0 && dVal[11] == 0 && dVal[12] == 0 && dVal[13] == 0 && dVal[14] == 0 && dVal[15] == 0){
      TurnRight(300);//400
      sensorRead();
      if(dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 ||dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1){
        Wall_PID_control();
        
      }else{
      TurnLeft(600);//8004
      sensorRead();
      if(dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 ||  dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1){
        Wall_PID_control();
      }}
    
    }else{
      Wall_PID_control();
    }
    delay(20);
  }
 int Gyro(){
  Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);
 
x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

//display_text(String(y), 10, 0);
return y;
}




void loop() {
WhiteBox=7;
display_text(String(WhiteBox), 0, 10);
  if (WhiteBox == 0){
   if(Start_1 == 0){
    goForward(500);
    Start_1 = 1;
   }
   
   LineFollow();
  }
  

  else if(WhiteBox == 1){
    if(Start_2 == 0){
      goForward(600);
      linefollow=true;
      wallfollow=false;

      Start_2 =1;
    }

    if(wallcount<3){
  
    Wall_Avoid();
    Serial.print(linefollow);
    Serial.println(wallfollow);
    if(linefollow==true && wallfollow==false){
    Wall_LineFollow();
    }
    }
    else{
      LineFollow();
    }
  }

  else if(WhiteBox  == 2){
    if(Start_3 == 0){
      goForward(800);
      TurnRight(700);
      lift();
      //goForward(300);
      rush(5000);
      Start_3 == 1;
     
    }

    int angle = Gyro();
    if (angle > 10 && angle < 30 ){
    baseSpeed = 200;
    Kp = 5.8;
    Kd = 7.5;
    }

    else {
    baseSpeed = 80;
    Kp = 1.1;
    Kd = 1.4;
  
    }
    LineFollow();

  
    

  }else if(WhiteBox=7){
  checkfft();
  checkmusic();
  if(linefollow){
    LineFollow();
  }else{
    stop();
    delay(100);
  }
    

  }
}


  
  

