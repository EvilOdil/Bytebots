
#include "Adafruit_VL53L0X.h"
#include <QTRSensors.h>
QTRSensors qtr;

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

int sensor3, sensor2, Sensor3,Sensor2,sensor1, Sensor1;


// set the pins to shutdown
#define SHT_LOX1 52
#define SHT_LOX2 51
#define SHT_LOX3 53


// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();


// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

//[-60,-52,-44,-36,-28,-20,-12,-4,4,12,20,28,36,44,52,60]
// [ -0.60, -0.52, -0.44, -0.36, -0.28, -0.20, -0.12, -0.04, 0.04, 0.12, 0.20, 0.28, 0.36, 0.44, 0.52, 0.60 ]
//-0.60, -0.52, -0.44, -0.40, -0.32, -0.24, -0.18, -0.04, 0.04, 0.18, 0.24, 0.32, 0.40, 0.44, 0.52, 0.60
const uint8_t SensorCount = 16;
float sensorW[16] = { -1.38, -1.12, -0.83, -0.68, -0.56, -0.44, -0.28, -0.12, 0.12, 0.28, 0.44, 0.56, 0.68, 0.83, 1.12, 1.38 };
uint16_t sensorValues[SensorCount];
double weightedVal[SensorCount];
double dVal[SensorCount];
double digital_thres = 500;

double position = 0;
float wallpositionLeft = 0;
float wallpositionRight = 0;
//float wallposition2 = 0;
float finalwallposition =0;
double P, I, D, PID, PreErr = 0;
double offset = 3;

double motorSpeedA;
double motorSpeedB;
double baseSpeed = 70;
double Kp = 1.1; //1.1
double Kd = 2.8;

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



/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);

  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);


  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
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



}

int read_sensor1() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);

  // print sensor two reading
  if (measure1.RangeStatus != 4) {
    sensor1 = measure1.RangeMilliMeter;
    //Serial.print(sensor2);
    //Serial.print("mm");
  } else {
    sensor1=200;
  }
  
  return sensor1;
}

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
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);


  if (measure3.RangeStatus != 4) {
    sensor3 = measure3.RangeMilliMeter;
    //Serial.print(sensor3);
    //Serial.print("mm");
  } else {
    sensor3=200;
  }


  return sensor3;
}
void setup() {
  

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
  Serial.begin(115200);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

   while (!Serial) {
    delay(1);
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  setID();
}

int wallsensorRead() {
  // Adjust these values based on your setup and requirements
  int wallDistance = 200; // Distance to the wall for smooth following
    Sensor3=read_sensor3();
    Sensor2=read_sensor2();
    wallpositionLeft = -(wallDistance-Sensor2);
    wallpositionRight = (wallDistance-Sensor3);
    //else {
      //wallposition=0;
    //}

   // Convert to meters
    if (Sensor2<wallDistance || Sensor3<wallDistance){
      linefollow=false;
      wallfollow=true;
    }else{
      linefollow=true;
      wallfollow=false;
    }
      // Adjust this value as needed
    


  
  //delay(50);
  //Serial.print(Sensor2);
  //Serial.print(" ");
  //Serial.println(Sensor3);
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
    //Serial.print(sensorValues[i]);
    //Serial.print('\t');
  }
  //Serial.println("");

  position = weightedSum / actualSum;
  ////Serial.println(position*100);


  //delay(50);
}


void PID_control() {
  // //Serial.println("Kp " + String(Kp,4) + "   KI " + String(Ki,4) + "   KD " + String(Kd,4) );
  ////Serial.println(positionLine);
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

  ////Serial.println("Position   " + String(position*100) + "  D  " + String(D) +  "  PID  "+ String(PID));
  // //Serial.println("A   " + String(MSpeedA-offset)+ "   B   " +String(MSpeedB+offset));
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
  analogWrite(enA, 70 + offset);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 70 - offset);
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


void GuardLidar(){
  Serial.print('guard bot lidar started');
  bool guardFront; //is guard bot in the front
  read_sensor1();
  read_sensor2();
  read_sensor3();
  while(true){
    read_sensor1();
  read_sensor2();
  read_sensor3();
    if( sensor1<50 && sensor2<50 && sensor3<50){
      delay(50);
    }
    else if(sensor1<50 && sensor2<50 && sensor3>50){
      Serial.println('going left'); // free to go
      break;
    }
    else if(sensor1<50 && sensor2>50 && sensor3<50){
      Serial.println('going right');
      delay(10000); // wait till guard make next rotation
    }
    else{
      delay(50);
    }

  }
  
Serial.println('Exit from guard bot detection - Lidar');
}



void loop() {
  Serial.println('start');
  //GuardLidar();
  
}
