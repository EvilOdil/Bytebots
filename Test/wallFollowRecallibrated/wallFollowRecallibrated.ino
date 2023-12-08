// connect motor controller pins to Arduino digital pins
// motor one
#include <QTRSensors.h>
QTRSensors qtr;

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
double P, I, D, PID, PreErr = 0;
double offset = 3;

double motorSpeedA;
double motorSpeedB;
double baseSpeed = 60;
double Kp = 0.7; //1.1
double Kd = 1.1;

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
bool wallFollow=false;
bool lineFollow=true;

// connect motor controller pins to Arduino digital pins
// motor one
#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

int sensor1, sensor2, sensor3;


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
    while (1)
      ;
  }



}

void read_dual_sensors() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);

  // print sensor one reading
  Serial.print("1: ");
  if (measure1.RangeStatus != 4) {  // if not out of range
    sensor1 = measure1.RangeMilliMeter;
    Serial.print(sensor1);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }

  Serial.print(" ");

  // print sensor two reading
  Serial.print("2: ");
  if (measure2.RangeStatus != 4) {
    sensor2 = measure2.RangeMilliMeter;
    Serial.print(sensor2);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }

  Serial.print(" ");

  // print sensor two reading
  Serial.print("3: ");
  if (measure3.RangeStatus != 4) {
    sensor3 = measure3.RangeMilliMeter;
    Serial.print(sensor3);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }

  Serial.print(" ");



  Serial.print(" ");



  Serial.println();
}



void setup()

{

  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);


  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  

  Serial.println("Both in reset mode...(pins are low)");


  Serial.println("Starting...");
  setID();

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
  Serial.begin(9600);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
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


  delay(50);
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


void wall(){
  read_dual_sensors();
  if(sensor2<200){
      TurnRight(200);
      goForward(200);
      
    }
    else if(sensor3<200){
      TurnLeft(200);
      goForward(200);
    }
}

void loop() {

  sensorRead();
  read_dual_sensors();

  if (sensor2<150 || sensor3<150){ 
    lineFollow=false;
    wallFollow = true;
  }
  else{

    lineFollow=true;
    wallFollow = false;
  }
  if(lineFollow){
  
    sensorRead();
    

    if(dVal[0] == 1 && dVal[1] == 1 && dVal[2] == 1 && dVal[3] == 1 && dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 && dVal[12] == 1 && dVal[13] == 1 && dVal[14] == 1 && dVal[15] == 1){
      goForward(200);
      delay(500);
      sensorRead();
      
      if(dVal[0] == 1 && dVal[1] == 1 && dVal[2] == 1 && dVal[3] == 1 && dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 && dVal[12] == 1 && dVal[13] == 1 && dVal[14] == 1 && dVal[15] == 1){
        stop();
        delay(4000);
        linefollow=false;
        //break;
      }else{
        TurnLeft(500); // not PID control
      }}
    else if (dVal[0] == 1 && dVal[1] == 1 && dVal[2] == 1 && dVal[3] == 1 && dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1){ 
      goForward(200);  //Turn Right
      TurnRight(500);
    }

    else if (dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 && dVal[12] == 1&& dVal[13] == 1 && dVal[14] == 1 && dVal[15] == 1){
      goForward(200);  //Turn Left
      TurnLeft(500);

    }
    
    else if(dVal[0] == 0 && dVal[1] == 0 && dVal[2] == 0 && dVal[3] == 0 && dVal[4] == 0 && dVal[5] == 0 && dVal[6] == 0 && dVal[7] == 0 && dVal[8] == 0 && dVal[9] == 0 && dVal[10] == 0 && dVal[11] == 0 && dVal[12] == 0 && dVal[13] == 0 && dVal[14] == 0 && dVal[15] == 0){
      TurnRight(400);//400
      if(dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 ||dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1){
        PID_control;
      }else{
      TurnLeft(800);//800
      if(dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 ||  dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1){
        PID_control;
      }}
    } 
  
    
    else{
      PID_control();
    }
    delay(20);
  }
  

else if(wallFollow){
  wall();
}
else{
  stop();
}
}