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

const int trigPinL = 25;  // Trigger pin of the ultrasonic sensor
const int echoPinL = 26; // Echo pin of the ultrasonic sensor

const int trigPinR = 27;  // Trigger pin of the ultrasonic sensor
const int echoPinR = 28; // Echo pin of the ultrasonic sensor



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

void GuardLidar(){
  bool guardFront; //is guard bot in the front
  read_dual_sensors();
  if( sensor1<800 || sensor2<800 || sensor3<800){
    guardFront = true;
  }
  else{
    guardFront = false;
  }
Serial.print('is guard infront');
Serial.print(guardFront);  
}


int GuardUltra() {
  int guardSide;
  // Generate a pulse to trigger the ultrasonic sensor
  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinR, LOW);

  // Measure the duration of the echo pulse
  long durationL = pulseIn(echoPinL, HIGH);
  long durationR = pulseIn(echoPinR, HIGH);

  // Calculate the distance in centimeters
  float distanceL = durationL * 0.034 / 2;
  float distanceR = durationR * 0.034 / 2;

  delay(200); // wait for second reading

  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinR, LOW);

  // Measure the duration of the echo pulse
  durationL = pulseIn(echoPinL, HIGH);
  durationR = pulseIn(echoPinR, HIGH);

  float distanceL1 = durationL * 0.034 / 2;
  float distanceR1 = durationR * 0.034 / 2;


  if(distanceL<80){
    if(distanceL - distanceL1 < 0){
      guardSide =1;
      Serial.print('guard on left and leaving');
    }
    else{
      guardSide =2;
      Serial.print('guard on left and approaching');
    }
    
  }
  else if(distanceR<80){
    if(distanceR - distanceR1 < 0){
      guardSide =3;
      Serial.print('guard on right and leaving');
    }
    else{
      guardSide =4;
      Serial.print('guard on right and approaching');
    }
  }
else{
  Serial.print('error');
}
  return guardSide; // left and leaving = 1 , left and approaching =2,  right and leaving = 3 , right and approaching =4
}


void setup() {
  Serial.begin(115200);

  pinMode(trigPinL, OUTPUT); // ultrasonic
  pinMode(echoPinL, INPUT);
   pinMode(trigPinR, OUTPUT); // ultrasonic
  pinMode(echoPinR, INPUT);

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
}

void loop() {

  read_dual_sensors();
  delay(10);
}