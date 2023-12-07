#include <Adafruit_TCS34725.h>
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

uint16_t r1, g1, b1, c1; // bottom

String colorSenseFront(){

  Serial.println("Colors sensor bottom");

  int rc = 0;
  int gc = 0;
  int bc = 0;
  int wc = 0;
   digitalWrite(30, HIGH);


for(int i =0; i<8;i++){

  tcs.getRawData(&r1, &g1, &b1, &c1);

  
  // convert raw data to RGB values
  float red1 = r1 / 256.0;
  float green1 = g1 / 256.0;
  float blue1 = b1 / 256.0;


  // determine the color based on RGB values
  if (red1 > green1 && red1 > blue1) {
    //Serial.println("Color bottom: Red");
     rc = rc + 1;
  } else if (green1 > red1 && green1 > blue1) {
    //Serial.println("Color bottom: Green");
     gc = gc + 1;
  } else if (blue1 > red1 && blue1 > green1) {
    //Serial.println("Color bottom: Blue");
     bc = bc + 1;
  } else if (red1 == green1 && green1 == blue1) {
    //Serial.println("Color bottom: White");
     wc = wc + 1;
  } else {
      Serial.println("Color bottom: Unknown");
  }
/*
  Serial.print("R1: ");
  Serial.print(red1);
  Serial.print(" G1: ");
  Serial.print(green1);
  Serial.print(" B1: ");
  Serial.println(blue1);
*/
  delay(100); // chnge this delay
  }
  digitalWrite(30,LOW);

if(rc > 3){
  return "RED";
  
}

else if(gc > 3){
  return "GREEN";
}


else if(bc > 3 ){
  return "BLUE";
  
}
else if(wc > 3){
  return "WHITE";
}

else {
  return "UNKNOWN";
}


}



void setup() {
  // put your setup code here, to run once:
  
  Wire.begin();
  Serial.begin(9600);
  pinMode(30, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  digitalWrite(30, LOW);
  digitalWrite(22, LOW);
  digitalWrite(23, LOW);
  delay(2000);

}

void loop() {
  // put your main code here, to run repeatedly:
   String front_color = colorSenseFront();
  Serial.println(front_color );
  if (front_color == "BLUE"){
    digitalWrite(23, HIGH);
    digitalWrite(22, LOW);

  }
  else if (front_color == "RED"){
    digitalWrite(22, HIGH);
    digitalWrite(23, LOW);

  }
  else{
    digitalWrite(22, LOW);
    digitalWrite(23, LOW);
  }
  


}