#include <Adafruit_TiCoServo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Adafruit_NeoPixel.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle nh;

#define MOTOR_L_pin 2
#define MOTOR_R_pin 3
#define eStopPin 7
#define headlightLeftPin 14
#define headlightRightPin 15
#define taillightLeftPin 8
#define taillightRightPin 9
#define IRPinLeft A1
#define IRPinRight A0

bool EStopped = false;
bool stopping = true;
bool IREStopped = false;

//Motor Variables
const int neutralAccel = 1470;
const int maxRange = 800;
float angularVel = 0;
float linearVel = 0;
int leftMotorValue = 0;
int rightMotorValue = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int prevTime = 0;
const int rampDelay = 1;
const int velScale = 150;
const int turnScale = 120;


//Light stuff
Adafruit_NeoPixel headlightLeft = Adafruit_NeoPixel(12, headlightLeftPin, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel headlightRight = Adafruit_NeoPixel(12, headlightRightPin, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel taillightLeft = Adafruit_NeoPixel(8, taillightLeftPin, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel taillightRight = Adafruit_NeoPixel(8, taillightRightPin, NEO_GRBW + NEO_KHZ800);

int brightness = 20;

uint32_t red = headlightLeft.Color(255, 0, 0);
uint32_t blue = headlightLeft.Color(0, 0, 255);
uint32_t green = headlightLeft.Color(0, 255, 0);
uint32_t white = headlightLeft.Color(255, 255, 255);
uint32_t yellow = headlightLeft.Color(255, 100, 0);
uint32_t off = headlightLeft.Color(0, 0, 0);
unsigned long prevTimeLightsBlink = 0;
unsigned long prevTimeLightsRipple = 0;
unsigned long currentMillis = 0;
unsigned long turnLightsMillis = 0;


//IR STUFF
unsigned long IRLeftMillis = 0;
unsigned long IRRightMillis = 0;
int IRDelay = 25;
int rippleIndex = 0;


Adafruit_TiCoServo MOTOR_L;  
Adafruit_TiCoServo MOTOR_R;


void CMD_VEL(const geometry_msgs::Twist& cmd_msg){
   angularVel = cmd_msg.angular.z;
   linearVel = cmd_msg.linear.x;   
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", CMD_VEL);
std_msgs::Float32 int_msg;
ros::Publisher chatter("chatter", &int_msg);

void setup() {
  IREStopped = false;
  Serial.begin(9600);
  pinMode(MOTOR_L_pin, OUTPUT);
  pinMode(MOTOR_R_pin, OUTPUT);
  pinMode(eStopPin, INPUT);
  MOTOR_L.attach(MOTOR_L_pin, neutralAccel - maxRange/2, neutralAccel + maxRange/2);
  MOTOR_R.attach(MOTOR_R_pin, neutralAccel - maxRange/2, neutralAccel + maxRange/2);
  
  headlightLeft.begin();
  headlightRight.begin();
  taillightLeft.begin();
  taillightRight.begin();
  headlightLeft.show();
  showLights();
  headlightLeft.setBrightness(brightness);
  headlightRight.setBrightness(brightness);
  taillightLeft.setBrightness(brightness);
  taillightRight.setBrightness(brightness);
  setSolid(headlightLeft, 12, blue);
  setSolid(headlightRight, 12, blue);
  MOTOR_L.write(90); 
  MOTOR_R.write(90);
  
  nh.advertise(chatter);
  nh.subscribe(sub);
  nh.initNode();
  
  //Serial.println("beginning");
  delay(5);
  currentMillis = millis();
  prevTime = millis();
  prevTimeLightsBlink = millis();
  prevTimeLightsRipple = millis();
}

void loop() {
  
  
//Check if the Estop is pressed
  if (IREStopped){ //Should change this to WHILE loop and assign STOP to R and L motors
      //EStopped = true;
      //MODE = 0;
      //Serial.println("ESTOPPED");
      MOTOR_L.write(mapMotor(0));
      MOTOR_R.write(mapMotor(0));
      //MOTOR_L.writeMicroseconds(STOP);
      //MOTOR_R.writeMicroseconds(STOP);    
   }else{
      EStopped = false;
      //calculateMotorSpeed(.8, 0);
      calculateMotorSpeed(linearVel, angularVel);
      rampSpeed(leftMotorSpeed, rightMotorSpeed);
      //MOTOR_L.write(mapMotor(leftMotorValue));
      //MOTOR_R.write(mapMotor(rightMotorValue));
      //Serial.println(leftMotorValue);
      //Serial.println(rightMotorValue);
   }


   if( currentMillis - turnLightsMillis > 100){
    //runLights();
    turnLightsMillis = currentMillis;
   }
   int_msg.data = linearVel;
   chatter.publish(&int_msg);
   //Serial.println(analogRead(IRPinLeft));

   nh.spinOnce();
   delay(1);
   SenseIR();
   currentMillis = millis();
   
}


void SenseIR(){
  int IRLeftDistance = analogRead(IRPinLeft);
  int IRRightDistance = analogRead(IRPinRight);
  //Serial.println(IRLeftDistance);
  //Serial.println(IRRightDistance);
  if (IRLeftDistance < 50){
    //Serial.println("test");
    if(currentMillis - IRLeftMillis > IRDelay){
      //Serial.println("test");
      IREStopped = true;
    }
  }else{
    IRLeftMillis = currentMillis;
    IREStopped = false;
  }
  if (IRRightDistance < 50){
    if(currentMillis - IRRightMillis > IRDelay){
      IREStopped = true;
    }
  }else{
    IRRightMillis = currentMillis;
    IREStopped = false;
  }
}
void runLights(){
  if(!EStopped){
    //headlightLeft.setPixelColor(5, blue);
    setSolid(taillightLeft, 8, red);
    setSolid(taillightRight, 8, red);
    
    if(angularVel < -.2){
      setSolid(taillightRight, 8, yellow);
    }else if(angularVel > .2){
      setSolid(taillightLeft, 8, yellow);
    }
  }else{
      setBlinking(taillightLeft, 8, red, 300);
      setBlinking(taillightRight, 8, red, 300);
  }
  //headlightLeft.show();
  //headlightRight.show();
  //taillightLeft.show();
  //taillightRight.show();
  showLights();
}


boolean readEstop() {
  boolean eStopTriggered = digitalRead(eStopPin);
  return eStopTriggered;
}

void calculateMotorSpeed(float linear, float angular){
  //calculates motor speed from linear and angular
  leftMotorSpeed = linear*velScale - angular*turnScale;
  rightMotorSpeed = linear*velScale + angular*turnScale;
}


void setSolid(Adafruit_NeoPixel &light, int numPins, uint32_t color){
  for(int i = 0; i<numPins; i++){
    light.setPixelColor(i, color);
  }
}
void setBlinking(Adafruit_NeoPixel &light, int numPins, uint32_t color, unsigned long delayTime){ 
  if((currentMillis - prevTimeLightsBlink) < delayTime/2){
    setSolid(light, numPins, color);
  }else if((currentMillis - prevTimeLightsBlink) < delayTime){
    setSolid(light, numPins, off);
  }else{
    prevTimeLightsBlink = currentMillis;
  }
}
void setRipple(Adafruit_NeoPixel &light, int numPins, uint32_t color){
  if(rippleIndex == -1){
    for(int i= 0; i < 12; i++){
       light.setPixelColor(i, off);
    }
  }else{
    light.setPixelColor(11 - rippleIndex, color);
  }
}
void updateRippleIndex(unsigned long delayTime){
  if(currentMillis - prevTimeLightsRipple > delayTime/13){
    rippleIndex ++;
    if(rippleIndex > 11){
      rippleIndex = -1;
    }
    prevTimeLightsRipple = currentMillis;
  }
}
  
void rampSpeed(int speedLeft, int speedRight)
{
  stopping = true;
  if(currentMillis - prevTime > rampDelay){
    if (speedLeft > leftMotorValue){
      leftMotorValue ++;
      stopping = false;
    }else if (speedLeft < leftMotorValue){
      leftMotorValue --;
    }
    if (speedRight > rightMotorValue){
      rightMotorValue ++;
      stopping = false;
    }else if (speedRight < rightMotorValue){
      rightMotorValue --;
    }
    prevTime = currentMillis;
  }
  MOTOR_L.write(mapMotor(leftMotorValue));
  MOTOR_R.write(mapMotor(rightMotorValue));
  
}
int mapMotor(float input){
  return map(input,-(turnScale + velScale), turnScale + velScale, 0, 180);
}

void showLights(){
  headlightLeft.show();
  headlightRight.show();
  taillightLeft.show();
  taillightRight.show();
}

