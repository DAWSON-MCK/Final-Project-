#include "Encoder.h"
#include <Wire.h>

// Here we define our pins and constants
// NEW IDEA if we are within the tolerance we will add the error to one of the wheels in order to 
#define M1SPEED       9 // used to modify the motor 1 speed RIGHT WHEEL
#define M2SPEED       10 // used to modify the motoe 2 speed LEFT WHEEL
#define M1DIR         7 // used to modify motor 1 direction 
#define M2DIR         8 // used to modify motor 2 direction 
#define CPR           800 // counts per revolution 
#define SLAVE_ADDRESS 0x04// dawson info 

#define POSTOLERANCE  3
#define NEGTOLERANCE  252

Encoder M1Encoder(2,5); // motor 1 encoder pins (RIGHT WHEEL)
Encoder M2Encoder(3,6); // motor 2 encoder pins (LEFT WHEEL)

// Here we have the variables in charge of recording old and new encoder values  
double newPosition1 = 0;
double newPosition2 = 0;

double errorAngle = 0; // used to calculate difference in target and current angle

// angle is desiredInputs[1], distance flag is desiredInputs[2]
byte desiredInputs[32] = {0}; // the list that will read in the values from the camera 

// here we set the variable to take care of ft to counts 
int desiredAngle;
double rectifiedAngle;

// flags
int state = 0; // used for the case statements 
int flagAngle; // used within the angle control 
int loopCounts = 0; // used within the angle control 
int tipFlag = 0;

int stopFlag = 0;


void setup() {
  pinMode(4, OUTPUT); // here we set the tri-state
  digitalWrite(4, HIGH);
  
  pinMode(M1DIR, OUTPUT); // RIGHT WHEEL 
  pinMode(M2DIR, OUTPUT); // LEFT WHEEL 
  
  pinMode(M1SPEED, OUTPUT); // motor 1 speed 
  pinMode(M2SPEED, OUTPUT); // motor 2 speed
  
  pinMode(12, INPUT); // status flag indicator 

  Serial.begin(9600); // baud rate for the serial monitor

  Wire.begin(SLAVE_ADDRESS); // initialize I2C as the subordinate

  // callbacks for I2C
  Wire.onRequest(sendData);
  Wire.onReceive(receiveData);
  Serial.println("Final Demo");
  Serial.println("Ready!");
  
}


// here we switch between the different states/functions 
void loop() {
  newPosition1 = abs(M1Encoder.read()/4); // scale down the values so calculations are easier on the Arduino 
  newPosition2 = abs(M2Encoder.read()/4);
  

// print statements for testing 
  //Serial.print("Angle:"); // print the angle flad from the PI 
  //Serial.println(desiredInputs[1]);
  //Serial.print("Distance:"); // print the distance flag from PI
  //Serial.println(desiredInputs[2]);

  switch (state){
    
    case 0: // LOCALIZE
    
    while((desiredInputs[1] > 26) && (desiredInputs[1] < 28)) { // rotate until the camera sees the tape 
      digitalWrite(M1DIR,LOW); 
      digitalWrite(M2DIR,LOW);
      analogWrite(M2SPEED, 32); 
      analogWrite(M1SPEED, 32);
      // we are going to slowly rotate until we have a value other than 27
    }
    
    if((desiredInputs[1] <= 26 || desiredInputs[1] >= 228) && (desiredInputs[1] != 0)) { // if we have a nonzero value that is less than 27 or greater than 228, we go to angle control 
      state = 1;
    }
    break;
    
    case 1: // CENTER
    // in this case we will make sure to center our robot with the tape 
    if((desiredInputs[1] <= 26) && (flagAngle == 0) && (desiredInputs[1] != 0)) {
      desiredAngle = desiredInputs[1]; // here we have the positive angle case 
      rectifiedAngle = 3.55*desiredAngle; // here we calculate the number of counts per degree and multiply by degrees (full for both wheels)
      angleControl(rectifiedAngle);
    }
    
    else if ((desiredInputs[1] >= 229) && (flagAngle == 0) && (desiredInputs[1] != 0)) {
      desiredAngle = desiredInputs[1] - 255; // here we have the negative angle case 
      rectifiedAngle = 3.55*desiredAngle; // here we calculate the number of counts per degree and multiply by degrees (full for both wheels)
      angleControl(rectifiedAngle);
    }
    
    break;
    
    case 2: // HIT  
    loopCounts = 0;
    flagAngle = 0;
    while((desiredInputs[2] == 0) && ((desiredInputs[1] < POSTOLERANCE) || (desiredInputs[1] > NEGTOLERANCE))){
      if(desiredInputs[1] < POSTOLERANCE) {
        digitalWrite(M1DIR,HIGH); 
        digitalWrite(M2DIR, LOW);

        analogWrite(M1SPEED, 64 + desiredInputs[1]);
        analogWrite(M2SPEED, 64); 
      }
      if (desiredInputs[1] > NEGTOLERANCE) {
        digitalWrite(M1DIR,HIGH); 
        digitalWrite(M2DIR, LOW);

        analogWrite(M1SPEED, 64);
        analogWrite(M2SPEED, 64 +(255 - desiredInputs[1]));
      }
    }

// we send it back to the angle control if we are out of our tolerance 
    if(desiredInputs[1] > POSTOLERANCE){
      state = 0;
    }
    else if (desiredInputs[1] < NEGTOLERANCE) {
      state = 0;
    }
    
    if(desiredInputs[2]== 1){ // if we hit our flag then stop the velocity entirely 
      analogWrite(M1SPEED, 0); // WRITES THE SPEED OF THE MOTOR 
      analogWrite(M2SPEED, 0); // set M2 speed to 0.5 * max speed
      tipFlag = 1;
      state = 0; // state is nonexistent so it should stay still  
      //throw the distance control in here to compensate the shortcoming 
    }
    break;

    case 3: // ALIGN
    // here we hardcode an angle to make sure we are parallel to the tape

    desiredAngle = 50; // here we hardcode the angle to get parallel to the tape  
    rectifiedAngle = 3.55*desiredAngle; // here we calculate the number of counts per degree and multiply by degrees (full for both wheels)
    angleControl(rectifiedAngle); 
    state = 0; // move to tracing the tape  
    break;

    case 4: // CURVE
    // we will go straight and adjust the angle as we traverse 
    loopCounts = 0;
    flagAngle = 0;
    while((desiredInputs[1] < POSTOLERANCE) || (desiredInputs[1] > NEGTOLERANCE)){
      digitalWrite(M1DIR,HIGH); 
      digitalWrite(M2DIR, LOW);
      analogWrite(M1SPEED, 64);
      analogWrite(M2SPEED, 64); 
    }

// we send it back to the angle control if we are out of our tolerance 
    if(desiredInputs[1] > POSTOLERANCE){
      state = 5;
    }
    else if (desiredInputs[1] < NEGTOLERANCE) {
      state = 5;
    }
    break;

    case 5:
    
    while((desiredInputs[1] > 26) && (desiredInputs[1] < 28) && (stopFlag == 0)) { // rotate until the camera sees the tape 
      stopFlag = 1;
      digitalWrite(M1DIR,LOW); 
      digitalWrite(M2DIR,LOW);
      analogWrite(M2SPEED, 32); 
      analogWrite(M1SPEED, 32);
      // we are going to slowly rotate until we have a value other than 27
    }
    if(stopFlag == 1) {
      digitalWrite(M1DIR,LOW); 
      digitalWrite(M2DIR,LOW);
      analogWrite(M2SPEED, 0); 
      analogWrite(M1SPEED, 0);
      state = 6;
    }
    
    if(desiredInputs[1] <= 26 || desiredInputs[1] >= 228) { // if we have a nonzero value that is less than 27 we have found our tape 
      state = 1;
    }
    break;
  }
}

// here we have the function which takes care of rotating a specified angle 
void angleControl(int targetAngle){
  // this keeps track of the direction of rotation 
  
  if (targetAngle > 0){ // turn CCW
    digitalWrite(M1DIR,HIGH); 
    digitalWrite(M2DIR,HIGH); 

    errorAngle = (targetAngle - newPosition1); // calculate the difference in our desired angle and current angle 

    if (errorAngle < 2){
      flagAngle = 1;
    }
    
    if (flagAngle == 1) {
      analogWrite(M2SPEED,0);
      analogWrite(M1SPEED, 0);

      if (tipFlag == 0) {
        state = 2;
      }
      else {
        state = 4;
      }      
      if (loopCounts == 0){
        delay(1000); // meant to delay after the angle is done so that we can take care of the angle nicely 
        M1Encoder.write(0);
        M2Encoder.write(0);
      }
      loopCounts = loopCounts + 1; // maybe change this back to normal 
    }
    else{
      analogWrite(M2SPEED, 32);
      analogWrite(M1SPEED, 32); 
    }
  }
  else{ // with a negative angle we will turn CW
    digitalWrite(M1DIR,LOW); 
    digitalWrite(M2DIR,LOW); 
    
    errorAngle = (abs(targetAngle) - newPosition2); // calculate the difference in our desired angle and current angle 

    if (errorAngle < 2){
      flagAngle = 1;
    }
    
    if (flagAngle == 1) {
      analogWrite(M1SPEED, 0);
      analogWrite(M2SPEED,0);

      if(tipFlag == 0){
        state = 2;
      }
      else {
        state = 4;
      }
            
      if (loopCounts == 0){
        delay(1000);
        M1Encoder.write(0);
        M2Encoder.write(0);
      }
      loopCounts = loopCounts + 1;
    }
    else{
      analogWrite(M1SPEED, 32);
      analogWrite(M2SPEED, 32);
    }
  }
}

void receiveData(int byteCount){
  int i = 0;
  while(Wire.available()) {
    desiredInputs[i] = Wire.read();
    i++;
  }
  i--;
}

void sendData(){
  Wire.write(desiredInputs[1]);
}
