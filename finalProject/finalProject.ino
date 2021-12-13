
#include <HCSR04.h>
#include <SharpIR.h>

#include "RobotKit.h"
#include <Servo.h>

// object declarations
// object of ultrasonic library
HCSR04 sensor1(2, 3); // sensor1 for object detection

// object of IR library
SharpIR sensor2(4, 5); // sensor2 for object detection

//object of ultrasonic library
HCSR04 safetySense(8, 9); // safetySense for making sure any persons are out of the way of the arm before movement

// robot object
KitBot robot;

// global var declarations
float value1 = -1; // var to store distance value from sensor1

float value2 = -1; // var to store distance value from sensor2

float safetyValue = -1; // var to store distance value from saftey sensor

float mean[12]; // array to store values from distance detection sensors

float count = 2; // counter variable to increase each time an object is stacked so that the next object will stack on top of last

float ultraAvg = 0; // var to store average measurement from ultrasonic distance detection sensor

float IRavg = 0; // var to store average measurement from IR distance detection sensor

float avgdist = 0; // var to store average measurement across both sensors

int onOff = 6; // var to store input from on/off switch

int onOffswitch = 7; // var to store output from on/off switch

// reads ultrasonic distance detection sensor
float readSensor1(){
  value1 = sensor1.dist();
  
  return value1;
}
// reads IR distance detection sensor
float readSensor2(){
  value2 = sensor2.getDistance();
  
  return value2;
}
// reads ultrasonic human safety sensor
float readSafety(){
  safetyValue = safetySense.dist();

  return safetyValue;
}
// robot setup 
void setup() {
  Serial.begin(9600);

  // setup pins for sensors
  pinMode(A0, INPUT);

  // define pin for on/off switch
  pinMode(onOff, INPUT_PULLUP); //using arduino built in pull up resistor on pin 6
  pinMode(onOffswitch, OUTPUT); // output pin set to pin 7

  // setup robot
  robot.enableRobot();
  robot.updateServos(robot.get_theta1(),robot.get_theta2(), robot.get_theta3(), robot.get_theta4(), robot.get_theta5(), robot.get_theta6());
}
// empties array after every use for average distance calculations
void empty(){
  for(int i = 0; i < 12; i++){
    mean[i] = 0;
  }
}

// calculates average of 10 measurements from ultrasonic sensor
float ultraMeasurementMean(){
 
  int count = 0;
  
  float sum = 0;
  
  while(count <10){
   
    readSensor1();
    
    mean[count] = readSensor1();

    count++;
  }

  for(int i =0; i < 10; i++){
    sum += mean[i]; 
  }
  
  ultraAvg = sum /10;

  empty();

  return ultraAvg;
}

// calculates average of 10 measurements from IR sensor
float IRmeasurementMean(){
 
  int count = 0;
  
  float sum = 0;
  
  while(count <10){
   
    readSensor2();
    
    mean[count] = readSensor2();

    count++;
  }

  for(int i =0; i < 10; i++){
    sum += mean[i]; 
  }
  
  IRavg = sum /10;

  empty();

  return IRavg;
}

// calculates average between the two sensor measurements 
float sensorAverageDist(){
  avgdist = (ultraMeasurementMean() + IRmeasurementMean())/2;
  
  return avgdist;
}

// checks that object detected by sensors is within the range that the robot can pick it up
boolean gripAble(){
  if(sensorAverageDist() < 2 || sensorAverageDist() > 15){
    
    // home position
    robot.move_xyzRPY_fast(SHOULDER_OFFSET, 0, HEIGHT_2_SECOND_MOTOR + LENGTH_ARM_2 + CLAW_LENGTH, 
    0, 90, 45);
    robot.delay_after_finish(5000);

    return false;
  
  }else{
    
    return true;
  }
}

//checks that there is no persons in range of danger when the robot is going to move
boolean safe(){
  if(readSafety() < 0 || readSafety() > 20){
    return true;
  }else{
    return false;
  }
}

void movementSequence(){
  // movement commands to pick up object and move it to specified stacking position
  // move to position of object with gripper open wide
  robot.move_xyzRPY_fast(FORWARD_COMMAND_WAIST, avgdist, 2, 0, 0, 95);
  robot.delay_after_finish(5000);

  //close gripper on object
  robot.move_xyzRPY_fast(FORWARD_COMMAND_WAIST, avgdist, 2, 0, 0, 35);
  robot.delay_after_finish(5000);

  //move halfway between object position and stacking position with some roll and pitch through the movement
  robot.move_xyzRPY_fast(130, 65, HEIGHT_2_SECOND_MOTOR + LENGTH_ARM_2, 100, 60, 35);
  robot.delay_after_finish(5000);
  
  //move with object in gripper to stacking position defined as LEFT_COMMAND_WAIST
  robot.move_xyzRPY_fast(LEFT_COMMAND_WAIST, 0, count, 0, 0, 35);
  robot.delay_after_finish(5000);

  // open gripper to drop object at height of count
  robot.move_xyzRPY_fast(LEFT_COMMAND_WAIST, 0, count, 0, 0, 95); 
  robot.delay_after_finish(5000);
  count += 4; // add height of object to count for next sequence to drop object on top of object already placed

  //move halfway between stacking position and home position
  robot.move_xyzRPY_fast(120, 25, HEIGHT_2_SECOND_MOTOR + LENGTH_ARM_2, 90, 20, 75);
  robot.delay_after_finish(5000);
  
  // return to home position
  robot.move_xyzRPY_fast(SHOULDER_OFFSET, 0, HEIGHT_2_SECOND_MOTOR + LENGTH_ARM_2 + CLAW_LENGTH, 
    0, 90, 45);
    robot.delay_after_finish(5000);
  
}


void loop() {

  // read value of on/off switch, value can either be 1 or 0
  int switchValue = digitalRead(onOff);
  
  if(switchValue == LOW){
    // if switch is set to on, allow for robot movement
    digitalWrite(onOffswitch, HIGH);

    if(safe()){
      
      // home position
      robot.move_xyzRPY_fast(SHOULDER_OFFSET, 0, HEIGHT_2_SECOND_MOTOR + LENGTH_ARM_2 + CLAW_LENGTH, 
      0, 90, 45);
      robot.delay_after_finish(5000);
    
      // calculate average distance and get avgdist value
      sensorAverageDist();
      Serial.println(ultraMeasurementMean());
      Serial.println(IRmeasurementMean());
      Serial.println(sensorAverageDist());
  
      gripAble();
    
      if(gripAble()){
        // movement commands for sequence
        movementSequence(); 
      }else{
        gripAble();
      }
  
    }
  }else{
    // if button is set to off, do not allow robot movement
    digitalWrite(onOffswitch, LOW);
  }
 
}
