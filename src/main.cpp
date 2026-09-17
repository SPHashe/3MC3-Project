#include <Arduino.h>
#include <Servo.h>

/*

***************************************************************************
***************************************************************************
*** Project: Robotic Arm Control for 3MC3 - Motion Control and robotics ***
*** Brought to you by Vic Cuatico, Nahor Debesay, and Parsa Hashemi     ***
***                                                                     ***
*** Hardware Configuration:                                             ***
*** - Base Servo: Pin 9                                                 ***
*** - Arm Joint 1 Servo (Left Servo): Pin 10                            ***
*** - Arm Joint 2 Servo (Right Servo): Pin 11                           ***
*** - Gripper Servo: Pin 12                                             ***
***************************************************************************
***************************************************************************

*/


//Servo objects
Servo BaseServo;
Servo ArmJoint1Servo;
Servo ArmJoint2Servo;
Servo GripperServo;


// Pin assignments
const int BaseServoPin = 9; // Pin for base servo
const int ArmJoint1ServoPin = 10; // Pin for arm joint 1 servo
const int ArmJoint2ServoPin = 11; // Pin for arm joint 2 servo
const int GripperServoPin = 12; // Pin for gripper servo

const int BasePotPin = A0; // Pin for base potentiometer
const int ArmJoint1PotPin = A1; // Pin for arm joint 1 potentiometer
const int ArmJoint2PotPin = A2; // Pin for arm joint 2 potentiometer
const int GripperPotPin = A3; // Pin for gripper potentiometer

// Servo control parameters

const int ServoIncrement = 5; // Change this to set the target increment in degrees
const int ServoDeadband = 3; // Required extra change before switching buckets. Make sure this is less than ServoIncrement to avoid skipping buckets.

const int BaseMaxPosition = 180; // Maximum position for base servo
const int ArmJoint1MaxPosition = 180; // Maximum position for arm joint 1 servo // Was 125
const int ArmJoint2MaxPosition = 180; // Maximum position for arm joint 2 //Was 80
const int GripperMaxPosition = 180; // Maximum position for gripper servo

int QuantizePosition(int position) // Incrementation of the position to the nearest ServoIncrement
{
  return (position / ServoIncrement) * ServoIncrement;
}

int StablePosition(int position, int previousPosition)
{
  int quantizedPosition = QuantizePosition(position);

  if (quantizedPosition > previousPosition && position >= quantizedPosition + ServoDeadband)
  {
    return quantizedPosition;
  }

  if (quantizedPosition < previousPosition && position <= previousPosition - ServoDeadband)
  {
    return quantizedPosition;
  }

  return previousPosition;
}

//Prototype functions
void ArmInitialize();

void MoveBase(int speed, int position);
void MoveArmJoint1(int speed, int position);
void MoveArmJoint2(int speed, int position);
void MoveGripper(int speed, int position);

//Global variables
int StartingBasePosition = 80; // Starting position of the base servo
int StartingArmJoint1Position = 120; // Starting position of arm joint 1 servo
int StartingArmJoint2Position = 60; // Starting position of arm joint 2 servo
int StartingGripperPosition = 180; // Starting position of gripper servo

int PreviousBasePosition = StartingBasePosition; // Current position of the base servo
int PreviousArmJoint1Position = StartingArmJoint1Position; // Current position of arm joint 1 servo
int PreviousArmJoint2Position = StartingArmJoint2Position; // Current position of arm joint 2 servo
int PreviousGripperPosition = StartingGripperPosition; // Current position of gripper servo


void setup() 
{
  Serial.begin(9600); // Initialize serial communication for debugging

  ArmInitialize();

  pinMode(BasePotPin, INPUT_PULLUP);
  pinMode(ArmJoint1PotPin, INPUT_PULLUP);
  pinMode(ArmJoint2PotPin, INPUT_PULLUP);
  pinMode(GripperPotPin, INPUT_PULLUP);
  
  Serial.println("Robotic Arm Control Initialized");
  delay(1000); // Wait for a second before starting the loop
}

void loop() 
{
  int BasePosition = StablePosition(map(analogRead(BasePotPin), 0, 1023, 0, 180), PreviousBasePosition); // Map potentiometer value to servo angle
  int ArmJoint1Position = StablePosition(map(analogRead(ArmJoint1PotPin), 0, 1023, 0, 180), PreviousArmJoint1Position); // Map potentiometer value to servo angle
  int ArmJoint2Position = StablePosition(map(analogRead(ArmJoint2PotPin), 0, 1023, 0, 180), PreviousArmJoint2Position); // Map potentiometer value to servo angle
  int GripperPosition = StablePosition(map(analogRead(GripperPotPin), 0, 1023, 0, 180), PreviousGripperPosition); // Map potentiometer value to servo angle


  // Debugging output to serial monitor
  Serial.print("Base Position: "); Serial.print(BasePosition);
  Serial.print(" | Arm Joint 1 Position: "); Serial.print(ArmJoint1Position);
  Serial.print(" | Arm Joint 2 Position: "); Serial.print(ArmJoint2Position);
  Serial.print(" | Gripper Position: "); Serial.println(GripperPosition);  

  MoveBase(1, BasePosition); // Move base servo based on potentiometer value
  MoveArmJoint1(1, ArmJoint1Position); // Move arm joint 1 servo based on potentiometer value
  MoveArmJoint2(1, ArmJoint2Position); // Move arm joint 2 servo based on potentiometer value
  MoveGripper(1, GripperPosition); // Move gripper servo based on potentiometer value
}

void ArmInitialize() 
{
  BaseServo.attach(BaseServoPin); // Attach base servo to pin 9
  ArmJoint1Servo.attach(ArmJoint1ServoPin); // Attach arm joint 1 servo to pin 10
  ArmJoint2Servo.attach(ArmJoint2ServoPin); // Attach arm joint 2 servo to pin 11
  GripperServo.attach(GripperServoPin); // Attach gripper servo to pin 12

  MoveBase(1, StartingBasePosition); // Move base to initial position
  MoveArmJoint1(1, StartingArmJoint1Position); // Move arm joint 1 to initial position
  MoveArmJoint2(1, StartingArmJoint2Position); // Move arm joint 2 to initial position
  MoveGripper(1, StartingGripperPosition); // Move gripper to initial position
}

void MoveBase(int speed, int position) 
{
  position = constrain(position, 0, BaseMaxPosition);
  speed = max(1, speed);
  int step = (position >= PreviousBasePosition) ? speed : -speed;
  for (int pos = PreviousBasePosition; (step > 0) ? pos <= position : pos >= position; pos += step) 
  {
    BaseServo.write(pos); // Move base servo to the specified position
    PreviousBasePosition = pos; // Update the previous position
    delay(15); // Wait for the servo to reach the position
  }
  BaseServo.write(position);
  PreviousBasePosition = position;
}

void MoveArmJoint1(int speed, int position) 
{
  position = constrain(position, 0, ArmJoint1MaxPosition);
  speed = max(1, speed);
  int step = (position >= PreviousArmJoint1Position) ? speed : -speed;
  for (int pos = PreviousArmJoint1Position; (step > 0) ? pos <= position : pos >= position; pos += step) 
  {
    ArmJoint1Servo.write(pos); // Move arm joint 1 servo to the specified position
    PreviousArmJoint1Position = pos; // Update the previous position
    delay(15); // Wait for the servo to reach the position
  }
  ArmJoint1Servo.write(position);
  PreviousArmJoint1Position = position;
}

void MoveArmJoint2(int speed, int position) 
{
  position = constrain(position, 0, ArmJoint2MaxPosition);
  speed = max(1, speed);
  int step = (position >= PreviousArmJoint2Position) ? speed : -speed;
  for (int pos = PreviousArmJoint2Position; (step > 0) ? pos <= position : pos >= position; pos += step) 
  {
    ArmJoint2Servo.write(pos); // Move arm joint 2 servo to the specified position
    PreviousArmJoint2Position = pos; // Update the previous position
    delay(15); // Wait for the servo to reach the position
  }
  ArmJoint2Servo.write(position);
  PreviousArmJoint2Position = position;
}

void MoveGripper(int speed, int position) 
{
  position = constrain(position, 0, GripperMaxPosition);
  speed = max(1, speed);
  int step = (position >= PreviousGripperPosition) ? speed : -speed;
  for (int pos = PreviousGripperPosition; (step > 0) ? pos <= position : pos >= position; pos += step) 
  {
    GripperServo.write(pos); // Move gripper servo to the specified position
    PreviousGripperPosition = pos; // Update the previous position
    delay(15); // Wait for the servo to reach the position
  }
  GripperServo.write(position);
  PreviousGripperPosition = position;
}