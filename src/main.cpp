#include <Arduino.h>
#include <Servo.h>
/*
*********************************
Project: Robotic Arm Control for 3MC3 - Motion Control and robotics
Brought to you by Vic Cuatico, Nahor Debesay, and Parsa Hashemi

Hardware Configuration:
- Base Servo: Pin 9
- Arm Joint 1 Servo (Left Servo): Pin 10
- Arm Joint 2 Servo (Right Servo): Pin 11
- Gripper Servo: Pin 12
*/

//Attached devices
int PotPinBase = A0; // Potentiometer pin for base servo
int PotPinArmJoint1 = A1; // Potentiometer pin for arm joint 1 servo
int PotPinArmJoint2 = A2; // Potentiometer pin for arm joint 2 servo
int PotPinGripper = A3; // Potentiometer pin for gripper servo

//Servo objects
Servo BaseServo;
Servo ArmJoint1Servo;
Servo ArmJoint2Servo;
Servo GripperServo;

//Prototype functions
void ArmInitialize();

void MoveBase(int speed, int position);
void MoveArmJoint1(int speed, int position);
void MoveArmJoint2(int speed, int position);
void MoveGripper(int speed, int position);

//Global variables
float PreviousBasePosition = 180; // Current position of the base servo
float PreviousArmJoint1Position = 180; // Current position of arm joint 1 servo
float PreviousArmJoint2Position = 180; // Current position of arm joint 2 servo
float PreviousGripperPosition = 180; // Current position of gripper servo

void setup() 
{
  Serial.begin(9600); // Initialize serial communication for debugging

  ArmInitialize();

  pinMode(PotPinBase, INPUT);
  pinMode(PotPinArmJoint1, INPUT);
  pinMode(PotPinArmJoint2, INPUT);
  pinMode(PotPinGripper, INPUT);
  
  Serial.println("Robotic Arm Control Initialized");
  delay(1000); // Wait for a second before starting the loop
}

void loop() 
{
  /*
  float basePosition = analogRead(PotPinBase); // Read potentiometer value for base servo
  float armJoint1Position = analogRead(PotPinArmJoint1); // Read potentiometer value for arm joint 1 servo
  float armJoint2Position = analogRead(PotPinArmJoint2); // Read potentiometer value for arm joint 2 servo
  float gripperPosition = analogRead(PotPinGripper); // Read potentiometer value for gripper servo

  float basePositionConverted = map(basePosition, 0, 1023, 0, 180); // Map potentiometer value to servo angle (0-180 degrees)
  float armJoint1PositionConverted = map(armJoint1Position, 0, 1023, 0, 180); // Map potentiometer value to servo angle (0-180 degrees)
  float armJoint2PositionConverted = map(armJoint2Position, 0, 1023, 0, 180); // Map potentiometer value to servo angle (0-180 degrees)
  float gripperPositionConverted = map(gripperPosition, 0, 1023, 0, 180); // Map potentiometer value to servo angle (0-180 degrees)

  Serial.print("Base Position: "); Serial.println(basePositionConverted);
  Serial.print("Arm Joint 1 Position: "); Serial.println(armJoint1PositionConverted);
  Serial.print("Arm Joint 2 Position: "); Serial.println(armJoint2PositionConverted);
  Serial.print("Gripper Position: "); Serial.println(gripperPositionConverted);

  MoveBase(1, basePositionConverted); // Move base servo based on potentiometer value
  MoveArmJoint1(1, armJoint1PositionConverted); // Move arm joint 1 servo based on potentiometer value
  MoveArmJoint2(1, armJoint2PositionConverted); // Move arm joint 2 servo based on potentiometer value
  MoveGripper(1, gripperPositionConverted); // Move gripper servo based on potentiometer value
  */
}

void ArmInitialize() 
{
  BaseServo.attach(9); // Attach base servo to pin 9
  ArmJoint1Servo.attach(10); // Attach arm joint 1 servo to pin 10
  ArmJoint2Servo.attach(11); // Attach arm joint 2 servo to pin 11
  GripperServo.attach(12); // Attach gripper servo to pin 12

  MoveBase(1, 0); // Move base to initial position
  MoveArmJoint1(1, 0); // Move arm joint 1 to initial position
  MoveArmJoint2(1, 0); // Move arm joint 2 to initial position
  MoveGripper(1, 0); // Move gripper to initial position
}

void MoveBase(int speed, int position) 
{
  for (int pos = PreviousBasePosition; pos <= position; pos += speed) 
  {
    BaseServo.write(pos); // Move base servo to the specified position
    PreviousBasePosition = pos; // Update the previous position
    delay(15); // Wait for the servo to reach the position
  }
}

void MoveArmJoint1(int speed, int position) 
{
  for (int pos = PreviousArmJoint1Position; pos <= position; pos += speed) 
  {
    ArmJoint1Servo.write(pos); // Move arm joint 1 servo to the specified position
    PreviousArmJoint1Position = pos; // Update the previous position
    delay(15); // Wait for the servo to reach the position
  }
}

void MoveArmJoint2(int speed, int position) 
{
  for (int pos = PreviousArmJoint2Position; pos <= position; pos += speed) 
  {
    ArmJoint2Servo.write(pos); // Move arm joint 2 servo to the specified position
    PreviousArmJoint2Position = pos; // Update the previous position
    delay(15); // Wait for the servo to reach the position
  }
}

void MoveGripper(int speed, int position) 
{
  for (int pos = PreviousGripperPosition; pos <= position; pos += speed) 
  {
    GripperServo.write(pos); // Move gripper servo to the specified position
    PreviousGripperPosition = pos; // Update the previous position
    delay(15); // Wait for the servo to reach the position
  }
}