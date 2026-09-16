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

const int BaseServoPin = 9; // Pin for base servo
const int ArmJoint1ServoPin = 10; // Pin for arm joint 1 servo
const int ArmJoint2ServoPin = 11; // Pin for arm joint 2 servo
const int GripperServoPin = 12; // Pin for gripper servo

//Prototype functions
void ArmInitialize();

void MoveBase(int speed, int position);
void MoveArmJoint1(int speed, int position);
void MoveArmJoint2(int speed, int position);
void MoveGripper(int speed, int position);

//Global variables
float StartingBasePosition = 180; // Starting position of the base servo
float StartingArmJoint1Position = 180; // Starting position of arm joint 1 servo
float StartingArmJoint2Position = 180; // Starting position of arm joint 2 servo
float StartingGripperPosition = 180; // Starting position of gripper servo

float PreviousBasePosition = StartingBasePosition; // Current position of the base servo
float PreviousArmJoint1Position = StartingArmJoint1Position; // Current position of arm joint 1 servo
float PreviousArmJoint2Position = StartingArmJoint2Position; // Current position of arm joint 2 servo
float PreviousGripperPosition = StartingGripperPosition; // Current position of gripper servo


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
  float BasePosition = map(analogRead(PotPinBase), 0, 1023, 0, 180); // Map potentiometer value to servo angle (0-180 degrees)
  float ArmJoint1Position = map(analogRead(PotPinArmJoint1), 0, 1023, 0, 180); // Map potentiometer value to servo angle (0-180 degrees)
  float ArmJoint2Position = map(analogRead(PotPinArmJoint2), 0, 1023, 0, 180); // Map potentiometer value to servo angle (0-180 degrees)
  float GripperPosition = map(analogRead(PotPinGripper), 0, 1023, 0, 180); // Map potentiometer value to servo angle (0-180 degrees)


  /* // Debugging output to serial monitor
  Serial.print("Base Position: "); Serial.println(BasePosition);
  Serial.print("Arm Joint 1 Position: "); Serial.println(ArmJoint1Position);
  Serial.print("Arm Joint 2 Position: "); Serial.println(ArmJoint2Position);
  Serial.print("Gripper Position: "); Serial.println(GripperPosition);
  */

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