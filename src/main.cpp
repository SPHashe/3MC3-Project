#include <Arduino.h>
#include <ArmController.h>

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


ArmController Arm;

const int WorldXMin = 120, WorldXMax = 360;
const int WorldYMin = -120, WorldYMax = 150;
const int WorldZMin = -120, WorldZMax = 100;

//Manual mode selection
bool ManualMode = false; // Set to true for manual control, false for automatic control
enum PotentiometerMode { DegreeMode, WorldMode };
PotentiometerMode CurrentPotentiometerMode = DegreeMode;

//Manual control variables
  // Pin assignments
  const int ManualModeButtonPin = 2; // Pin for manual mode button

#if defined(ARDUINO_ARCH_ESP32) //This is when I move to Esp32
  const int BaseServoPin = 18;
  const int ArmJoint1ServoPin = 19;
  const int ArmJoint2ServoPin = 21;
  const int GripperServoPin = 22;
  const int BasePotPin = 32;
  const int ArmJoint1PotPin = 33;
  const int ArmJoint2PotPin = 34;
  const int GripperPotPin = 35;
#else
  const int BaseServoPin = 9; // Pin for base servo
  const int ArmJoint1ServoPin = 10; // Pin for arm joint 1 servo
  const int ArmJoint2ServoPin = 11; // Pin for arm joint 2 servo
  const int GripperServoPin = 12; // Pin for gripper servo

  const int BasePotPin = A0; // Pin for base potentiometer
  const int ArmJoint1PotPin = A1; // Pin for arm joint 1 potentiometer
  const int ArmJoint2PotPin = A2; // Pin for arm joint 2 potentiometer
  const int GripperPotPin = A3; // Pin for gripper potentiometer
#endif

  //manual control functions
  void ManualControl();

  //Automatic control functions
  void AutomaticControl();



void setup() 
{
  Serial.begin(9600); // Initialize serial communication for debugging

  Arm.begin(BaseServoPin, ArmJoint1ServoPin, ArmJoint2ServoPin, GripperServoPin);

  //manual control pin modes
  pinMode(BasePotPin, INPUT);
  pinMode(ArmJoint1PotPin, INPUT);
  pinMode(ArmJoint2PotPin, INPUT);
  pinMode(GripperPotPin, INPUT);
  pinMode(ManualModeButtonPin, INPUT);
  
  Serial.println("Robotic Arm Control Initialized");
  delay(1000); // Wait for a second before starting the loop
}

void loop() 
{
  if (Serial.available() > 0) //Future ready for manual control via serial commands in bluetooth or USB serial communication. For now, it will switch between manual and automatic modes based on the received command.
  {
    char command = Serial.read();
    if (command == 'W' || command == 'w')
    {
      CurrentPotentiometerMode = WorldMode;
      Serial.println("World XYZ Mode");
    }
    else if (command == 'D' || command == 'd')
    {
      CurrentPotentiometerMode = DegreeMode;
      Serial.println("Degree Mode");
    }
  }

  ManualMode = digitalRead(ManualModeButtonPin) == HIGH;

  if (ManualMode) 
  {
    ManualControl();
  } 
  else 
  {
    AutomaticControl();
  }

}



void ManualControl() 
{
  int GripperPosition = map(analogRead(GripperPotPin), 0, 1023, 0, 180); // Map potentiometer value to servo angle
  Arm.setManualGripperPosition(GripperPosition);
  bool reachable = true;
  float x = 0.0f, y = 0.0f, z = 0.0f;

  if (CurrentPotentiometerMode == WorldMode)
  {
    static float fx = 512, fy = 512, fz = 512;          // smoothed pot readings
    fx += 0.2f * (analogRead(BasePotPin)      - fx);
    fy += 0.2f * (analogRead(ArmJoint1PotPin) - fy);
    fz += 0.2f * (analogRead(ArmJoint2PotPin) - fz);

    x = WorldXMin + (WorldXMax - WorldXMin) * fx / 1023.0f;
    y = WorldYMin + (WorldYMax - WorldYMin) * fy / 1023.0f;
    z = WorldZMin + (WorldZMax - WorldZMin) * fz / 1023.0f;
    reachable = Arm.moveToPosition(x, y, z, ManualModeButtonPin);
  }
  else
  {
    int BasePosition = map(analogRead(BasePotPin), 0, 1023, 0, 180); // Map potentiometer value to servo angle
    int ArmJoint1Position = map(analogRead(ArmJoint1PotPin), 0, 1023, 0, 180); // Map potentiometer value to servo angle
    int ArmJoint2Position = map(analogRead(ArmJoint2PotPin), 0, 1023, 0, 180); // Map potentiometer value to servo angle

    Arm.moveJoints(BasePosition, ArmJoint1Position, ArmJoint2Position, 1, ManualModeButtonPin);
  }

  static unsigned long LastManualStatusTime = 0;
  if (millis() - LastManualStatusTime >= 250)
  {
    Serial.print("Base: ");
    Serial.print(Arm.baseAngle(), 1);
    Serial.print(" deg, Joint 1: ");
    Serial.print(Arm.armJoint1Angle(), 1);
    Serial.print(" deg, Joint 2: ");
    Serial.print(Arm.armJoint2Angle(), 1);
    Serial.print(" deg, Grip: ");
    Serial.print(Arm.gripperAngle());
    Serial.print(" deg");

    if (CurrentPotentiometerMode == WorldMode)
    {
      Serial.print(" | Target XYZ: ");
      Serial.print(x, 1);
      Serial.print(", ");
      Serial.print(y, 1);
      Serial.print(", ");
      Serial.print(z, 1);
      Serial.print(" | Reachable: ");
      if (reachable)
      {
        Serial.println("true");
      }
      else
      {
        Serial.println("false");
      }
    }
    else
    {
      Serial.println();
    }

    LastManualStatusTime = millis();
  }
}

void AutomaticControl() 
{
}
