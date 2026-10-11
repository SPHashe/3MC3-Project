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

//Kinematics
  // Variables for the lengths of the arm segments
  const float L1 = 140.0f, L2 = 185.0f; // mm

  const float ShoulderHeight = 0.0f;   // mm from the world origin
  const float R0 = 0.0f;   
  const float RadiansToDegrees = 180.0f / PI;
  const float DegreesToRadians = PI / 180.0f;
  const float BaseServoOffset = 85.0f;        
  const float ArmJoint1ServoOffset = 36.0f;   
  const float ArmJoint2ServoOffset = 36.0f;   
  const int BaseServoDirection = 1;
  const int ArmJoint1ServoDirection = 1;
  const int ArmJoint2ServoDirection = -1;

  const float CartesianStepSize = 2.0f; // mm between points along a world-frame move
  const int CartesianStepDelayMs = 20;  // 2 mm / 20 ms = 100 mm/s, raise this to go slower

  const int WorldXMin = 120, WorldXMax = 360;   // placeholders, set from your real workspace
  const int WorldYMin = -120, WorldYMax = 150;
  const int WorldZMin = -120, WorldZMax = 100;  // z is relative to the shoulder axis (ShoulderHeight = 0)

  // World frame: origin at the base, +x forward, +y left, +z up.
  struct Joints { float base, th1, th2; }; // radians

  //Inverse Kinematics 

  float DegToUs(float deg);
  void WriteServos(float baseDeg, float arm1Deg, float arm2Deg, int gripperDeg);
  bool GetServoPositions(float x, float y, float z, float &baseDeg, float &arm1Deg, float &arm2Deg);
  bool MoveToPosition(float x, float y, float z, int gripperPosition);

  bool IK(float x, float y, float z, Joints &Angles);
  void fk(const Joints &Angles, float &x, float &y, float &z);

//Position Control Variables
int BasePosition = 0; // Current position of the base servo
int ArmJoint1Position = 0; // Current position of arm joint 1 servo
int ArmJoint2Position = 0; // Current position of arm joint 2 servo
int GripperPosition = 0; // Current position of gripper servo

//Manual mode selection
bool ManualMode = false; // Set to true for manual control, false for automatic control
enum PotentiometerMode { DegreeMode, WorldMode };
PotentiometerMode CurrentPotentiometerMode = DegreeMode;

//Manual control variables
  // Pin assignments
  const int ManualModeButtonPin = 2; // Pin for manual mode button

  const int BaseServoPin = 9; // Pin for base servo
  const int ArmJoint1ServoPin = 10; // Pin for arm joint 1 servo
  const int ArmJoint2ServoPin = 11; // Pin for arm joint 2 servo
  const int GripperServoPin = 12; // Pin for gripper servo

  const int BasePotPin = A0; // Pin for base potentiometer
  const int ArmJoint1PotPin = A1; // Pin for arm joint 1 potentiometer
  const int ArmJoint2PotPin = A2; // Pin for arm joint 2 potentiometer
  const int GripperPotPin = A3; // Pin for gripper potentiometer

  // Servo control parameters
  const int BaseMaxPosition = 180; // Maximum position for base servo
  const int ArmJoint1MaxPosition = 180; // Maximum position for arm joint 1 servo // Was 125
  const int ArmJoint2MaxPosition = 180; // Maximum position for arm joint 2 //Was 80
  const int GripperMaxPosition = 180; // Maximum position for gripper servo

  //manual control functions
  void ManualControl();

// Automatic control variables
  // State control
  bool HoldingItem = false; // Flag to indicate if the arm is holding an item
  bool PresentingItem = false; // Flag to indicate if the arm is presenting an item

  //Location of the cabinet locations (to be changed when hardware comes in) (To be changed to fwd or rev kinematics)
  //also please dont forgfet to change the gripper positioning
const int PositionVector[9][4] = {
  {105, 65, 105, 0},
  {85, 70, 110, 0},
  {55, 65, 95, 0},
  {105, 85, 80, 0},
  {85, 85, 80, 0},
  {55, 65, 95, 0},
  {105, 65, 105, 0},
  {85, 70, 110, 0},
  {55, 65, 95, 0}
}; 



  //Automatic control functions
  void AutomaticControl();

  //present item sequence
  void PresentItemSequence();

  //Store item sequence
  void StoreItemSequence();

  //Pickup item sequence
  void PickupItemSequence();

  //place item sequence
  void PlaceItemSequence();



//Prototype functions
void ArmInitialize();

void MovePose(int step, int basePosition, int armJoint1Position, int armJoint2Position, int gripperPosition);

//Global variables
int StartingBasePosition = 80; // Starting position of the base servo
int StartingArmJoint1Position = 120; // Starting position of arm joint 1 servo
int StartingArmJoint2Position = 60; // Starting position of arm joint 2 servo
int StartingGripperPosition = 180; // Starting position of gripper servo

int PreviousBasePosition = StartingBasePosition; // Current position of the base servo
int PreviousArmJoint1Position = StartingArmJoint1Position; // Current position of arm joint 1 servo
int PreviousArmJoint2Position = StartingArmJoint2Position; // Current position of arm joint 2 servo
int PreviousGripperPosition = StartingGripperPosition; // Current position of gripper servo

float CurBaseDeg = StartingBasePosition;
float CurArm1Deg = StartingArmJoint1Position;
float CurArm2Deg = StartingArmJoint2Position;


void setup() 
{
  Serial.begin(9600); // Initialize serial communication for debugging

  ArmInitialize();

  //manual control pin modes
  pinMode(BasePotPin, INPUT);
  pinMode(ArmJoint1PotPin, INPUT);
  pinMode(ArmJoint2PotPin, INPUT);
  pinMode(GripperPotPin, INPUT);
  pinMode(ManualModeButtonPin, INPUT_PULLUP);
  
  Serial.println("Robotic Arm Control Initialized");
  delay(1000); // Wait for a second before starting the loop
}

void loop() 
{
  // Check for manual mode button press or serial command to switch between manual and automatic modes
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
    else if (command == 'm') 
    {
      ManualMode = true; // Switch to manual mode
      Serial.println("Switched to Manual Mode");
    } 
    else if (command == 'a') 
    {
      ManualMode = false; // Switch to automatic mode
      Serial.println("Switched to Automatic Mode");
    }
  }

  if (digitalRead(ManualModeButtonPin) == HIGH) //Manual mode button is a momentary push button
  {
    ManualMode = true; // Switch to manual mode
    //Serial.println("Switched to Manual Mode");
  }
  else 
  {
    ManualMode = false; // Switch to automatic mode
    //Serial.println("Switched to Automatic Mode");
  }

  if (ManualMode) 
  {
    ManualControl();
  } 
  else 
  {
    AutomaticControl();
  }
}



void ArmInitialize() 
{
BaseServo.attach(BaseServoPin);
  ArmJoint1Servo.attach(ArmJoint1ServoPin);
  ArmJoint2Servo.attach(ArmJoint2ServoPin);
  GripperServo.attach(GripperServoPin);

  WriteServos(StartingBasePosition, StartingArmJoint1Position, StartingArmJoint2Position, StartingGripperPosition);
}

void MovePose(int step, int basePosition, int armJoint1Position, int armJoint2Position, int gripperPosition)
{
  basePosition = constrain(basePosition, 0, BaseMaxPosition);
  armJoint1Position = constrain(armJoint1Position, 0, ArmJoint1MaxPosition);
  armJoint2Position = constrain(armJoint2Position, 0, ArmJoint2MaxPosition);
  gripperPosition = constrain(gripperPosition, 0, GripperMaxPosition);
  step = max(1, step);

  while (PreviousBasePosition != basePosition ||
         PreviousArmJoint1Position != armJoint1Position ||
         PreviousArmJoint2Position != armJoint2Position ||
         PreviousGripperPosition != gripperPosition)
  {
    if (PreviousBasePosition != basePosition)
    {
      if (basePosition > PreviousBasePosition)
      {
        PreviousBasePosition += min(step, basePosition - PreviousBasePosition);
      }
      else
      {
        PreviousBasePosition -= min(step, PreviousBasePosition - basePosition);
      }
    }
    if (PreviousArmJoint1Position != armJoint1Position)
    {
      if (armJoint1Position > PreviousArmJoint1Position)
      {
        PreviousArmJoint1Position += min(step, armJoint1Position - PreviousArmJoint1Position);
      }
      else
      {
        PreviousArmJoint1Position -= min(step, PreviousArmJoint1Position - armJoint1Position);
      }
    }
    if (PreviousArmJoint2Position != armJoint2Position)
    {
      if (armJoint2Position > PreviousArmJoint2Position)
      {
        PreviousArmJoint2Position += min(step, armJoint2Position - PreviousArmJoint2Position);
      }
      else
      {
        PreviousArmJoint2Position -= min(step, PreviousArmJoint2Position - armJoint2Position);
      }
    }
    if (PreviousGripperPosition != gripperPosition)
    {
      if (gripperPosition > PreviousGripperPosition)
      {
        PreviousGripperPosition += min(step, gripperPosition - PreviousGripperPosition);
      }
      else
      {
        PreviousGripperPosition -= min(step, PreviousGripperPosition - gripperPosition);
      }
    }

    BasePosition = PreviousBasePosition;
    ArmJoint1Position = PreviousArmJoint1Position;
    ArmJoint2Position = PreviousArmJoint2Position;
    GripperPosition = PreviousGripperPosition;

    BaseServo.write(BasePosition);
    ArmJoint1Servo.write(ArmJoint1Position);
    ArmJoint2Servo.write(ArmJoint2Position);
    GripperServo.write(GripperPosition);
    delay(15);
  }

  CurBaseDeg = PreviousBasePosition;
  CurArm1Deg = PreviousArmJoint1Position;
  CurArm2Deg = PreviousArmJoint2Position;
}

void ManualControl() 
{
  int GripperPosition = map(analogRead(GripperPotPin), 0, 1023, 0, 180); // Map potentiometer value to servo angle
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
    reachable = MoveToPosition(x, y, z, GripperPosition);
  }
  else
  {
    int BasePosition = map(analogRead(BasePotPin), 0, 1023, 0, 180); // Map potentiometer value to servo angle
    int ArmJoint1Position = map(analogRead(ArmJoint1PotPin), 0, 1023, 0, 180); // Map potentiometer value to servo angle
    int ArmJoint2Position = map(analogRead(ArmJoint2PotPin), 0, 1023, 0, 180); // Map potentiometer value to servo angle

    MovePose(1, BasePosition, ArmJoint1Position, ArmJoint2Position, GripperPosition);
  }

  static unsigned long LastManualStatusTime = 0;
  if (millis() - LastManualStatusTime >= 250)
  {
    Serial.print("Base: ");
    Serial.print(CurBaseDeg, 1);
    Serial.print(" deg, Joint 1: ");
    Serial.print(CurArm1Deg, 1);
    Serial.print(" deg, Joint 2: ");
    Serial.print(CurArm2Deg, 1);
    Serial.print(" deg, Grip: ");
    Serial.print(PreviousGripperPosition);
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
  /*
  for (int i = 0; i < 9; i++) 
  {
    MovePose(1, PositionVector[i][0], PositionVector[i][1], PositionVector[i][2], PositionVector[i][3]);
    delay(1000); // Wait for a second before moving to the next position


  }
    */
}

//present item sequence
  void PresentItemSequence()
  {

  }

  //Store item sequence
  void StoreItemSequence()
  {

  }

  //Pickup item sequence
  void PickupItemSequence()
  {

  }

  //place item sequence
  void PlaceItemSequence()
  {

  }

  bool IK(float x, float y, float z, Joints &Angles)
  {
    // Step 1: base rotation picks the vertical plane that contains the target
    float r = sqrtf(x * x + y * y);          // horizontal distance from the base axis
    if (r < 20.0f) return false;             // too close to the base axis: atan2 is unstable there
    float baseAngle = atan2f(y, x);

    // Step 2: your two-link math, in that plane, using (r, h) instead of (x, z)
    float h  = z - ShoulderHeight;
    float C2 = r * r + h * h;
    float C  = sqrtf(C2);                    // shoulder -> target distance in 3D

    float cosJoint1 = (L1 * L1 + C2 - L2 * L2) / (2.0f * L1 * C);
    float cosElbow  = (L1 * L1 + L2 * L2 - C2) / (2.0f * L1 * L2);
    if (cosJoint1 < -1.0f || cosJoint1 > 1.0f || cosElbow < -1.0f || cosElbow > 1.0f)
    {
      return false;
    }

    float upperArmAngle = atan2f(h, r) + acosf(cosJoint1);        // joint 1, from the ground
    float forearmAngle  = upperArmAngle + acosf(cosElbow) - PI;   // joint 2, from the ground

    Angles.base = baseAngle;
    Angles.th1  = upperArmAngle;
    Angles.th2  = forearmAngle - upperArmAngle;   // relative angle for fk() and GetServoPositions()
    return true;
  }

  //Forward Kinematics
  void fk(const Joints &Angles, float &x, float &y, float &z)
  {
    float r = L1 * cosf(Angles.th1) + L2 * cosf(Angles.th1 + Angles.th2);
    z = ShoulderHeight + L1 * sinf(Angles.th1) + L2 * sinf(Angles.th1 + Angles.th2);
    x = r * cosf(Angles.base);
    y = r * sinf(Angles.base);
  }

  float DegToUs(float deg)
{
  deg = constrain(deg, 0.0f, 180.0f);
  return 544.0f + deg * (2400.0f - 544.0f) / 180.0f;   // same range Servo.write() uses
}

void WriteServos(float baseDeg, float arm1Deg, float arm2Deg, int gripperDeg)
{
  BaseServo.writeMicroseconds((int)lroundf(DegToUs(baseDeg)));
  ArmJoint1Servo.writeMicroseconds((int)lroundf(DegToUs(arm1Deg)));
  ArmJoint2Servo.writeMicroseconds((int)lroundf(DegToUs(arm2Deg)));
  GripperServo.write(gripperDeg);

  CurBaseDeg = baseDeg;
  CurArm1Deg = arm1Deg;
  CurArm2Deg = arm2Deg;
  PreviousBasePosition = lroundf(baseDeg);
  PreviousArmJoint1Position = lroundf(arm1Deg);
  PreviousArmJoint2Position = lroundf(arm2Deg);
  PreviousGripperPosition = gripperDeg;
}

bool GetServoPositions(float x, float y, float z, float &baseDeg, float &arm1Deg, float &arm2Deg)
{
  Joints angles;
  if (!IK(x, y, z, angles))
  {
    return false;
  }

  float forearmAngle = angles.th1 + angles.th2;   // servo 2 = forearm angle vs. ground
  baseDeg = BaseServoOffset      + BaseServoDirection      * angles.base * RadiansToDegrees;
  arm1Deg = ArmJoint1ServoOffset + ArmJoint1ServoDirection * angles.th1  * RadiansToDegrees;
  arm2Deg = ArmJoint2ServoOffset + ArmJoint2ServoDirection * forearmAngle * RadiansToDegrees;

  return baseDeg >= 0 && baseDeg <= BaseMaxPosition &&
         arm1Deg >= 0 && arm1Deg <= ArmJoint1MaxPosition &&
         arm2Deg >= 0 && arm2Deg <= ArmJoint2MaxPosition;
}

bool MoveToPosition(float x, float y, float z, int gripperPosition)
{
  if (gripperPosition < 0 || gripperPosition > GripperMaxPosition)
  {
    return false;
  }

  // Where the tool is now, from the float servo angles
  Joints currentAngles;
  currentAngles.base = (CurBaseDeg - BaseServoOffset) / BaseServoDirection * DegreesToRadians;
  currentAngles.th1  = (CurArm1Deg - ArmJoint1ServoOffset) / ArmJoint1ServoDirection * DegreesToRadians;
  float currentForearmAngle = (CurArm2Deg - ArmJoint2ServoOffset) / ArmJoint2ServoDirection * DegreesToRadians;
  currentAngles.th2 = currentForearmAngle - currentAngles.th1;

  float currentX, currentY, currentZ;
  fk(currentAngles, currentX, currentY, currentZ);

  float deltaX = x - currentX;
  float deltaY = y - currentY;
  float deltaZ = z - currentZ;
  float distance = sqrtf(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

  // Deadband: ignore pot noise, only update the gripper
  if (distance < 1.5f)
  {
    GripperServo.write(gripperPosition);
    PreviousGripperPosition = gripperPosition;
    return true;
  }

  int numberOfSteps = max(1, (int)ceilf(distance / CartesianStepSize));

  // Check the whole path first so the arm never stops halfway
  float b, a1, a2;
  for (int i = 1; i <= numberOfSteps; i++)
  {
    float f = (float)i / numberOfSteps;
    if (!GetServoPositions(currentX + deltaX * f, currentY + deltaY * f, currentZ + deltaZ * f, b, a1, a2))
    {
      return false;
    }
  }

  // Follow the line: all joints are written together at every step
  for (int i = 1; i <= numberOfSteps; i++)
  {
    float f = (float)i / numberOfSteps;
    GetServoPositions(currentX + deltaX * f, currentY + deltaY * f, currentZ + deltaZ * f, b, a1, a2);
    WriteServos(b, a1, a2, gripperPosition);
    delay(CartesianStepDelayMs);
  }

  return true;
}

