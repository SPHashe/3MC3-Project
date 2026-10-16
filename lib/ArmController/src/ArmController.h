#include <Arduino.h>
//#include <ESP32Servo.h>
#include <Servo.h>

class ArmController
{
private:
  struct Joints { float base, th1, th2; };

  Servo m_baseServo;
  Servo m_armJoint1Servo;
  Servo m_armJoint2Servo;
  Servo m_gripperServo;

  int m_basePosition;
  int m_armJoint1Position;
  int m_armJoint2Position;
  int m_gripperServoPosition;
  bool m_gripperPosition;

  float m_baseAngle;
  float m_armJoint1Angle;
  float m_armJoint2Angle;

  bool inverseKinematics(float x, float y, float z, Joints &angles) const;
  void forwardKinematics(const Joints &angles, float &x, float &y, float &z) const;
  bool getServoPositions(float x, float y, float z, float &baseAngle, float &armJoint1Angle, float &armJoint2Angle) const;
  void writeWorldPose(float baseAngle, float armJoint1Angle, float armJoint2Angle);
  void writeGripperPosition(int position);
  float degreesToMicroseconds(float angle) const;

public:
  ArmController();

  void begin(int basePin, int armJoint1Pin, int armJoint2Pin, int gripperPin);
  void moveJoints(int basePosition, int armJoint1Position, int armJoint2Position, int step = 1, int jogEnablePin = -1);
  bool moveToPosition(float x, float y, float z, int jogEnablePin = -1);
  void setManualGripperPosition(int position);
  void setAutomaticGripper(bool position);

  float baseAngle() const;
  float armJoint1Angle() const;
  float armJoint2Angle() const;
  int gripperAngle() const;
};