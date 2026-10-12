#include "ArmController.h"
#include <math.h>

  const float Link1Length = 140.0f;
  const float Link2Length = 185.0f;
  const float ShoulderHeight = 0.0f;
  const float RadiansToDegrees = 180.0f / PI;
  const float DegreesToRadians = PI / 180.0f;
  const float BaseServoOffset = 85.0f;
  const float ArmJoint1ServoOffset = 36.0f;
  const float ArmJoint2ServoOffset = 36.0f;
  const int BaseServoDirection = 1;
  const int ArmJoint1ServoDirection = 1;
  const int ArmJoint2ServoDirection = -1;
  const float CartesianStepSize = 1.0f;
  const int CartesianStepDelayMs = 5;
  const int BaseMaxPosition = 180;
  const int ArmJoint1MaxPosition = 180;
  const int ArmJoint2MaxPosition = 180;
  const int GripperMinPosition = 70;
  const int GripperMaxPosition = 150;

ArmController::ArmController()
{
    m_basePosition = 80;
    m_armJoint1Position = 120;
    m_armJoint2Position = 60;
    m_gripperServoPosition = 180;
    m_gripperPosition = false;
    m_baseAngle = 80.0f;
    m_armJoint1Angle = 120.0f;
    m_armJoint2Angle= 60.0f;
}

void ArmController::begin(int basePin, int armJoint1Pin, int armJoint2Pin, int gripperPin)
{
  m_baseServo.attach(basePin);
  m_armJoint1Servo.attach(armJoint1Pin);
  m_armJoint2Servo.attach(armJoint2Pin);
  m_gripperServo.attach(gripperPin);

  m_baseServo.write(m_basePosition);
  m_armJoint1Servo.write(m_armJoint1Position);
  m_armJoint2Servo.write(m_armJoint2Position);
  m_gripperServo.write(m_gripperServoPosition);
}

void ArmController::moveJoints(int basePosition, int armJoint1Position, int armJoint2Position, int step, int jogEnablePin)
{
  basePosition = constrain(basePosition, 0, BaseMaxPosition);
  armJoint1Position = constrain(armJoint1Position, 0, ArmJoint1MaxPosition);
  armJoint2Position = constrain(armJoint2Position, 0, ArmJoint2MaxPosition);
  step = max(1, step);

  while (m_basePosition != basePosition ||
         m_armJoint1Position != armJoint1Position ||
         m_armJoint2Position != armJoint2Position)
  {
    if (jogEnablePin >= 0 && digitalRead(jogEnablePin) != HIGH)
    {
      break;
    }

    if (m_basePosition != basePosition)
    {
      if (basePosition > m_basePosition)
      {
        m_basePosition += min(step, basePosition - m_basePosition);
      }
      else
      {
        m_basePosition -= min(step, m_basePosition - basePosition);
      }
    }
    if (m_armJoint1Position != armJoint1Position)
    {
      if (armJoint1Position > m_armJoint1Position)
      {
        m_armJoint1Position += min(step, armJoint1Position - m_armJoint1Position);
      }
      else
      {
        m_armJoint1Position -= min(step, m_armJoint1Position - armJoint1Position);
      }
    }
    if (m_armJoint2Position != armJoint2Position)
    {
      if (armJoint2Position > m_armJoint2Position)
      {
        m_armJoint2Position += min(step, armJoint2Position - m_armJoint2Position);
      }
      else
      {
        m_armJoint2Position -= min(step, m_armJoint2Position - armJoint2Position);
      }
    }
    m_baseServo.write(m_basePosition);
    m_armJoint1Servo.write(m_armJoint1Position);
    m_armJoint2Servo.write(m_armJoint2Position);
    delay(15);
  }

  m_baseAngle = m_basePosition;
  m_armJoint1Angle = m_armJoint1Position;
  m_armJoint2Angle = m_armJoint2Position;
}

bool ArmController::inverseKinematics(float x, float y, float z, Joints &angles) const
{
  float radius = sqrtf(x * x + y * y);
  if (radius < 20.0f)
  {
    return false;
  }

  float baseAngle = atan2f(y, x);
  float height = z - ShoulderHeight;
  float distanceSquared = radius * radius + height * height;
  float distance = sqrtf(distanceSquared);

  float cosShoulder = (Link1Length * Link1Length + distanceSquared - Link2Length * Link2Length) /
                      (2.0f * Link1Length * distance);
  float cosElbow = (Link1Length * Link1Length + Link2Length * Link2Length - distanceSquared) /
                   (2.0f * Link1Length * Link2Length);
  if (cosShoulder < -1.0f || cosShoulder > 1.0f || cosElbow < -1.0f || cosElbow > 1.0f)
  {
    return false;
  }

  float upperArmAngle = atan2f(height, radius) + acosf(cosShoulder);
  float forearmAngle = upperArmAngle + acosf(cosElbow) - PI;

  angles.base = baseAngle;
  angles.th1 = upperArmAngle;
  angles.th2 = forearmAngle - upperArmAngle;
  return true;
}

void ArmController::forwardKinematics(const Joints &angles, float &x, float &y, float &z) const
{
  float radius = Link1Length * cosf(angles.th1) + Link2Length * cosf(angles.th1 + angles.th2);
  z = ShoulderHeight + Link1Length * sinf(angles.th1) + Link2Length * sinf(angles.th1 + angles.th2);
  x = radius * cosf(angles.base);
  y = radius * sinf(angles.base);
}

float ArmController::degreesToMicroseconds(float angle) const
{
  angle = constrain(angle, 0.0f, 180.0f);
  return 544.0f + angle * (2400.0f - 544.0f) / 180.0f;
}

void ArmController::writeWorldPose(float baseAngle, float armJoint1Angle, float armJoint2Angle)
{
  m_baseServo.writeMicroseconds((int)lroundf(degreesToMicroseconds(baseAngle)));
  m_armJoint1Servo.writeMicroseconds((int)lroundf(degreesToMicroseconds(armJoint1Angle)));
  m_armJoint2Servo.writeMicroseconds((int)lroundf(degreesToMicroseconds(armJoint2Angle)));

  m_baseAngle = baseAngle;
  m_armJoint1Angle = armJoint1Angle;
  m_armJoint2Angle = armJoint2Angle;
  m_basePosition = lroundf(baseAngle);
  m_armJoint1Position = lroundf(armJoint1Angle);
  m_armJoint2Position = lroundf(armJoint2Angle);
}

bool ArmController::getServoPositions(float x, float y, float z, float &baseAngle, float &armJoint1Angle, float &armJoint2Angle) const
{
  Joints angles;
  if (!inverseKinematics(x, y, z, angles))
  {
    return false;
  }

  float forearmAngle = angles.th1 + angles.th2;
  baseAngle = BaseServoOffset + BaseServoDirection * angles.base * RadiansToDegrees;
  armJoint1Angle = ArmJoint1ServoOffset + ArmJoint1ServoDirection * angles.th1 * RadiansToDegrees;
  armJoint2Angle = ArmJoint2ServoOffset + ArmJoint2ServoDirection * forearmAngle * RadiansToDegrees;

  return baseAngle >= 0 && baseAngle <= BaseMaxPosition &&
         armJoint1Angle >= 0 && armJoint1Angle <= ArmJoint1MaxPosition &&
         armJoint2Angle >= 0 && armJoint2Angle <= ArmJoint2MaxPosition;
}

bool ArmController::moveToPosition(float x, float y, float z, int jogEnablePin)
{
  Joints currentAngles;
  currentAngles.base = (m_baseAngle - BaseServoOffset) / BaseServoDirection * DegreesToRadians;
  currentAngles.th1 = (m_armJoint1Angle - ArmJoint1ServoOffset) / ArmJoint1ServoDirection * DegreesToRadians;
  float currentForearmAngle = (m_armJoint2Angle - ArmJoint2ServoOffset) / ArmJoint2ServoDirection * DegreesToRadians;
  currentAngles.th2 = currentForearmAngle - currentAngles.th1;

  float currentX, currentY, currentZ;
  forwardKinematics(currentAngles, currentX, currentY, currentZ);

  float deltaX = x - currentX;
  float deltaY = y - currentY;
  float deltaZ = z - currentZ;
  float distance = sqrtf(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

  if (distance < 1.5f)
  {
    return true;
  }

  int numberOfSteps = max(1, (int)ceilf(distance / CartesianStepSize));
  float baseAngle, armJoint1Angle, armJoint2Angle;

  for (int i = 1; i <= numberOfSteps; i++)
  {
    if (jogEnablePin >= 0 && digitalRead(jogEnablePin) != HIGH)
    {
      return false;
    }

    float fraction = (float)i / numberOfSteps;
    if (!getServoPositions(currentX + deltaX * fraction,
                           currentY + deltaY * fraction,
                           currentZ + deltaZ * fraction,
                           baseAngle, armJoint1Angle, armJoint2Angle))
    {
      return false;
    }
  }

  for (int i = 1; i <= numberOfSteps; i++)
  {
    if (jogEnablePin >= 0 && digitalRead(jogEnablePin) != HIGH)
    {
      return false;
    }

    float fraction = (float)i / numberOfSteps;
    getServoPositions(currentX + deltaX * fraction,
                      currentY + deltaY * fraction,
                      currentZ + deltaZ * fraction,
                      baseAngle, armJoint1Angle, armJoint2Angle);
    writeWorldPose(baseAngle, armJoint1Angle, armJoint2Angle);
    delay(CartesianStepDelayMs);
  }

  return true;
}

float ArmController::baseAngle() const
{
  return m_baseAngle;
}

float ArmController::armJoint1Angle() const
{
  return m_armJoint1Angle;
}

float ArmController::armJoint2Angle() const
{
  return m_armJoint2Angle;
}

int ArmController::gripperAngle() const
{
  return m_gripperServoPosition;
}

void ArmController::writeGripperPosition(int position)
{
  m_gripperServoPosition = constrain(position, GripperMinPosition, GripperMaxPosition);
  m_gripperServo.write(m_gripperServoPosition);
}

void ArmController::setManualGripperPosition(int position)
{
  writeGripperPosition(position);
}

void ArmController::setAutomaticGripper(bool position)
{
  m_gripperPosition = position;
  if (m_gripperPosition == 0)
  {
    writeGripperPosition(90);
  }
  else
  {
    writeGripperPosition(180);
  }
}