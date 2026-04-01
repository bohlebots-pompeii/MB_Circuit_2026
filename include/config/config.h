//
// Created by julius on 23.03.2025.
//
#pragma once

#define I2C_SDA 21 // I2C
#define I2C_SCL 22

namespace GeneralConfig {
  constexpr int HasBallValidTime = 200;

  constexpr bool USE_COMMUNICATION = false;

  constexpr bool DISABLE_WARNINGS = true;
}

// pins
namespace PINS {
  constexpr int buttonPIN = 19;
  constexpr int piezoPIN = 23;
  constexpr int communicationModulePIN = 36;
  constexpr int lightGatePIN = 39;
}

// i²c addresses
namespace I2C_ADDRESSES {
  constexpr int motorMBAddress = 0x69;
  constexpr int lineSensorAddress = 0x40;
  constexpr int usAddress = 0x50;
  constexpr int imuAddress = 0x60;
  constexpr int buttonModuleAddress = 0x20;
}

namespace SpeedLimiting {
  constexpr double maxDistance = 80.0;
  constexpr double slowingDistance = 60.0;
}

namespace Goalie {
  constexpr unsigned long BALL_STATIONARY_MS = 2000;
  constexpr unsigned long DRIVE_TO_BALL_MS = 600;
  constexpr double BALL_MOVED_THRESH = 5.0;
  constexpr float BALL_AVOID_DIST = 50.0f;
  constexpr double HALF_CIRCLE_RADIUS = 60.0;
}

// PID Configuration
namespace PIDConfig {
  // Y-axis motion PID
  constexpr double Y_Kp = 2.0;
  constexpr double Y_Ki = 0.07;
  constexpr double Y_Kd = 0.065;
  constexpr double Y_OutputMin = -70.0;
  constexpr double Y_OutputMax = 70.0;
  constexpr int Y_SampleTime = 21;

  // X-axis motion PID
  constexpr double X_Kp = 2.0;
  constexpr double X_Ki = 0.07;
  constexpr double X_Kd = 0.065;
  constexpr double X_OutputMin = -70.0;
  constexpr double X_OutputMax = 70.0;
  constexpr int X_SampleTime = 21;

  // Rotation PID
  constexpr double Rot_Kp = 0.5;
  constexpr double Rot_Ki = 0.0;
  constexpr double Rot_Kd = 0.05;
  constexpr double Rot_OutputMin = -50.0;
  constexpr double Rot_OutputMax = 50.0;
  constexpr int Rot_SampleTime = 21;

  // Rotation deadline
  constexpr double Rot_deadline = 1.0;
}

namespace FieldConfig {
  constexpr double FieldHalfWidth = 100.0;
  constexpr double FieldHalfHeight = 100.0;

  constexpr double GoalHalfWidth = 40.0;

  constexpr double FieldLinePositionY = 80.0;

  constexpr double FieldPocketPositionX = 60.0;
  constexpr double FieldPocketPositionY = 40.0;
  constexpr double FieldPocketAngle = 40.0;

  constexpr double NeutralPointPositionX = 70.0;
  constexpr double NeutralPointPositionY = 40.0;

  constexpr double GoalNeutralPointPositionX = -80.0;

  constexpr double PointReachedDistance = 10.0;

  constexpr double kickDistance = 170.0;
  constexpr double rotateToBallAngle = 70.0;
}
