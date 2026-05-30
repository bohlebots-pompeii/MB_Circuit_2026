//
// Created by julius on 23.03.2025.
//
#pragma once

#include <util/Vector2.hpp>
#include <array>

#define I2C_SDA 21 // I2C
#define I2C_SCL 22

namespace GeneralConfig {
  constexpr int HAS_BALL_VALID_TIME = 200;

  constexpr double HEADING_HARD_LIMIT_DEG = 45.0;

  constexpr bool USE_COMMUNICATION = true;

  constexpr bool DISABLE_WARNINGS = false;

  constexpr bool USE_HIDDEN_BALL = false;

  constexpr double BOT_DIAMETER = 22.0;
}

namespace ObjectHeights {
  constexpr double BALL = 2.1;
  constexpr double GOAL = 6.0;
  constexpr double MIRROR = 17.46;
}

// pins
namespace PINS {
  constexpr int SINGLE_BUTTON_PIN = 19;
  constexpr int COMMS_MODULE_PIN = 36;
  constexpr int LIGHT_GATE_PIN = 39;
}

// i²c addresses
namespace I2C_ADDRESSES {
  constexpr int MOTOR_MB_ADDR = 0x69;
  constexpr int LINE_ADDR = 0x40;
  constexpr int US_ADDR = 0x50;
  constexpr int IMU_ADDR = 0x60;
  constexpr int BUTTON_MODULE_ADDR = 0x20;
}

// PID Configuration
namespace PIDConfig {
  // Y-axis motion PID
  constexpr double Y_Kp = 1.6;
  constexpr double Y_Ki = 0.25;
  constexpr double Y_Kd = 0.2;
  constexpr double Y_O_MIN = -70.0;
  constexpr double Y_O_MAX = 70.0;
  constexpr int Y_SampleTime = 21;

  // X-axis motion PID
  constexpr double X_Kp = 1.8;
  constexpr double X_Ki = 0.2;
  constexpr double X_Kd = 0.09;
  constexpr double X_O_MIN = -70.0;
  constexpr double X_O_MAX = 70.0;
  constexpr int X_SAMPLE_T = 21;

  // Rotation PID
  constexpr double ROTATION_Kp = 0.5;
  constexpr double ROTATION_Ki = 0.0;
  constexpr double ROTATION_Kd = 0.05;
  constexpr double ROTATION_O_MIN = -50.0;
  constexpr double ROTATION_O_MAX = 50.0;
  constexpr int ROTATION_SAMPLE_T = 21;

  // Rotation deadline
  constexpr double ROTATION_DEADZONE = 1.0;
}

// Goalie PID configuration
namespace GoaliePIDConfig {
  // Y-axis motion PID
  constexpr double Y_Kp = 1.8;
  constexpr double Y_Ki = 0.00;
  constexpr double Y_Kd = 0.08;
  constexpr double Y_O_MIN = -70.0;
  constexpr double Y_O_MAX = 70.0;
  constexpr int Y_SAMPLE_T = 21;

  // X-axis motion PID
  constexpr double X_Kp = 1.8;
  constexpr double X_Ki = 0.00;
  constexpr double X_Kd = 0.08;
  constexpr double X_O_MIN = -70.0;
  constexpr double X_O_MAX = 70.0;
  constexpr int X_SAMPLE_T = 21;

  // Rotation PID
  constexpr double ROTATION_Kp = 0.5;
  constexpr double ROTATION_Ki = 0.0;
  constexpr double ROTATION_Kd = 0.05;
  constexpr double ROTATION_O_MIN = -50.0;
  constexpr double ROTATION_O_MAX = 50.0;
  constexpr int ROTATION_SAMPLE_T = 21;

  // Rotation deadline
  constexpr double ROTATION_DEADZONE = 1.0;
}

namespace FieldConfig {
  // main field config
  const auto MAX_FIELD_MEASUREMENTS = Vector2(112, 72); // measure max x and max y

  const double REAL_FIELD_HEIGHT = MAX_FIELD_MEASUREMENTS.getX() * 2;
  const double REAL_FIELD_WIDTH = MAX_FIELD_MEASUREMENTS.getY() * 2;

  constexpr double PERFECT_FIELD_HEIGHT = 243.0 - GeneralConfig::BOT_DIAMETER;
  constexpr double PERFECT_FIELD_WIDTH = 182.0 - GeneralConfig::BOT_DIAMETER;

  constexpr double Hx = PERFECT_FIELD_HEIGHT / 2.0;
  constexpr double Wy = PERFECT_FIELD_WIDTH / 2.0;
  constexpr double GY = 40.0 + GeneralConfig::BOT_DIAMETER / 2.0;
  constexpr double GX = PERFECT_FIELD_HEIGHT / 2.0 - 25.0;
  constexpr double TARGET_GX = GX - GeneralConfig::BOT_DIAMETER / 2.0;

  const std::array FIELD_CONTOUR = {
    Vector2(Hx, Wy), // top right
    Vector2(Hx, GY), // down till right corner of penalty (enmy)
    Vector2(GX, GY), // in
    Vector2(GX, -GY), // down till left corner of penalty (enemy)
    Vector2(Hx, -GY), // out
    Vector2(Hx, -Wy), // bottom right
    Vector2(-Hx, -Wy), // bottom left
    Vector2(-Hx, -GY), // up till right corner of penalty (own)
    Vector2(-GX, -GY), // in
    Vector2(-GX, GY), // up till left corner of penalty (own)
    Vector2(-Hx, GY), // out
    Vector2(-Hx, Wy) // till top left
  };

  constexpr double GOAL_NEUTRAL_POS_X = -GX + GeneralConfig::BOT_DIAMETER * 1.5;
  constexpr double LINE_POS_Y = Wy - GeneralConfig::BOT_DIAMETER * 3;

  constexpr double IN_POCKET_ANGLE = 40.0;
  constexpr double POINT_REACHED_DIST = GeneralConfig::BOT_DIAMETER / 2.0;
  constexpr double KICK_DISTANCE = GX + GeneralConfig::BOT_DIAMETER * 2;

  constexpr double HARD_BARRIER = -GX + GeneralConfig::BOT_DIAMETER * 2;
}

namespace Goalie {
  constexpr unsigned long BALL_STATIONARY_MS = 2000;
  constexpr unsigned long DRIVE_TO_BALL_MS = 600;
  constexpr double BALL_MOVED_THRESH = 5.0;

  constexpr float BALL_AVOID_DIST = GeneralConfig::BOT_DIAMETER * 2.0;
  constexpr double HALF_CIRCLE_RADIUS = 25.0 + GeneralConfig::BOT_DIAMETER * 1.5;
}
