//
// Created by julius on 23.03.2025.
//
#pragma once

#define I2C_SDA 21 // I2C
#define I2C_SCL 22

// pins
constexpr int buttonPIN = 19;
constexpr int piezoPIN = 23;

// i²c adresses
constexpr int motorMBAddress = 0x69;

constexpr int lineSensorAddress = 0x40;
constexpr int usAddress = 0x50;
constexpr int imuAddress = 0x60;

constexpr int buttonModuleAddress = 0x20;

constexpr double maxDistance = 80.0;
constexpr double slowingDistance = 60.0;

// Goalie config
constexpr unsigned long BALL_STATIONARY_MS = 2000;
constexpr unsigned long DRIVE_TO_BALL_MS = 600;
constexpr double BALL_MOVED_THRESH = 5.0;

// PID Configuration
namespace PIDConfig {
    // Y axis motion PID
    constexpr double Y_Kp = 1.6;
    constexpr double Y_Ki = 0.0;
    constexpr double Y_Kd = 0.05;
    constexpr double Y_OutputMin = -70.0;
    constexpr double Y_OutputMax = 70.0;
    constexpr int Y_SampleTime = 30;

    // X axis motion PID
    constexpr double X_Kp = 1.6;
    constexpr double X_Ki = 0.0;
    constexpr double X_Kd = 0.05;
    constexpr double X_OutputMin = -70.0;
    constexpr double X_OutputMax = 70.0;
    constexpr int X_SampleTime = 30;

    // Rotation PID
    constexpr double Rot_Kp = 0.5;
    constexpr double Rot_Ki = 0.0;
    constexpr double Rot_Kd = 0.05;
    constexpr double Rot_OutputMin = -50.0;
    constexpr double Rot_OutputMax = 50.0;
    constexpr int Rot_SampleTime = 30;

    // Rotation deadzone
    constexpr double Rot_Deadzone = 5.0;
}

namespace FieldConfig {
    constexpr double FieldHalfWidth = 100.0;
    constexpr double FieldHalfHeight = 100.0;

    constexpr double GoalHalfWidth = 40.0;

    constexpr double FieldLinePositionY = 80.0;

    constexpr double FieldPocketPositionX = 60.0;
    constexpr double FieldPocketPositionY = 40.0;

    constexpr double NeutralPointPositionX = 70.0;
    constexpr double NeutralPointPositionY = 40.0;

    constexpr double GoalNeutralPointPositionX = -80.0;

    constexpr double PointReachedDistance = 10.0;
}


