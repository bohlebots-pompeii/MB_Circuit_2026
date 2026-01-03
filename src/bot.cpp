//
// Created by julius on 11.11.2025.
//

#include <bot.h>
#include <config.h>
#include <comms/i2c.h>
#include <Arduino.h>
#include <Vector2.hpp>
#include <util/piezo.h>
#include <comms/serial.h>

Piezo buzzer(piezoPIN, 0, 2000, 8);

void Bot::init() {
  I2C::init();
  buzzer.begin();

  //set_heading();

  pinMode(buttonPIN, INPUT);
}

Vector2 degreeToVector(const float degrees) {
  const float radians = degrees * (PI / 180.0f);
  return Vector2(cosf(radians), sinf(radians));
}

void Bot::update() {
  constexpr int speed = 30;

  getSensorData(); // process i2c data

  updateCM5(); // proccess cm5 data

  if (isHoming) {
    home();
    buzzer.playTone(1000, 200, 128);
    return;
  }

  int vx, vy;
  if (line_rot != -1 && progress != -1) {
    Vector2 lineDir = degreeToVector(line_rot);

    lineDir.rotate(M_PI);

    vx = static_cast<int>(roundf(lineDir.getY() * 100));
    vy = static_cast<int>(roundf(lineDir.getX() * 100));

    pushData(_ena, false, static_cast<int>(roundf(vx)), static_cast<int>(roundf(vy)), 0, 0);
    return;
  }

  int rot = 0 - static_cast<int>(heading) / 4;

  float ballrot = 0.0f;
  float balldist = 0.0f;

  float yellow_rot = 0.0f;

  for (int i = 0; i < public_num_detections; ++i) {
    if (public_detections[i].label == 3) {
      ballrot = public_detections[i].rotation_deg;
      balldist = public_detections[i].dist_cm;
      break;
    }
    if (public_detections[i].label == 2) {
      yellow_rot = public_detections[i].rotation_deg;
    }
  }

  /*
  for (int i = 0; i < public_num_detections; ++i) {
    Serial.print("Label ");
    Serial.print(public_detections[i].label);
    Serial.print(" dist ");
    Serial.println(public_detections[i].dist_cm);
    //Serial.print(" x ");
    //Serial.print(public_detections[i].rel_x);
    //Serial.print(" y ");
    //erial.println(public_detections[i].rel_y);
  }
  */

  if (balldist > 100) {
    balldist = 100;
  }

  const float ballRad = ballrot * (PI / 180.0f);
  auto target = Vector2(cosf(ballRad) * balldist, sinf(ballRad) * balldist);

  target.normalize();

  if (abs(ballrot) > 30.0f) {
    if (balldist < 25.0f) {
      if (ballrot > 0) {
        target.rotate(M_PI / 2);
      }
      else {
        target.rotate(-M_PI / 2);
      }
    }
  }

  if (abs(ballrot) < 10.0f) {
    target = degreeToVector(yellow_rot);
     rot = 0 - static_cast<int>(-yellow_rot) / 2;
  }

  vx = static_cast<int>(roundf(target.getY() * speed));
  vy = static_cast<int>(roundf(target.getX() * speed));

  pushData(_ena, false, static_cast<int>(roundf(vx)), static_cast<int>(roundf(vy)), rot, 0);
}

void Bot::getSensorData() {
  // US Positon
  constexpr uint8_t numBytes = 4;
  I2C::requestData(usCircuit, numBytes);
  float local_x = 0;
  float local_y = 0;

  if (I2C::available() >= numBytes) {
    const uint8_t xLow  = I2C::read();
    const uint8_t xHigh = I2C::read();
    const uint8_t yLow  = I2C::read();
    const uint8_t yHigh = I2C::read();

    const uint16_t x_u = (static_cast<uint16_t>(xHigh) << 8) | static_cast<uint16_t>(xLow);
    const uint16_t y_u = (static_cast<uint16_t>(yHigh) << 8) | static_cast<uint16_t>(yLow);

    const int16_t x = static_cast<int16_t>(x_u);
    const int16_t y = static_cast<int16_t>(y_u);

    constexpr float scale = 100.0f;
    local_x = static_cast<float>(x) / scale;
    local_y = static_cast<float>(y) / scale;
  }

  // Ground Sensor
  constexpr uint8_t len = 4;
  I2C::requestData(groundSensor, len);
  if (I2C::available() >= len) {
    const uint8_t progressLow  = I2C::read();
    const uint8_t progressHigh = I2C::read();
    const uint8_t lineRotLow  = I2C::read();
    const uint8_t lineRotHigh = I2C::read();

    const uint16_t lineRot_u = (static_cast<uint16_t>(lineRotHigh) << 8) | static_cast<uint16_t>(lineRotLow);
    const uint16_t progress_u = (static_cast<uint16_t>(progressHigh) << 8) | static_cast<uint16_t>(progressLow);

    line_rot = static_cast<int16_t>(lineRot_u);
    progress = static_cast<int16_t>(progress_u);

    if (progress >= 16) {
      line_rot += 180;
    }

    if (line_rot > 360) {
      line_rot -= 360;
    }
  }

  readButton();

  localToWorld(local_x, local_y, heading, _x_pos, _y_pos);
}

int Bot::readCompass() {
  constexpr uint8_t Angle = 1;
  I2C::transmit(imuAddress, &Angle, 1);
  I2C::requestData(imuAddress, 3);
  while (I2C::available() < 3)
    ;
  unsigned char angle8 = I2C::read();  // Read back the 5 bytes
  const unsigned char high_byte = I2C::read();
  const unsigned char low_byte = I2C::read();
  unsigned int angle16 = high_byte;  // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;
  const unsigned int precalc = angle16 / 10;

  return ((((precalc - _head_calib) + 180 + 360) % 360) - 180);
}

void Bot::set_heading() {
  _head_calib = readCompass();
}

void Bot::localToWorld(const float lx, const float ly,const float heading_deg, float &gx, float &gy) {
  const float theta = heading_deg * (PI / 180.0f);
  gx = cosf(theta) * lx - sinf(theta) * ly;
  gy = sinf(theta) * lx + cosf(theta) * ly;
}

void Bot::readButton() {
  if (digitalRead(buttonPIN) == HIGH) {
    _ena = !_ena;
  }
  while (digitalRead(buttonPIN) == HIGH)
    ;
}

void Bot::pushData(const bool enable, const bool kick, int vx, int vy, int rot, int dribbler) {
  MotorCmd cmd{};

  vx *= -1;
  vy *= -1;

  vx = constrain(vx, -100, 100);
  vy = constrain(vy, -100, 100);
  rot = constrain(rot, -100, 100);
  dribbler = constrain(dribbler, -100, 100);

  cmd.flags = 0;
  if (enable) cmd.flags |= 0x01;
  if (kick)   cmd.flags |= 0x02;

  cmd.vx   = static_cast<int8_t>(vx);
  cmd.vy   = static_cast<int8_t>(vy);
  cmd.rot  = static_cast<int8_t>(rot);
  cmd.drib = static_cast<int8_t>(dribbler);

  I2C::transmit(motorDriverMB, reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}

void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}

void Bot::home() {
  getSensorData();

  constexpr float TARGET_X = -50.0f;
  constexpr float TARGET_Y = -90.0f;
  constexpr float KP_POS = 1.5f;
  constexpr float KP_ROT = 0.8f;
  constexpr float MAX_SPEED = 40.0f;
  constexpr float MAX_ROT_SPEED = 50.0f;

  const float error_x = TARGET_X - _x_pos;
  const float error_y = TARGET_Y - _y_pos;
  const float distance = sqrtf(error_x * error_x + error_y * error_y);

  if (constexpr float GOAL_RADIUS = 3.0f; distance < GOAL_RADIUS) {
    isHoming = false;
    _ena = 0;
    pushData(_ena, false, 0, 0, 0, 0);
    return;
  }

  float world_vx = error_x * KP_POS;
  float world_vy = error_y * KP_POS;

  if (const float speed = sqrtf(world_vx * world_vx + world_vy * world_vy); speed > MAX_SPEED) {
    world_vx = (world_vx / speed) * MAX_SPEED;
    world_vy = (world_vy / speed) * MAX_SPEED;
  }

  const float theta = _head * (PI / 180.0f);
  const float cos_theta = cosf(theta);
  const float sin_theta = sinf(theta);

  const float local_vx = cos_theta * world_vx + sin_theta * world_vy;
  const float local_vy = -sin_theta * world_vx + cos_theta * world_vy;

  float rot_speed = -_head * KP_ROT;
  rot_speed = constrain(rot_speed, -MAX_ROT_SPEED, MAX_ROT_SPEED);

  pushData(true, false, static_cast<int>(local_vx), static_cast<int>(local_vy), static_cast<int>(rot_speed), 0);
}
