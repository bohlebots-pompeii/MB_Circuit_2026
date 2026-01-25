//
// Created for wireless vector debugging
//

#ifndef BOHLEBOTS_2026_WEBDEBUGGER_H
#define BOHLEBOTS_2026_WEBDEBUGGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Vector2.hpp>

struct DebugData {
  // Vectors
  Vector2 target;
  Vector2 ballVec;
  Vector2 goalVec;
  Vector2 middlePointVec;
  Vector2 lineVec;

  // Scalars
  float heading;
  float ballRot;
  float ballDist;
  float yellowRot;
  float yellowDist;
  float blueRot;
  float blueDist;
  float globalX;
  float globalY;

  // State
  bool hasBall;
  bool lineSeen;
  bool ballExists;

  // PID outputs
  float xOutput;
  float yOutput;
  float rotOutput;
};

class WebDebugger {
public:
  WebDebugger();

  void begin(const char* ssid, const char* password);
  void beginAP(const char* ssid, const char* password = nullptr);
  void update();

  void setData(const DebugData& data);

  [[nodiscard]] bool isConnected() const { return _clientConnected; }
  [[nodiscard]] String getIP() const { return _ip; }

private:
  void setupServer();
  void sendDebugData();
  String buildJSON();

  AsyncWebServer _server;
  AsyncWebSocket _ws;
  DebugData _data;
  bool _clientConnected;
  String _ip;
  unsigned long _lastSend;
  static constexpr unsigned long SEND_INTERVAL = 100; // 10 Hz
};

extern WebDebugger webDebugger;

#endif //BOHLEBOTS_2026_WEBDEBUGGER_H

