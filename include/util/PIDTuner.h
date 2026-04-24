#pragma once
#include <WiFi.h>
#include <PID_v1.h>
#include <config/config.h>

// ── Port & timing ──────────────────────────────────────────────────────────
static constexpr uint16_t PIDTUNER_PORT    = 9000;
static constexpr uint32_t PIDTUNER_PUSH_MS = 50;

// ── Wire protocol (must match Python structs exactly) ──────────────────────
//
//  TX  ESP32 → Python   128 bytes
//      "<II" + "8f"*3 + "24x"
//      [0]      uint32   magic    = 0xDEADBEEF
//      [1]      uint32   timecode (millis)
//      [2..9]   8×float  x   : kp ki kd pOut iOut dOut pidIn pidOut
//      [10..17] 8×float  y   : same layout
//      [18..25] 8×float  yaw : same layout
//      [26..31] 24 bytes padding (zeros)
//
//  RX  Python → ESP32   56 bytes
//      "<IBfff47x"
//      [0]  uint32   magic   = 0xCAFEBABE
//      [4]  uint8    channel (0=x  1=y  2=yaw)
//      [5]  float    kp
//      [9]  float    ki
//      [13] float    kd
//      [17..55] 47 bytes padding (ignored)

static constexpr uint32_t MAGIC_TX = 0xDEADBEEF;
static constexpr uint32_t MAGIC_RX = 0xCAFEBABE;
static constexpr size_t   FRAME_TX = 128;
static constexpr size_t   FRAME_RX = 56;

// ── Helper: write little-endian primitives into a byte buffer ──────────────
namespace PIDTunerUtil {
  static void writeU32LE(uint8_t* b, uint32_t v) {
    b[0]=v&0xFF; b[1]=(v>>8)&0xFF; b[2]=(v>>16)&0xFF; b[3]=(v>>24)&0xFF;
  }
  static void writeF32LE(uint8_t* b, float v) {
    uint32_t tmp; memcpy(&tmp, &v, 4); writeU32LE(b, tmp);
  }
  static uint32_t readU32LE(const uint8_t* b) {
    return (uint32_t)b[0] | ((uint32_t)b[1]<<8) |
           ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24);
  }
  static float readF32LE(const uint8_t* b) {
    uint32_t tmp = readU32LE(b); float v; memcpy(&v, &tmp, 4); return v;
  }
}

// ── PIDTuner ───────────────────────────────────────────────────────────────
class PIDTuner {
public:

  // Call once from Bot::Bot() — same signature as before
  static void begin(PID* xPID,   double* xIn,   double* xOut,   double* xSet,
                    PID* yPID,   double* yIn,   double* yOut,   double* ySet,
                    PID* rotPID, double* rotIn, double* rotOut, double* rotSet,
                    const char* ssid     = "MB_PID_TUNER",
                    const char* password = "tuner1234") {

    _xPID=xPID; _xIn=xIn; _xOut=xOut; _xSet=xSet;
    _yPID=yPID; _yIn=yIn; _yOut=yOut; _ySet=ySet;
    _rPID=rotPID; _rIn=rotIn; _rOut=rotOut; _rSet=rotSet;

    // Load initial gains from config
    _gains[0][0]=PIDConfig::X_Kp;   _gains[0][1]=PIDConfig::X_Ki;   _gains[0][2]=PIDConfig::X_Kd;
    _gains[1][0]=PIDConfig::Y_Kp;   _gains[1][1]=PIDConfig::Y_Ki;   _gains[1][2]=PIDConfig::Y_Kd;
    _gains[2][0]=PIDConfig::Rot_Kp; _gains[2][1]=PIDConfig::Rot_Ki; _gains[2][2]=PIDConfig::Rot_Kd;

    WiFiClass::mode(WIFI_AP_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.begin(NetworkConfig::WIFI_SSID, NetworkConfig::WIFI_PASSWORD);

    const uint32_t connectStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - connectStart) < NetworkConfig::WIFI_CONNECT_TIMEOUT_MS) {
      delay(20);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[PIDTuner] STA connected SSID=%s IP=%s\n",
                    NetworkConfig::WIFI_SSID,
                    WiFi.localIP().toString().c_str());
    } else {
      Serial.printf("[PIDTuner] STA connect timeout for SSID=%s\n", NetworkConfig::WIFI_SSID);
    }

    _server = new WiFiServer(PIDTUNER_PORT);
    _server->begin();
    Serial.printf("[PIDTuner] TCP server on port %u  (binary protocol)\n", PIDTUNER_PORT);
  }

  // Call every loop iteration from Bot::tick()
  static void tick() {
    if (!_server) return;

    // Accept new connection if idle
    if (!_client || !_client.connected()) {
      _rxLen = 0;
      WiFiClient nc = _server->accept();
      if (nc) {
        _client = nc;
        _client.setNoDelay(true);
        Serial.println("[PIDTuner] Python client connected");
      }
    }
    if (!_client.connected()) return;

    // Drain incoming RX frames
    _readFrames();

    // Push state at configured rate
    uint32_t now = millis();
    if (now - _lastPush >= PIDTUNER_PUSH_MS) {
      _lastPush = now;
      _pushState();
    }
  }

private:
  // PID pointers
  static PID    *_xPID,  *_yPID,  *_rPID;
  static double *_xIn,   *_yIn,   *_rIn;
  static double *_xOut,  *_yOut,  *_rOut;
  static double *_xSet,  *_ySet,  *_rSet;

  // Gains indexed by channel: 0=x 1=y 2=yaw(rot)
  // [ch][0]=Kp [ch][1]=Ki [ch][2]=Kd
  static double _gains[3][3];

  static WiFiServer* _server;
  static WiFiClient  _client;
  static uint32_t    _lastPush;

  static uint8_t  _rxBuf[FRAME_RX * 4]; // buffer a few frames
  static size_t   _rxLen;

  // ── Read & process all complete RX frames in the socket buffer ───────────
  static void _readFrames() {
    // Drain available bytes into our reassembly buffer
    while (_client.available() && _rxLen < sizeof(_rxBuf)) {
      _rxBuf[_rxLen++] = (uint8_t)_client.read();
    }

    // Process every complete frame
    while (_rxLen >= FRAME_RX) {
      // Hunt for magic at offset 0; if wrong, skip one byte (resync)
      uint32_t magic = PIDTunerUtil::readU32LE(_rxBuf);
      if (magic != MAGIC_RX) {
        memmove(_rxBuf, _rxBuf + 1, --_rxLen);
        continue;
      }

      // We have a full valid frame
      uint8_t  ch = _rxBuf[4];
      float    kp = PIDTunerUtil::readF32LE(_rxBuf + 5);
      float    ki = PIDTunerUtil::readF32LE(_rxBuf + 9);
      float    kd = PIDTunerUtil::readF32LE(_rxBuf + 13);
      // bytes 17..55 are padding — ignored

      _applyGains(ch, kp, ki, kd);

      // Consume the frame
      _rxLen -= FRAME_RX;
      if (_rxLen) memmove(_rxBuf, _rxBuf + FRAME_RX, _rxLen);
    }
  }

  // ── Apply gains received from Python to the correct PID ──────────────────
  static void _applyGains(uint8_t ch, float kp, float ki, float kd) {
    if (ch > 2) return;

    if (kp >= 0.f) _gains[ch][0] = kp;
    if (ki >= 0.f) _gains[ch][1] = ki;
    if (kd >= 0.f) _gains[ch][2] = kd;

    PID* pid = (ch == 0) ? _xPID : (ch == 1) ? _yPID : _rPID;
    if (pid) pid->SetTunings(_gains[ch][0], _gains[ch][1], _gains[ch][2]);

    static const char* names[] = {"X", "Y", "Rot"};
    Serial.printf("[PIDTuner] %s  Kp=%.4f Ki=%.4f Kd=%.4f\n",
                  names[ch], _gains[ch][0], _gains[ch][1], _gains[ch][2]);
  }

  // ── Build & send one 128-byte TX frame to the Python client ──────────────
  static void _pushState() {
    uint8_t frame[FRAME_TX];
    memset(frame, 0, FRAME_TX);

    PIDTunerUtil::writeU32LE(frame + 0, MAGIC_TX);
    PIDTunerUtil::writeU32LE(frame + 4, (uint32_t)millis());

    // Channel data pointers: {in, out, p-term, i-term, d-term}
    // pidIn  = *_xIn  (the error / input to the PID)
    // pidOut = *_xOut (the PID output)
    // pOut/iOut/dOut are not directly exposed by PID_v1 — we approximate
    // from current output and stored gains. Replace with your own tracking
    // vars if your PID library exposes the individual terms.
    struct ChanInfo {
      double kp, ki, kd;
      double pidIn, pidOut;
    };

    ChanInfo ch[3] = {
      { _gains[0][0], _gains[0][1], _gains[0][2],
        _xIn ? *_xIn : 0.0, _xOut ? *_xOut : 0.0 },
      { _gains[1][0], _gains[1][1], _gains[1][2],
        _yIn ? *_yIn : 0.0, _yOut ? *_yOut : 0.0 },
      { _gains[2][0], _gains[2][1], _gains[2][2],
        _rIn ? *_rIn : 0.0, _rOut ? *_rOut : 0.0 },
    };

    for (int i = 0; i < 3; i++) {
      uint8_t* base = frame + 8 + i * 32; // 8 floats × 4 bytes = 32 bytes per channel
      PIDTunerUtil::writeF32LE(base +  0, (float)ch[i].kp);
      PIDTunerUtil::writeF32LE(base +  4, (float)ch[i].ki);
      PIDTunerUtil::writeF32LE(base +  8, (float)ch[i].kd);
      PIDTunerUtil::writeF32LE(base + 12, 0.f);   // pOut  — fill if available
      PIDTunerUtil::writeF32LE(base + 16, 0.f);   // iOut  — fill if available
      PIDTunerUtil::writeF32LE(base + 20, 0.f);   // dOut  — fill if available
      PIDTunerUtil::writeF32LE(base + 24, (float)ch[i].pidIn);
      PIDTunerUtil::writeF32LE(base + 28, (float)ch[i].pidOut);
    }
    // bytes 104..127 already zeroed (24-byte padding)

    if (_client.connected())
      _client.write(frame, FRAME_TX);
  }
};

// ── Static member definitions ──────────────────────────────────────────────
WiFiServer* PIDTuner::_server  = nullptr;
WiFiClient  PIDTuner::_client;
uint32_t    PIDTuner::_lastPush = 0;
size_t      PIDTuner::_rxLen   = 0;
uint8_t     PIDTuner::_rxBuf[FRAME_RX * 4];

double PIDTuner::_gains[3][3] = {};

PID*    PIDTuner::_xPID  = nullptr;
double* PIDTuner::_xIn   = nullptr;
double* PIDTuner::_xOut  = nullptr;
double* PIDTuner::_xSet  = nullptr;
PID*    PIDTuner::_yPID  = nullptr;
double* PIDTuner::_yIn   = nullptr;
double* PIDTuner::_yOut  = nullptr;
double* PIDTuner::_ySet  = nullptr;
PID*    PIDTuner::_rPID  = nullptr;
double* PIDTuner::_rIn   = nullptr;
double* PIDTuner::_rOut  = nullptr;
double* PIDTuner::_rSet  = nullptr;

