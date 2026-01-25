//
// Created for wireless vector debugging
//

#include <util/WebDebugger.h>

WebDebugger webDebugger;

// HTML page with vector visualization
const char WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Robot Vector Debugger</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: 'Segoe UI', Arial, sans-serif;
      background: #1a1a2e;
      color: #eee;
      padding: 10px;
    }
    .container {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      max-width: 1200px;
      margin: 0 auto;
    }
    .panel {
      background: #16213e;
      border-radius: 8px;
      padding: 15px;
    }
    h1 {
      text-align: center;
      color: #e94560;
      margin-bottom: 15px;
      font-size: 1.5em;
    }
    h2 {
      color: #0f3460;
      background: #e94560;
      padding: 5px 10px;
      border-radius: 4px;
      margin-bottom: 10px;
      font-size: 0.9em;
    }
    canvas {
      background: #0f0f23;
      border-radius: 8px;
      width: 100%;
      aspect-ratio: 1;
    }
    .data-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 5px;
    }
    .data-item {
      background: #0f3460;
      padding: 8px;
      border-radius: 4px;
      font-size: 0.85em;
    }
    .label { color: #aaa; font-size: 0.75em; }
    .value { font-weight: bold; color: #4ecca3; font-family: monospace; }
    .status {
      text-align: center;
      padding: 10px;
      border-radius: 4px;
      margin-bottom: 10px;
    }
    .connected { background: #4ecca3; color: #000; }
    .disconnected { background: #e94560; }
    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-top: 10px;
      font-size: 0.8em;
    }
    .legend-item { display: flex; align-items: center; gap: 5px; }
    .legend-color { width: 20px; height: 3px; }
    .state-indicator {
      display: inline-block;
      width: 12px;
      height: 12px;
      border-radius: 50%;
      margin-right: 5px;
    }
    .state-on { background: #4ecca3; }
    .state-off { background: #555; }
    @media (max-width: 768px) {
      .container { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <h1>🤖 Robot Vector Debugger</h1>
  <div id="status" class="status disconnected">Disconnected</div>

  <div class="container">
    <div class="panel">
      <h2>📐 Vector Visualization</h2>
      <canvas id="vectorCanvas" width="400" height="400"></canvas>
      <div class="legend">
        <div class="legend-item"><div class="legend-color" style="background:#ff6b6b"></div>Target</div>
        <div class="legend-item"><div class="legend-color" style="background:#ffd93d"></div>Ball</div>
        <div class="legend-item"><div class="legend-color" style="background:#6bcb77"></div>Goal</div>
        <div class="legend-item"><div class="legend-color" style="background:#4d96ff"></div>MiddlePoint</div>
        <div class="legend-item"><div class="legend-color" style="background:#ff9f1c"></div>Line</div>
      </div>
    </div>

    <div class="panel">
      <h2>📊 Sensor Data</h2>
      <div class="data-grid">
        <div class="data-item">
          <div class="label">Heading</div>
          <div class="value" id="heading">--</div>
        </div>
        <div class="data-item">
          <div class="label">Global X</div>
          <div class="value" id="globalX">--</div>
        </div>
        <div class="data-item">
          <div class="label">Global Y</div>
          <div class="value" id="globalY">--</div>
        </div>
        <div class="data-item">
          <div class="label">Ball Rot</div>
          <div class="value" id="ballRot">--</div>
        </div>
        <div class="data-item">
          <div class="label">Ball Dist</div>
          <div class="value" id="ballDist">--</div>
        </div>
        <div class="data-item">
          <div class="label">Yellow Rot</div>
          <div class="value" id="yellowRot">--</div>
        </div>
        <div class="data-item">
          <div class="label">Yellow Dist</div>
          <div class="value" id="yellowDist">--</div>
        </div>
        <div class="data-item">
          <div class="label">Blue Rot</div>
          <div class="value" id="blueRot">--</div>
        </div>
        <div class="data-item">
          <div class="label">Blue Dist</div>
          <div class="value" id="blueDist">--</div>
        </div>
      </div>

      <h2 style="margin-top:15px">🎯 PID Output</h2>
      <div class="data-grid">
        <div class="data-item">
          <div class="label">X Output</div>
          <div class="value" id="xOutput">--</div>
        </div>
        <div class="data-item">
          <div class="label">Y Output</div>
          <div class="value" id="yOutput">--</div>
        </div>
        <div class="data-item">
          <div class="label">Rot Output</div>
          <div class="value" id="rotOutput">--</div>
        </div>
      </div>

      <h2 style="margin-top:15px">🔘 States</h2>
      <div class="data-grid">
        <div class="data-item">
          <span class="state-indicator" id="hasBallInd"></span>Has Ball
        </div>
        <div class="data-item">
          <span class="state-indicator" id="lineSeenInd"></span>Line Seen
        </div>
        <div class="data-item">
          <span class="state-indicator" id="ballExistsInd"></span>Ball Exists
        </div>
      </div>

      <h2 style="margin-top:15px">🧭 Vectors (X, Y)</h2>
      <div class="data-grid">
        <div class="data-item">
          <div class="label">Target</div>
          <div class="value" id="targetVec">--</div>
        </div>
        <div class="data-item">
          <div class="label">Ball</div>
          <div class="value" id="ballVec">--</div>
        </div>
        <div class="data-item">
          <div class="label">Goal</div>
          <div class="value" id="goalVec">--</div>
        </div>
        <div class="data-item">
          <div class="label">MiddlePoint</div>
          <div class="value" id="middleVec">--</div>
        </div>
      </div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('vectorCanvas');
    const ctx = canvas.getContext('2d');
    let data = null;

    function connect() {
      const ws = new WebSocket('ws://' + location.host + '/ws');

      ws.onopen = () => {
        document.getElementById('status').className = 'status connected';
        document.getElementById('status').textContent = 'Connected';
      };

      ws.onclose = () => {
        document.getElementById('status').className = 'status disconnected';
        document.getElementById('status').textContent = 'Disconnected - Reconnecting...';
        setTimeout(connect, 1000);
      };

      ws.onmessage = (event) => {
        try {
          data = JSON.parse(event.data);
          updateDisplay();
          drawVectors();
        } catch(e) { console.error(e); }
      };
    }

    function updateDisplay() {
      if (!data) return;

      document.getElementById('heading').textContent = data.heading.toFixed(1) + '°';
      document.getElementById('globalX').textContent = data.globalX.toFixed(1);
      document.getElementById('globalY').textContent = data.globalY.toFixed(1);
      document.getElementById('ballRot').textContent = data.ballRot.toFixed(1) + '°';
      document.getElementById('ballDist').textContent = data.ballDist.toFixed(1);
      document.getElementById('yellowRot').textContent = data.yellowRot.toFixed(1) + '°';
      document.getElementById('yellowDist').textContent = data.yellowDist.toFixed(1);
      document.getElementById('blueRot').textContent = data.blueRot.toFixed(1) + '°';
      document.getElementById('blueDist').textContent = data.blueDist.toFixed(1);

      document.getElementById('xOutput').textContent = data.xOutput.toFixed(2);
      document.getElementById('yOutput').textContent = data.yOutput.toFixed(2);
      document.getElementById('rotOutput').textContent = data.rotOutput.toFixed(2);

      document.getElementById('hasBallInd').className = 'state-indicator ' + (data.hasBall ? 'state-on' : 'state-off');
      document.getElementById('lineSeenInd').className = 'state-indicator ' + (data.lineSeen ? 'state-on' : 'state-off');
      document.getElementById('ballExistsInd').className = 'state-indicator ' + (data.ballExists ? 'state-on' : 'state-off');

      document.getElementById('targetVec').textContent = `(${data.target.x.toFixed(1)}, ${data.target.y.toFixed(1)})`;
      document.getElementById('ballVec').textContent = `(${data.ballVec.x.toFixed(1)}, ${data.ballVec.y.toFixed(1)})`;
      document.getElementById('goalVec').textContent = `(${data.goalVec.x.toFixed(1)}, ${data.goalVec.y.toFixed(1)})`;
      document.getElementById('middleVec').textContent = `(${data.middlePointVec.x.toFixed(1)}, ${data.middlePointVec.y.toFixed(1)})`;
    }

    function drawVectors() {
      if (!data) return;

      const w = canvas.width;
      const h = canvas.height;
      const cx = w / 2;
      const cy = h / 2;

      // Find max magnitude to auto-scale
      const magnitudes = [
        Math.sqrt(data.target.x**2 + data.target.y**2),
        Math.sqrt(data.ballVec.x**2 + data.ballVec.y**2),
        Math.sqrt(data.goalVec.x**2 + data.goalVec.y**2),
        Math.sqrt(data.middlePointVec.x**2 + data.middlePointVec.y**2),
        Math.sqrt(data.lineVec.x**2 + data.lineVec.y**2)
      ];
      const maxMag = Math.max(...magnitudes, 50); // minimum 50 to avoid division issues
      const scale = (Math.min(w, h) / 2 - 20) / maxMag; // fit within canvas with margin

      ctx.fillStyle = '#0f0f23';
      ctx.fillRect(0, 0, w, h);

      // Grid - dynamic based on scale
      ctx.strokeStyle = '#333';
      ctx.lineWidth = 1;
      const gridStep = maxMag > 100 ? 50 : 20;
      for (let i = -200; i <= 200; i += gridStep) {
        ctx.beginPath();
        ctx.moveTo(cx + i * scale, 0);
        ctx.lineTo(cx + i * scale, h);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(0, cy - i * scale);
        ctx.lineTo(w, cy - i * scale);
        ctx.stroke();
      }

      // Axes
      ctx.strokeStyle = '#555';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, cy);
      ctx.lineTo(w, cy);
      ctx.moveTo(cx, 0);
      ctx.lineTo(cx, h);
      ctx.stroke();

      // Axis labels
      ctx.fillStyle = '#888';
      ctx.font = '12px monospace';
      ctx.fillText('+X', w - 25, cy - 5);
      ctx.fillText('+Y', cx + 5, 15);

      // Robot
      ctx.fillStyle = '#fff';
      ctx.beginPath();
      ctx.arc(cx, cy, 8, 0, Math.PI * 2);
      ctx.fill();

      // Draw vectors
      drawArrow(cx, cy, data.target.x, data.target.y, '#ff6b6b', scale, 3);
      drawArrow(cx, cy, data.ballVec.x, data.ballVec.y, '#ffd93d', scale, 2);
      drawArrow(cx, cy, data.goalVec.x, data.goalVec.y, '#6bcb77', scale, 2);
      drawArrow(cx, cy, data.middlePointVec.x, data.middlePointVec.y, '#4d96ff', scale, 2);
      drawArrow(cx, cy, data.lineVec.x, data.lineVec.y, '#ff9f1c', scale, 2);

      // Scale indicator
      ctx.fillStyle = '#666';
      ctx.font = '10px monospace';
      ctx.fillText('Max: ' + maxMag.toFixed(0) + ' cm', 10, h - 10);
    }

    function drawArrow(cx, cy, vx, vy, color, scale, width) {
      if (Math.abs(vx) < 0.1 && Math.abs(vy) < 0.1) return;

      const ex = cx + vx * scale;
      const ey = cy - vy * scale; // Y inverted for canvas

      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = width;

      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(ex, ey);
      ctx.stroke();

      // Arrowhead
      const angle = Math.atan2(cy - ey, ex - cx);
      const headLen = 10;
      ctx.beginPath();
      ctx.moveTo(ex, ey);
      ctx.lineTo(ex - headLen * Math.cos(angle - 0.4), ey + headLen * Math.sin(angle - 0.4));
      ctx.lineTo(ex - headLen * Math.cos(angle + 0.4), ey + headLen * Math.sin(angle + 0.4));
      ctx.closePath();
      ctx.fill();
    }

    // Resize canvas
    function resizeCanvas() {
      const rect = canvas.getBoundingClientRect();
      canvas.width = rect.width;
      canvas.height = rect.width;
      drawVectors();
    }

    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();
    connect();
  </script>
</body>
</html>
)rawliteral";

WebDebugger::WebDebugger() : _server(80), _ws("/ws"), _clientConnected(false), _lastSend(0) {
  memset(&_data, 0, sizeof(_data));
}

void WebDebugger::setupServer() {
  // Serve the webpage
  _server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", WEBPAGE);
  });

  // WebSocket event handler
  _ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client,
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
      case WS_EVT_CONNECT:
        Serial.printf("[WS] Client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        _clientConnected = true;
        break;
      case WS_EVT_DISCONNECT:
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
        _clientConnected = _ws.count() > 1; // >1 weil dieser noch gezählt wird
        break;
      case WS_EVT_ERROR:
        Serial.printf("[WS] Client #%u error: %s\n", client->id(), (char*)arg);
        break;
      case WS_EVT_PONG:
        break;
      case WS_EVT_DATA:
        // Eingehende Daten ignorieren
        break;
    }
  });

  _server.addHandler(&_ws);
  _server.begin();
  Serial.println("WebServer started");
}

void WebDebugger::begin(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    _ip = WiFi.localIP().toString();
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(_ip);
    Serial.print("Open http://");
    Serial.print(_ip);
    Serial.println(" in your browser");
  } else {
    Serial.println("\nWiFi connection failed, starting AP mode...");
    beginAP("RobotDebugger");
    return;
  }

  setupServer();
}

void WebDebugger::beginAP(const char* ssid, const char* password) {
  WiFiClass::mode(WIFI_AP);

  if (password && strlen(password) >= 8) {
    WiFi.softAP(ssid, password);
  } else {
    WiFi.softAP(ssid);
  }

  _ip = WiFi.softAPIP().toString();
  Serial.println("AP Mode started!");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(_ip);
  Serial.print("Open http://");
  Serial.print(_ip);
  Serial.println(" in your browser");

  setupServer();
}

void WebDebugger::update() {
  // Nur alle 5 Sekunden cleanup, nicht bei jedem Update
  static unsigned long lastCleanup = 0;
  if (millis() - lastCleanup > 5000) {
    _ws.cleanupClients();
    lastCleanup = millis();
  }

  _clientConnected = _ws.count() > 0;

  if (const unsigned long now = millis(); _clientConnected && (now - _lastSend >= SEND_INTERVAL)) {
    // Nur senden wenn die Queue nicht voll ist
    if (_ws.availableForWriteAll()) {
      _lastSend = now;
      sendDebugData();
    }
  }
}

void WebDebugger::setData(const DebugData& data) {
  _data = data;
}

// Helper to safely convert float to string (handles NaN/Inf)
static String safeFloat(float val, int decimals = 2) {
  if (isnan(val) || isinf(val)) {
    return "0";
  }
  return String(val, decimals);
}

static String safeDouble(double val, int decimals = 2) {
  if (isnan(val) || isinf(val)) {
    return "0";
  }
  return String(val, decimals);
}

String WebDebugger::buildJSON() {
  String json = "{";

  // Vectors
  json += "\"target\":{\"x\":" + safeDouble(_data.target.getX()) + ",\"y\":" + safeDouble(_data.target.getY()) + "},";
  json += "\"ballVec\":{\"x\":" + safeDouble(_data.ballVec.getX()) + ",\"y\":" + safeDouble(_data.ballVec.getY()) + "},";
  json += "\"goalVec\":{\"x\":" + safeDouble(_data.goalVec.getX()) + ",\"y\":" + safeDouble(_data.goalVec.getY()) + "},";
  json += "\"middlePointVec\":{\"x\":" + safeDouble(_data.middlePointVec.getX()) + ",\"y\":" + safeDouble(_data.middlePointVec.getY()) + "},";
  json += "\"lineVec\":{\"x\":" + safeDouble(_data.lineVec.getX()) + ",\"y\":" + safeDouble(_data.lineVec.getY()) + "},";

  // Scalars
  json += "\"heading\":" + safeFloat(_data.heading, 1) + ",";
  json += "\"ballRot\":" + safeFloat(_data.ballRot, 1) + ",";
  json += "\"ballDist\":" + safeFloat(_data.ballDist, 1) + ",";
  json += "\"yellowRot\":" + safeFloat(_data.yellowRot, 1) + ",";
  json += "\"yellowDist\":" + safeFloat(_data.yellowDist, 1) + ",";
  json += "\"blueRot\":" + safeFloat(_data.blueRot, 1) + ",";
  json += "\"blueDist\":" + safeFloat(_data.blueDist, 1) + ",";
  json += "\"globalX\":" + safeFloat(_data.globalX, 1) + ",";
  json += "\"globalY\":" + safeFloat(_data.globalY, 1) + ",";

  // PID outputs
  json += "\"xOutput\":" + safeFloat(_data.xOutput) + ",";
  json += "\"yOutput\":" + safeFloat(_data.yOutput) + ",";
  json += "\"rotOutput\":" + safeFloat(_data.rotOutput) + ",";

  // States
  json += "\"hasBall\":" + String(_data.hasBall ? "true" : "false") + ",";
  json += "\"lineSeen\":" + String(_data.lineSeen ? "true" : "false") + ",";
  json += "\"ballExists\":" + String(_data.ballExists ? "true" : "false");

  json += "}";
  return json;
}

void WebDebugger::sendDebugData() {
  if (_ws.count() == 0) return;

  String json = buildJSON();

  // Debug: Print first send
  static bool firstSend = true;
  if (firstSend) {
    Serial.println("[WS] First JSON send:");
    Serial.println(json);
    firstSend = false;
  }

  _ws.textAll(json);
}

