# MB\_Circuit\_2026 · ballDrive

Autonomous soccer robot firmware for the ESP32. Runs the full game loop — vision, positioning, team communication, and motor control — at roughly 1 kHz.

---

## What it does

A CM5 coprocessor (connected over Serial2) handles the heavy lifting: object detection, goal tracking, and global position estimation via overhead mirror. This firmware receives that data, builds a clean world-state snapshot every tick, and picks a behaviour node to execute.

Two robots talk to each other over ESP-NOW. When both are alive, they negotiate roles (striker / goalie) dynamically based on who's closer to the ball. When one goes down, the other adapts.

---

## Hardware

- **MCU:** ESP32 (standard dev board)
- **Vision:** CM5 coprocessor — Serial2 at 115200 baud, pins 16/17
- **Motors + dribbler:** Custom motor board at I²C address `0x69`
- **Line sensor:** I²C at `0x40` — returns progress and angle as two 16-bit signed ints
- **Button/LED modules:** I²C addresses `0x20–0x27` (BohleBots standard)
- **Light gate:** Analog pin 39 — ball detection threshold at 3900
- **Comms module:** Digital pin 36 — LOW = force halt

---

## Project structure

```
include/
  config/config.h         — all tuning constants (PID, field dimensions, pins)
  comms/CM5.h             — vision coprocessor interface
  comms/esp-now.h         — peer robot communication
  util/Vector2.hpp        — 2D vector math (BohleBots library)
  util/MovingAverage.h    — templated rolling average
  nodes/                  — one header per behaviour

src/
  main.cpp                — setup + loop
  Bot.cpp                 — main decision loop, role switching
  WorldState.cpp          — builds a flat snapshot each tick
  Sensors.cpp             — I²C sensor polling
  Positioning.cpp         — velocity estimation, field boundary enforcement
  MotionController.cpp    — PID wrappers for X/Y/rotation
  GameStateHandler.cpp    — setup state machine (target → role → running)
  motor_mb.cpp            — motor command buffering and I²C send
  comms/CM5.cpp           — serial parser + geometry
  comms/esp-now.cpp       — ESP-NOW init, send/receive, peer lifecycle
  nodes/                  — one .cpp per behaviour
```

---

## Getting started

### Dependencies

Everything is managed by PlatformIO:

```
pfeerick/elapsedMillis @ ^1.0.6
br3ttb/PID             @ ^1.2.1
wire                   (bundled with ESP32 Arduino core)
```

Platform: `espressif32` via Jason2866's fork (IDF 5.3 / Arduino).

### Build

```bash
pio run
```

### Upload

```bash
pio run --target upload
```

Monitor at 115200 baud:

```bash
pio device monitor
```

### Required: configure MAC addresses

Copy the template and fill in both robots' MAC addresses before building:

```bash
cp include/config/config_esp_now.h.example include/config/config_esp_now.h
```

Then edit `config_esp_now.h`:

```cpp
namespace EspNowConfig {
  constexpr uint8_t MAC_ROBOT_A[6] = { 0xAA, 0xBB, ... };
  constexpr uint8_t MAC_ROBOT_B[6] = { 0xCC, 0xDD, ... };
  constexpr uint32_t TX_INTERVAL_MS = 50;
}
```

`config_esp_now.h` is gitignored — keep MACs out of version control.

---

## Configuration

All tuning lives in `include/config/config.h`. Key sections:

| Namespace | What it controls |
|---|---|
| `GeneralConfig` | Ball valid time, heading limits, communication toggle, bot diameter |
| `FieldConfig` | Field dimensions, goal positions, kick distance, boundary polygon |
| `PIDConfig` | Striker PID gains and sample times |
| `GoaliePIDConfig` | Goalie PID gains (separate tuning) |
| `Goalie` | Half-circle radius, ball avoidance distance, stationary timers |
| `PINS` | All GPIO pin assignments |
| `I2C_ADDRESSES` | All I²C peripheral addresses |

---

## Behaviour nodes

The decision loop in `Bot::decideAndExecute` runs top-to-bottom priority. First match wins.

**Shared (both roles)**
- `LineEscape` — if line sensor fires, escape toward center

**Striker**
- `HiddenBallNPocket` / `DribbleToGoal` — has ball, goal in pocket
- `GetBehindBall` — ball visible, no possession
- `HoldNeutral` — ball briefly lost (< 500 ms)
- `DriveToNeutral` — ball lost > 500 ms

**Goalie**
- `EmergencyPosition` — own ball not visible but peer sees it
- `HalfCircleGuard` — ball visible, tracking on arc in front of goal
- `GoalNeutral` — default rest position

Kick logic runs separately in `decideKickAndExecute` — fires when the ball is in possession, the goal is within range, and the heading window is satisfied.

---

## Role switching

With `GeneralConfig::USE_COMMUNICATION = true`, the two robots negotiate who plays striker. The logic is in `Bot::getSwitchWanted`:

- Always want striker if the peer is dead or not yet running
- Immediately want striker when we have the ball
- After a 2-second cooldown, want striker if we are meaningfully closer to the ball *and* have a better angle than the peer

Whichever robot signals `switchWanted` becomes striker. The peer, upon receiving that signal, drops to goalie. There is a 500 ms grace period at game start to absorb imprecise button presses.

Set `USE_COMMUNICATION = false` for single-robot testing. Switch logic and emergency positioning both short-circuit cleanly at compile time.

---

## Game state setup sequence

On boot the robot starts in `TARGET_SELECT`. The physical flow:

```
TARGET_SELECT
  right button → toggle blue/yellow target goal
  left button  → advance

LOCKED
  selected colour blinks 2× then auto-advances

ROLE_SELECT
  left button  → toggle striker/goalie
  right button → start (saves to EEPROM)

RUNNING
  either button → stop, clear EEPROM, return to ROLE_SELECT
```

EEPROM state is restored on power-up, so the robot resumes its last configuration after a brief power cut.

---

## Coordinate system

Follows BohleBots convention (from `Vector2.hpp`):

```
        ^ x  (forward)
        |
        |
--------+--------> y  (right)
```

Angles are measured from the x-axis. Positive = clockwise when viewed from above. `heading` from the CM5 is in this convention. Field X runs along the long axis (goal to goal).