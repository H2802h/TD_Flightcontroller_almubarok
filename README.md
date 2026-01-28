# TD Flight Controller - Almubarok Edition

[![Platform](https://img.shields.io/badge/platform-Teensy%204.0-orange.svg)](https://www.pjrc.com/store/teensy40.html)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-active%20development-green.svg)]()

Advanced quadcopter flight controller based on Teensy 4.0 with GPS loiter, altitude hold, RTL, and real-time telemetry system.

---

## 📋 Table of Contents

- [System Overview](#system-overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Software Architecture](#software-architecture)
- [Pin Configuration](#pin-configuration)
- [Flight Modes](#flight-modes)
- [Installation Guide](#installation-guide)
- [Calibration Procedures](#calibration-procedures)
- [PID Tuning](#pid-tuning)
- [Telemetry System](#telemetry-system)
- [Ground Control Station](#ground-control-station)
- [Troubleshooting](#troubleshooting)
- [Safety Guidelines](#safety-guidelines)

---

## 🎯 System Overview

The TD Flight Controller is a high-performance quadcopter control system featuring:

- **Microcontroller**: Teensy 4.0 (600 MHz ARM Cortex-M7)
- **Control Loop**: 250 Hz (4000μs period)
- **Sensor Fusion**: Madgwick AHRS algorithm with magnetometer
- **Altitude Estimation**: 2D Kalman filter (altitude + vertical velocity)
- **GPS Navigation**: Loiter and RTL capabilities
- **Communication**: Dual serial telemetry (433MHz + 2.4GHz)

### System Capabilities

- Stabilize mode for manual flight
- GPS Loiter for position hold
- Altitude Hold using barometer
- Return to Launch (RTL)
- Real-time telemetry streaming
- Ground station integration
- Remote PID tuning

---

## ✨ Features

### Flight Control
- ✅ **Cascade PID Control**: Angle → Rate control loops
- ✅ **Sensor Fusion**: Madgwick algorithm with 9-DOF IMU
- ✅ **Altitude Hold**: Kalman-filtered barometer with vertical velocity
- ✅ **GPS Loiter**: Position hold using GPS coordinates
- ✅ **RTL Mode**: Autonomous return to home point
- ✅ **Failsafe**: Automatic mode switching on signal loss

### Sensor Integration
- ✅ **MPU9250**: 9-axis IMU (gyro + accel + mag)
- ✅ **BMP280**: Barometric pressure sensor
- ✅ **GPS Module**: Position and velocity data
- ✅ **PPM Receiver**: Multi-channel radio input

### Communication
- ✅ **433 MHz Telemetry**: Long-range data link
- ✅ **2.4 GHz RC**: Standard radio control
- ✅ **JSON Protocol**: Structured telemetry data
- ✅ **Live PID Tuning**: Real-time parameter adjustment

---

## 🏗️ System Architecture

### Block Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    GROUND CONTROL STATION                       │
│  ┌──────────────┐    ┌───────┐    ┌──────────────────────┐    │
│  │ GCS PC +     │◄───┤ WiFi  │───►│ Telemetry 433.3 MHz  │    │
│  │ Laptop       │    └───────┘    └──────────────────────┘    │
│  └──────────────┘                                               │
│                                                                  │
│  ┌──────────────┐                                               │
│  │ 2.4 GHz RC   │                                               │
│  │ Transmitter  │                                               │
│  └──────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
                               │
                               │ 433 MHz / 2.4 GHz
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                          WAHANA (DRONE)                          │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │              FLIGHT CONTROLLER (Teensy 4.0)                 │ │
│  │                                                              │ │
│  │  Inputs:                    Processing:                     │ │
│  │  • MPU9250 (IMU)            • Sensor Fusion (Madgwick)      │ │
│  │  • GPS Module               • Kalman Filter (Altitude)      │ │
│  │  • Barometer (BMP280)       • PID Controllers               │ │
│  │  • 2.4GHz Receiver          • Flight Mode Logic             │ │
│  │  • LED Indicators           • Motor Mixing                  │ │
│  │                                                              │ │
│  │  Outputs:                                                    │ │
│  │  • 4x ESC PWM Signals                                       │ │
│  │  • Telemetry (433.3 MHz)                                    │ │
│  │  • Status LEDs                                              │ │
│  └────────────────────────────────────────────────────────────┘ │
│                               │                                  │
│        ┌──────────────────────┼──────────────────────┐          │
│        ▼                      ▼                       ▼          │
│    ┌────────┐           ┌────────┐              ┌────────┐     │
│    │ ESC 1  │──Motor 1  │ ESC 2  │──Motor 2     │ ESC 3  │     │
│    └────────┘           └────────┘              └────────┘     │
│        ▼                                             ▼          │
│    ┌────────┐                                   ┌────────┐     │
│    │ ESC 4  │──Motor 4                          │Power   │     │
│    └────────┘                                   │Distrib │     │
│                                                  └────────┘     │
│                                                      ▲          │
│                                              LiPo Battery 4S    │
└─────────────────────────────────────────────────────────────────┘
                               │
                               │
                               ▼
                    ┌─────────────────────┐
                    │   Mini PC/          │
                    │   Raspberry Pi      │
                    │   + Camera          │
                    └─────────────────────┘
```

---

## 🔧 Hardware Components

### Core Electronics

| Component | Model | Purpose | Interface |
|-----------|-------|---------|-----------|
| **Flight Controller** | Teensy 4.0 | Main processor | - |
| **IMU** | MPU9250 | Attitude sensing | I2C (0x68) |
| **Magnetometer** | AK8963 (in MPU9250) | Heading reference | I2C (0x0C) |
| **Barometer** | BMP280 | Altitude sensing | I2C (0x76) |
| **GPS Module** | NEO-6M/M8N | Position/velocity | UART |
| **RC Receiver** | PPM/PWM | Radio control | Pin 10 |
| **Telemetry Radio** | 433 MHz | Long-range comms | Serial4 |
| **ESCs** | 30A (4x) | Motor control | PWM (Pins 2-5) |
| **Motors** | Brushless (4x) | Propulsion | - |
| **Battery** | LiPo 4S | Power | - |

### Frame Configuration

```
Motor Layout (X-Configuration):

        FRONT
         (3)
          ↑
    (2) ← + → (4)
          ↓
         (1)

Motor Rotation:
- Motor 1 (Front): CCW
- Motor 2 (Left):  CCW  
- Motor 3 (Right): CW
- Motor 4 (Back):  CW
```

### Power Distribution

```
LiPo 4S (14.8V) ──► Power Distribution Board ──┬──► ESC 1-4 ──► Motors
                                                 │
                                                 ├──► 5V BEC ──► FC + Sensors
                                                 │
                                                 └──► 5V BEC ──► Receiver
```

---

## 💻 Software Architecture

### Control Loop Structure (250 Hz)

```
┌─────────────────────────────────────────────────────────────┐
│                     Main Loop (4ms period)                   │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  1. Sensor Reading                                           │
│     ├─ Read MPU9250 (Gyro + Accel + Mag)                    │
│     ├─ Read BMP280 (Pressure)                               │
│     └─ Parse GPS Data                                        │
│                                                               │
│  2. Sensor Fusion & State Estimation                         │
│     ├─ Madgwick AHRS (Roll, Pitch, Yaw)                     │
│     ├─ Kalman Filter (Altitude + Velocity)                  │
│     └─ Pressure Baseline Update                             │
│                                                               │
│  3. Input Processing                                         │
│     ├─ Read RC Receiver (PPM)                               │
│     ├─ Handle Serial Commands                               │
│     └─ Check Arming Status                                   │
│                                                               │
│  4. Flight Mode Logic                                        │
│     ├─ Mode Selection (Stabilize/Loiter/Alt Hold/RTL)      │
│     └─ Mode-Specific Processing                             │
│                                                               │
│  5. Control Cascade                                          │
│     ├─ Position PID (GPS) ──► Angle Setpoints              │
│     ├─ Altitude PID ──► Throttle Correction                │
│     ├─ Angle PID ──► Rate Setpoints                        │
│     └─ Rate PID ──► Control Outputs                        │
│                                                               │
│  6. Motor Mixing & Output                                    │
│     ├─ X-Configuration Mixing                               │
│     ├─ Constraint Application                               │
│     └─ PWM Signal Generation                                │
│                                                               │
│  7. Telemetry & Communication                                │
│     └─ Send Status @ 200ms intervals                        │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### Sensor Fusion Pipeline

```
MPU9250                         Madgwick AHRS
┌──────────┐                    ┌────────────┐
│ Gyro X,Y,Z│──┐               │            │
│ Accel X,Y,Z│──┼──────────────►│ Quaternion │──► Roll, Pitch, Yaw
│ Mag X,Y,Z │──┘ (calibrated)   │  Update    │    (angles)
└──────────┘                    └────────────┘
                                     β = 0.3

BMP280                         Kalman Filter 2D
┌──────────┐                   ┌─────────────────┐
│ Pressure │──► Altitude ──────►│ State Vector:   │
└──────────┘    Calculation     │ [Altitude]      │──► Filtered
                                 │ [Velocity]      │    Altitude &
    AccZ (inertial)              │                 │    Velocity
         │                       │ Prediction +    │
         └──────────────────────►│ Correction      │
                                 └─────────────────┘
```

---

## 📌 Pin Configuration

### Teensy 4.0 Pin Assignments

```cpp
// Motor Outputs (PWM - 250 Hz)
#define MOTOR_PIN1    2    // Front Motor (M1)
#define MOTOR_PIN2    3    // Left Motor (M2)
#define MOTOR_PIN3    4    // Right Motor (M3)
#define MOTOR_PIN4    5    // Back Motor (M4)

// Peripherals
#define BUZZER_PIN    6    // Status buzzer
#define RECEIVER_PIN  10   // PPM input from RC receiver
#define LED_PIN       13   // Status LED (built-in)

// I2C Bus (Wire)
// SDA - Pin 18
// SCL - Pin 19
//   - MPU9250 (0x68)
//   - AK8963 (0x0C)
//   - BMP280 (0x76)

// Serial Ports
// Serial  (USB)    - Debug/programming
// Serial4 (Pins 7,8) - 433 MHz telemetry @ 115200 baud
// GPS uses Software Serial or dedicated UART
```

### Connection Diagram

```
Teensy 4.0 Pinout:
                         ┌─────────────┐
                    GND ─┤1          40├─ VIN (5V)
         ESC 1 (M1)  D2 ─┤2          39├─ 3V3
         ESC 2 (M2)  D3 ─┤3          38├─ D23
         ESC 3 (M3)  D4 ─┤4          37├─ D22
         ESC 4 (M4)  D5 ─┤5          36├─ D21
         Buzzer      D6 ─┤6          35├─ D20
                     D7 ─┤7  TEENSY  34├─ D19 (SCL - I2C)
                     D8 ─┤8   4.0    33├─ D18 (SDA - I2C)
                     D9 ─┤9          32├─ D17 (TX4 - Telemetry)
        PPM Input   D10 ─┤10         31├─ D16 (RX4 - Telemetry)
                    D11 ─┤11         30├─ D15
                    D12 ─┤12         29├─ D14
        LED (built) D13 ─┤13         28├─ GND
                         └─────────────┘

I2C Connections:
  MPU9250:  VCC→3.3V, GND→GND, SDA→18, SCL→19
  BMP280:   VCC→3.3V, GND→GND, SDA→18, SCL→19

ESC Connections:
  Signal: Teensy PWM pins → ESC signal input
  Power:  ESC power from PDB, ground common with Teensy
```

---

## ✈️ Flight Modes

### Mode 0: STABILIZE (Default)
**Purpose**: Manual flight with attitude stabilization

**Behavior**:
- Direct stick input controls desired angles
- Altitude controlled manually via throttle
- No GPS usage
- Ideal for: Takeoff, landing, manual flying

**RC Input Mapping**:
```
Roll Stick  → Desired Roll Angle  (-50° to +50°)
Pitch Stick → Desired Pitch Angle (-50° to +50°)
Throttle    → Motor Thrust        (1000-2000 μs)
Yaw Stick   → Yaw Rate            (±50°/s)
```

### Mode 1: LOITER
**Purpose**: GPS position hold

**Behavior**:
- Maintains horizontal position using GPS
- Auto-compensates for wind drift
- Altitude hold active
- Target position set when entering mode

**Requirements**:
- GPS fix valid (≥4 satellites)
- Stable altitude reference

**PID Cascade**:
```
GPS Position Error → Desired Tilt Angle → Rate → Motor Output
```

### Mode 2: ALT_HOLD
**Purpose**: Maintain altitude, manual positioning

**Behavior**:
- Barometer maintains altitude
- Manual roll/pitch control
- Target altitude set when entering mode
- Throttle stick adjusts target altitude

**Control**:
```
Throttle centered (1500) → Hold current altitude
Throttle up              → Climb
Throttle down            → Descend
```

### Mode 3: RTL (Return To Launch)
**Purpose**: Autonomous return to takeoff point

**Behavior**:
- Flies back to GPS home position
- Maintains safe altitude
- Automatic landing sequence (future)

**Activation**:
- Switch to mode 3
- Requires valid home position recorded at arming

---

## 🚀 Installation Guide

### Prerequisites

```bash
Required Software:
- Arduino IDE 1.8.19+ or Teensyduino
- Teensy Loader Application
- USB drivers for Teensy 4.0

Required Libraries:
- Wire (built-in)
- PulsePosition (Teensyduino)
- BasicLinearAlgebra (for Kalman filter)
```

### Step 1: Install Arduino IDE and Teensyduino

1. Download Arduino IDE from arduino.cc
2. Download Teensyduino installer from pjrc.com
3. Run Teensyduino installer and select Arduino IDE location
4. Install with default libraries

### Step 2: Install Required Libraries

```bash
# In Arduino IDE:
Sketch → Include Library → Manage Libraries

Search and install:
1. "BasicLinearAlgebra" by Tom Stewart
```

### Step 3: Hardware Setup

1. **Mount Flight Controller**
   - Arrow pointing forward
   - Level mounting surface
   - Vibration dampening (optional but recommended)

2. **Connect I2C Devices**
   ```
   All on same bus:
   SDA (Pin 18) → MPU9250 SDA, BMP280 SDA
   SCL (Pin 19) → MPU9250 SCL, BMP280 SCL
   3.3V → All VCC pins
   GND → All GND pins
   ```

3. **Connect ESCs**
   ```
   ESC signal wires to pins 2, 3, 4, 5
   ESC ground to Teensy GND
   Do NOT connect ESC 5V to Teensy
   ```

4. **Connect Receiver**
   ```
   PPM output → Pin 10
   Receiver VCC → 5V (from BEC)
   Receiver GND → Teensy GND
   ```

5. **Connect Telemetry**
   ```
   Radio TX → Pin 16 (RX4)
   Radio RX → Pin 17 (TX4)
   ```

### Step 4: Upload Firmware

1. Open `nasional.ino` in Arduino IDE
2. Select Tools → Board → "Teensy 4.0"
3. Select Tools → USB Type → "Serial"
4. Select Tools → CPU Speed → "600 MHz"
5. Click Upload (or Ctrl+U)
6. Teensy Loader will open automatically
7. Press button on Teensy to program

### Step 5: Verify Installation

1. Open Serial Monitor (115200 baud)
2. You should see initialization messages
3. Check for:
   ```
   MPU9250 detected: 0x71
   Magnetometer detected: 0x48
   BMP280 initialized
   Calibration complete
   ```

---

## 🎛️ Calibration Procedures

### A. IMU Calibration (Gyro + Accelerometer)

**Pre-calibration values** (currently in code):
```cpp
accel_offset_x = -0.0511
accel_offset_y = -0.0116
accel_offset_z = 0.0102
gyro_offset_x = -1.0869
gyro_offset_y = -0.0208
gyro_offset_z = -0.6061
```

**When to recalibrate**:
- After firmware updates
- If angles drift significantly
- After physical impact/crash
- If using different MPU9250

**Procedure**:
1. Place drone on perfectly level surface
2. Ensure no vibrations
3. Power on and wait 5 seconds
4. Calibration runs automatically during startup
5. Do not move for 3 seconds during calibration
6. Note new values from serial output
7. Update values in code if needed

### B. Magnetometer Calibration

**Pre-calibration values**:
```cpp
mag_offset_x = 248.50
mag_offset_y = -104.00
mag_offset_z = 12.00
mag_scale_x = 1.0047
mag_scale_y = 1.0567
mag_scale_z = 0.9449
```

**Procedure**:
1. Power on drone
2. Slowly rotate in figure-8 pattern
3. Cover all orientations
4. Duration: 30-60 seconds
5. Update calibration values

**Testing**:
- Yaw should track compass heading
- No drift in LOITER mode

### C. ESC Calibration

**Required**: Only if ESCs haven't been calibrated

**Procedure**:
1. **REMOVE ALL PROPELLERS**
2. Disconnect LiPo battery
3. Turn on radio transmitter
4. Move throttle stick to MAXIMUM
5. Connect LiPo battery
6. Wait for ESC beeps
7. Move throttle stick to MINIMUM
8. ESCs will beep again (calibration saved)
9. Disconnect battery
10. Return throttle to minimum
11. Reconnect battery

**Verification**:
- All motors should start at same throttle point
- All motors should spin at same speed

### D. Radio Calibration

**Receiver endpoints** should be:
```
Channel 1 (Roll):     1000 - 2000 μs
Channel 2 (Pitch):    1000 - 2000 μs
Channel 3 (Throttle): 1000 - 2000 μs
Channel 4 (Yaw):      1000 - 2000 μs
Channel 5 (Mode):     1000 - 2000 μs
Channel 6 (Arm):      1000 - 2000 μs
```

**Verify in serial monitor**:
```
Watch ReceiverValue[0-5] as you move sticks
All channels should show 1000-2000 range
Center should be ~1500
```

### E. Barometer Baseline

**Automatic** during startup:
- 3000 samples collected
- Average calculated
- Stored as `AltitudeBarometerStartUp`
- All subsequent readings referenced to this

**Manual reset**: Power cycle the drone at ground level

---

## ⚙️ PID Tuning

### Current PID Values

#### Rate Controllers (Inner Loop)
```cpp
// Roll Axis
PRateRoll = 1.3     // Proportional gain
IRateRoll = 0.01    // Integral gain
DRateRoll = 0.005   // Derivative gain

// Pitch Axis
PRatePitch = 1.3
IRatePitch = 0.01
DRatePitch = 0.003

// Yaw Axis
PRateYaw = 3.0
IRateYaw = 0.0      // Usually 0 for yaw
DRateYaw = 0.0
```

#### Angle Controllers (Outer Loop)
```cpp
// Roll Axis
PAngleRoll = 2.5
IAngleRoll = 0.1
DAngleRoll = 0.003

// Pitch Axis
PAnglePitch = 2.5
IAnglePitch = 0.01
DAnglePitch = 0.003

// Yaw Axis
PAngleYaw = 3.0
IAngleYaw = 0.001
DAngleYaw = 0.002
```

#### Position Controllers (GPS Loiter)
```cpp
PPositionX = 0.8    // Latitude
IPositionX = 0.02
DPositionX = 0.1

PPositionY = 0.8    // Longitude
IPositionY = 0.02
DPositionY = 0.1
```

#### Altitude Controllers
```cpp
// Altitude Hold
PAltitude = 2.0
IAltitude = 0.1
DAltitude = 0.03

// Vertical Velocity
PVelocity = 3.0
IVelocity = 0.1
DVelocity = 0.05
```

### Tuning Procedure

#### Phase 1: Rate PIDs (Most Important!)

**Start with P-gain only** (set I and D to 0):

1. **Test P-gain**:
   ```
   - Start at PRateRoll = 0.5
   - Increase by 0.1 until drone responds crisply
   - Too high → oscillations
   - Too low → sluggish response
   - Target: Sharp response without overshoot
   ```

2. **Add D-gain**:
   ```
   - Start at DRateRoll = PRateRoll * 0.01
   - Increase if oscillations present
   - D-gain dampens oscillations
   - Too high → amplifies noise
   ```

3. **Add I-gain**:
   ```
   - Start at IRateRoll = PRateRoll * 0.01
   - Only add if steady-state error exists
   - Too high → slow oscillations
   ```

4. **Repeat for Pitch and Yaw**

#### Phase 2: Angle PIDs

**After rate PIDs are good**:

1. Set `PAngleRoll = 1.0`, test hover
2. Increase until drone returns to level quickly
3. Add small I-gain for drift correction
4. Add tiny D-gain if overshooting

#### Phase 3: Altitude PID

1. **In ALT_HOLD mode**:
   - Start with PAltitude = 1.0
   - Increase until altitude holds
   - Add I-gain if drifting
   - D-gain helps with momentum

2. **Tune Velocity PID** for smooth altitude changes

#### Phase 4: Position PID

1. **In LOITER mode**:
   - Start with PPositionX/Y = 0.5
   - Increase until position holds
   - Watch for toilet-bowling (too high)
   - Add I-gain if drifting

### Live Tuning via Telemetry

Send commands via serial:
```json
{"t":"SET_PID","param":"PR","val":1.5}

Available parameters:
PR, IR, DR  - Rate Roll
PP, IP, DP  - Rate Pitch
PY, IY, DY  - Rate Yaw
PAR, IAR, DAR - Angle Roll
PAP, IAP, DAP - Angle Pitch
PAY, IAY, DAY - Angle Yaw
```

### Signs of Poor Tuning

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Fast oscillations | P too high | Reduce P-gain |
| Slow oscillations | I too high | Reduce I-gain |
| Sluggish response | P too low | Increase P-gain |
| Overshooting | D too low | Increase D-gain |
| Drift in hover | I too low | Increase I-gain slightly |
| Noise amplification | D too high | Reduce D-gain |

---

## 📡 Telemetry System

### JSON Protocol

Data sent every 200ms at 115200 baud:

```json
{
  "t": "TELEM",
  "R": 2.35,          // Roll angle (degrees)
  "P": -1.87,         // Pitch angle (degrees)  
  "Y": 45.2,          // Yaw angle (degrees)
  "RR": 12.5,         // Roll rate (deg/s)
  "PR": -8.3,         // Pitch rate (deg/s)
  "YR": 2.1,          // Yaw rate (deg/s)
  "A": 125.3,         // Altitude (cm)
  "V": -15.2,         // Vertical velocity (cm/s)
  "M": 1,             // Flight mode (0-3)
  "ARM": 1,           // Armed status (0/1)
  "LAT": 37.7749,     // GPS latitude
  "LON": -122.4194,   // GPS longitude
  "SAT": 8,           // Satellite count
  "FIX": 1,           // GPS fix valid (0/1)
  "T1": 1450,         // Motor 1 output (μs)
  "T2": 1520,         // Motor 2 output (μs)
  "T3": 1480,         // Motor 3 output (μs)
  "T4": 1510          // Motor 4 output (μs)
}
```

### Command Protocol

Send commands to drone:

```json
// Arm/Disarm
{"t":"ARM","val":1}    // Arm
{"t":"ARM","val":0}    // Disarm

// Change Flight Mode
{"t":"MODE","val":0}   // Stabilize
{"t":"MODE","val":1}   // Loiter
{"t":"MODE","val":2}   // Alt Hold
{"t":"MODE","val":3}   // RTL

// Set PID Values
{"t":"SET_PID","param":"PR","val":1.5}

// Request Data
{"t":"STATUS"}         // Request full status
```

---

## 🖥️ Ground Control Station

### System Components

**Aircraft Side**:
- Flight controller (Teensy 4.0)
- 433 MHz telemetry radio
- 2.4 GHz RC receiver

**Ground Side**:
- 433 MHz telemetry radio
- WiFi router
- GCS PC/Laptop
- 2.4 GHz RC transmitter

### Data Flow

```
Drone ←→ 433MHz ←→ Ground Radio ←→ WiFi ←→ GCS Software
       (telemetry)                 (network)  (monitoring)

Pilot ←→ 2.4GHz RC Transmitter ←→ Receiver on Drone
       (control)                   (manual override)
```

### Recommended GCS Software

- **Mission Planner** (Windows)
- **QGroundControl** (Cross-platform)
- **Custom web dashboard** (See docs folder)

### Setting Up Ground Station

1. **Connect telemetry radio** to PC via USB
2. **Configure COM port** in GCS software
3. **Set baud rate** to 115200
4. **Parse JSON** telemetry stream
5. **Display** on dashboard/map

---

## 🐛 Troubleshooting

### Motors Don't Spin

**Symptoms**: No response when armed and throttle up

**Checks**:
1. Is drone armed? (Channel 6 > 1500)
2. Is throttle above minimum? (> 1030)
3. Are ESCs powered?
4. Are ESC signal wires connected?
5. Check serial output for receiver values

**Solution**:
```cpp
// Verify receiver inputs
ReceiverValue[2] should be 1000-2000
ReceiverValue[5] > 1500 when arm switch on
```

### Oscillations in Flight

**Fast oscillations** (>5 Hz):
- Rate P-gain too high
- Reduce PRateRoll/Pitch by 10-20%
- Add D-gain if needed

**Slow oscillations** (1-3 Hz):
- Angle P-gain too high, OR
- Rate I-gain too high
- Reduce gains by 20%

**Toilet-bowling** (circular drift):
- Position P-gain too high
- Reduce PPositionX/Y

### GPS Not Working

**Symptoms**: SAT = 0, FIX = 0

**Checks**:
1. Is GPS module powered?
2. Is GPS connected to correct UART?
3. Is GPS outdoors with clear sky view?
4. Wait 1-2 minutes for cold start

**Testing**:
```cpp
// Monitor GPS data
Serial.print(gps.satellites);
Serial.println(gps.fix_valid);
```

### Altitude Drifts

**Symptoms**: Drone climbs/descends in ALT_HOLD

**Causes**:
1. Barometer not calibrated at ground level
2. Kalman filter not converged
3. Temperature affecting pressure reading
4. PAltitude gain too low

**Solutions**:
- Always power on at ground level
- Allow 5 seconds startup
- Increase PAltitude slightly
- Check for pressure sensor errors

### Drone Drifts in LOITER

**Symptoms**: Position not maintained

**Causes**:
1. GPS accuracy poor (< 4 satellites)
2. Position PID not tuned
3. Wind too strong
4. Magnetic interference

**Solutions**:
- Ensure ≥6 satellites
- Tune PPositionX/Y
- Avoid flying in high wind initially
- Check magnetometer calibration

### Communication Lost

**Symptoms**: No telemetry data

**Checks**:
1. Is telemetry radio powered?
2. Correct baud rate (115200)?
3. Are radios paired?
4. Check antenna connections

**Failsafe**: Drone should RTL if RC signal lost

---

## ⚠️ Safety Guidelines

### Pre-Flight Checklist

- [ ] All propellers securely mounted (correct rotation)
- [ ] Battery fully charged and securely connected
- [ ] All connections tight (no loose wires)
- [ ] Firmware uploaded successfully
- [ ] Calibration completed
- [ ] GPS has fix (≥4 satellites) for LOITER/RTL
- [ ] Clear flight area (no people/obstacles)
- [ ] Weather suitable (low wind, no rain)
- [ ] Kill switch/disarm ready
- [ ] Spotter present (recommended)

### Flight Safety Rules

1. **Always** remove propellers when testing on bench
2. **Never** arm with propellers near people
3. **Always** fly in open areas away from crowds
4. **Test** new firmware/settings without props first
5. **Start** in STABILIZE mode before advanced modes
6. **Monitor** battery voltage (land at 3.5V/cell)
7. **Have** RC override available at all times
8. **Land** immediately if unusual behavior
9. **Don't** fly beyond visual line of sight
10. **Follow** local regulations and laws

### Emergency Procedures

**Loss of Control**:
1. Switch to STABILIZE mode
2. Reduce throttle
3. If no response, cut throttle completely
4. Be prepared for crash landing

**GPS Lost in LOITER**:
1. Drone should auto-switch to STABILIZE
2. Take manual control immediately
3. Land as soon as possible

**Low Battery**:
1. RTL if GPS available
2. Otherwise descend immediately
3. Don't attempt to return from far away

**Flyaway**:
1. Cut throttle immediately (disarm)
2. Try mode switch to RTL
3. If no response, prepare for impact
4. Note last known position

### Legal & Operational

- Register drone if required in your country
- Obtain necessary permits for operation
- Respect no-fly zones
- Maintain visual line of sight
- Don't fly over people or property
- Have appropriate insurance
- Follow local aviation authority rules

---

## 📚 Additional Resources

### Code Structure

```
nasional.ino (main file)
├── Hardware Definitions
├── Enums and Structures  
├── Global Variables
├── Sensor Initialization
│   ├── initMPU9250()
│   ├── initMagnetometer()
│   └── initBarometer()
├── Sensor Reading
│   ├── sensor_signals()
│   ├── barometer_signals()
│   └── readMagnetometer()
├── Sensor Fusion
│   └── madgwickQuaternionUpdate()
├── Kalman Filter
│   └── kalman_2d()
├── PID Controllers
│   └── pid_equation()
├── Flight Modes
│   ├── processLoiterMode()
│   └── processAltitudeControl()
├── Communication
│   ├── parseGPS()
│   ├── sendTelemetry()
│   └── handleSerial()
└── Setup and Main Loop
```

### Key Algorithms

**Madgwick AHRS**: Sensor fusion for attitude estimation
- Uses gyro, accel, and magnetometer
- Quaternion-based representation
- Beta parameter (0.3) controls fusion rate

**Kalman Filter 2D**: Altitude estimation
- State: [altitude, vertical_velocity]
- Measurement: barometer altitude
- Control: vertical acceleration from IMU
- Updates at 250 Hz

**Cascade PID**: Multi-loop control
- Position → Angle → Rate
- Each loop runs at full 250 Hz
- Anti-windup and limits applied

### External Links

- [Teensy 4.0 Documentation](https://www.pjrc.com/store/teensy40.html)
- [MPU9250 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/)
- [BMP280 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp280/)
- [Madgwick AHRS Paper](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- [BasicLinearAlgebra Library](https://github.com/tomstewart89/BasicLinearAlgebra)

---

## 📄 License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments
- ** AL-MUBAROK-TD ** UMY
- **Teensy Community** for excellent hardware and support
- **Madgwick** for the AHRS algorithm
- **Arduino/Teensyduino** team
- All contributors and testers

---

**⚡ Status**: Active Development  
**🔧 Last Updated**: January 2026  

---

*Fly safe, fly smart! 🚁*
