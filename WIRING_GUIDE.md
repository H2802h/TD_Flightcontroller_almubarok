# Hardware Wiring & Setup Guide

Complete wiring instructions for the TD Flight Controller based on Teensy 4.0.

---

## Table of Contents

1. [Component List](#component-list)
2. [Power Distribution](#power-distribution)
3. [Teensy 4.0 Connections](#teensy-40-connections)
4. [I2C Bus Wiring](#i2c-bus-wiring)
5. [Motor/ESC Connections](#motoresc-connections)
6. [RC Receiver Wiring](#rc-receiver-wiring)
7. [Telemetry Radio Wiring](#telemetry-radio-wiring)
8. [GPS Module Wiring](#gps-module-wiring)
9. [Power Supply](#power-supply)
10. [Assembly Steps](#assembly-steps)
11. [Verification Tests](#verification-tests)
12. [Troubleshooting](#troubleshooting)

---

## Component List

### Required Components

| Qty | Component | Specification | Notes |
|-----|-----------|--------------|-------|
| 1 | Teensy 4.0 | 600 MHz ARM M7 | Flight controller |
| 1 | MPU9250 | 9-DOF IMU | Gyro + Accel + Mag |
| 1 | BMP280 | Barometer | Altitude sensor |
| 1 | GPS Module | NEO-6M or NEO-M8N | UART interface |
| 1 | RC Receiver | PPM output | 6+ channels |
| 1 | 433MHz Radio | UART interface | Telemetry |
| 4 | ESCs | 30A (minimum) | Oneshot125 compatible |
| 4 | Brushless Motors | 2300KV typical | Match to frame size |
| 1 | LiPo Battery | 4S 2200-5000mAh | XT60 connector |
| 1 | Power Distribution Board | 30A+ rating | With 5V BECs |
| 4 | Propellers | Match motor size | 2 CW, 2 CCW |
| 1 | Frame | 450mm recommended | X-configuration |

### Wiring & Connectors

| Qty | Item | Purpose |
|-----|------|---------|
| 1m | 22 AWG Silicone Wire (Red) | Power positive |
| 1m | 22 AWG Silicone Wire (Black) | Power ground |
| 2m | 26 AWG Wire (various colors) | Signal wires |
| 1 | XT60 Connector (male) | Battery connection |
| 4 | 3-pin Servo Connectors | ESC signal |
| 10 | JST-SH 1.0mm connectors | I2C devices |
| 1 | Heat shrink tubing | Various sizes |
| 1 | Zip ties | Cable management |

### Tools Required

- Soldering iron (60W recommended)
- Solder (60/40 or lead-free)
- Wire strippers
- Multimeter
- Heat gun
- Screwdrivers (Phillips, hex)
- Helping hands/PCB holder

---

## Power Distribution

### Power System Architecture

```
LiPo 4S Battery (14.8V nominal, 12.6V-16.8V range)
    │
    ├──► Power Distribution Board (PDB)
          │
          ├──► ESC 1 ──► Motor 1 (Front)
          ├──► ESC 2 ──► Motor 2 (Left)
          ├──► ESC 3 ──► Motor 3 (Right)
          ├──► ESC 4 ──► Motor 4 (Back)
          │
          ├──► 5V BEC (3A) ──┬──► Teensy 4.0 (VIN or USB)
          │                   ├──► MPU9250 (3.3V via Teensy)
          │                   ├──► BMP280 (3.3V via Teensy)
          │                   └──► GPS Module (5V tolerant)
          │
          └──► 5V BEC (2A) ──┬──► RC Receiver
                             └──► Telemetry Radio
```

### Power Requirements

| Component | Voltage | Current | Power |
|-----------|---------|---------|-------|
| Teensy 4.0 | 5V | 150mA | 0.75W |
| MPU9250 | 3.3V | 3.5mA | 0.01W |
| BMP280 | 3.3V | 2.7mA | 0.01W |
| GPS | 5V | 50mA | 0.25W |
| Receiver | 5V | 100mA | 0.5W |
| Telemetry | 5V | 100mA | 0.5W |
| **Total** | **5V** | **~410mA** | **~2W** |

**Note**: Motors can draw 20-30A each at full throttle. Use appropriate wire gauge!

---

## Teensy 4.0 Connections

### Pin Map (Based on Your Code)

```
                         ┌─────────────┐
                    GND ─┤1          40├─ VIN (5V from BEC)
         ESC 1 (M1)  D2 ─┤2          39├─ 3V3 Output
         ESC 2 (M2)  D3 ─┤3          38├─ D23
         ESC 3 (M3)  D4 ─┤4          37├─ D22
         ESC 4 (M4)  D5 ─┤5          36├─ D21
         Buzzer      D6 ─┤6          35├─ D20
                     D7 ─┤7  TEENSY  34├─ D19 (SCL) ─── I2C Clock
                     D8 ─┤8   4.0    33├─ D18 (SDA) ─── I2C Data
                     D9 ─┤9          32├─ D17 (TX4) ─── Telem TX
        PPM Input   D10 ─┤10         31├─ D16 (RX4) ─── Telem RX
                    D11 ─┤11         30├─ D15
                    D12 ─┤12         29├─ D14
        LED (built) D13 ─┤13         28├─ GND
                         └─────────────┘
```

### Detailed Pin Functions

| Pin | Function | Connection | Signal Type |
|-----|----------|------------|-------------|
| D2 | Motor 1 PWM | ESC 1 Signal | PWM 250Hz |
| D3 | Motor 2 PWM | ESC 2 Signal | PWM 250Hz |
| D4 | Motor 3 PWM | ESC 3 Signal | PWM 250Hz |
| D5 | Motor 4 PWM | ESC 4 Signal | PWM 250Hz |
| D6 | Buzzer | Piezo buzzer | Digital Out |
| D10 | PPM Input | RC Receiver PPM | Digital In |
| D13 | Status LED | Built-in LED | Digital Out |
| D16 | Telemetry RX | 433MHz Radio TX | UART RX |
| D17 | Telemetry TX | 433MHz Radio RX | UART TX |
| D18 | I2C SDA | MPU9250, BMP280 | I2C Data |
| D19 | I2C SCL | MPU9250, BMP280 | I2C Clock |
| VIN | Power In | 5V BEC | Power |
| 3V3 | Power Out | Sensor power | Power |
| GND | Ground | Common ground | Ground |

---

## I2C Bus Wiring

### I2C Device Addresses

| Device | Address | Function |
|--------|---------|----------|
| MPU9250 | 0x68 | IMU (Gyro + Accel) |
| AK8963 | 0x0C | Magnetometer (inside MPU9250) |
| BMP280 | 0x76 | Barometer |

### Wiring Diagram

```
Teensy 4.0          MPU9250 Breakout        BMP280 Breakout
                         
D18 (SDA) ────┬──────── SDA                     SDA ────┐
              │                                          │
D19 (SCL) ────┼──────── SCL                     SCL ────┤
              │                                          │
3V3 ──────────┼──────── VCC (3.3V)              VCC ────┤
              │                                          │
GND ──────────┴──────── GND                     GND ────┘

Pull-up resistors (4.7kΩ) already on breakout boards
```

### I2C Connection Details

**MPU9250**:
```
Pin 1 (VCC) ── 3.3V from Teensy
Pin 2 (GND) ── GND
Pin 3 (SCL) ── D19 (SCL)
Pin 4 (SDA) ── D18 (SDA)
Pin 5 (INT) ── Not connected
Pin 6 (AD0) ── GND (sets address to 0x68)
```

**BMP280**:
```
Pin 1 (VCC) ── 3.3V from Teensy
Pin 2 (GND) ── GND
Pin 3 (SCL) ── D19 (SCL)
Pin 4 (SDA) ── D18 (SDA)
Pin 5 (CSB) ── 3.3V (I2C mode)
Pin 6 (SDO) ── GND (address 0x76)
```

### Important Notes

1. **3.3V ONLY**: MPU9250 and BMP280 are 3.3V devices
2. **Pull-ups**: Most breakouts have 4.7kΩ pull-ups already
3. **Wire Length**: Keep I2C wires < 20cm for reliability
4. **Clock Speed**: Code sets 400kHz (fast mode)

---

## Motor/ESC Connections

### Motor Layout (X-Configuration)

```
        FRONT (Nose)
             ↑
             
      (2)         (4)
       ↺           ↻
         
         [FC]
         
       ↻           ↺
      (1)         (3)
      
             ↓
        BACK (Tail)

Motor 1: Front - CCW rotation - Pin D2
Motor 2: Left  - CCW rotation - Pin D3
Motor 3: Right - CW rotation  - Pin D4
Motor 4: Back  - CW rotation  - Pin D5
```

### ESC Wiring

**Each ESC has 3 wires**:

1. **Power Wires** (thick, red/black):
   - Red → PDB positive pad
   - Black → PDB negative pad
   - Use 18-20 AWG wire
   - Solder directly to PDB

2. **Motor Wires** (thick, 3 wires):
   - Connect to motor
   - Any 2 wires can be swapped to reverse direction

3. **Signal Wire** (thin, 3-pin connector):
   ```
   ESC Signal Connector:
   [Black] ── GND (common with Teensy)
   [Red]   ── 5V (DO NOT CONNECT to Teensy!)
   [White] ── Signal to Teensy pin
   ```

### ESC to Teensy Connection

| ESC | Motor | Teensy Pin | Notes |
|-----|-------|------------|-------|
| ESC 1 | M1 (Front) | D2 | CCW propeller |
| ESC 2 | M2 (Left) | D3 | CCW propeller |
| ESC 3 | M3 (Right) | D4 | CW propeller |
| ESC 4 | M4 (Back) | D5 | CW propeller |

**CRITICAL**: Do NOT connect ESC 5V (red wire) to Teensy! Only signal (white) and ground (black).

### ESC Signal Details

- **PWM Frequency**: 250 Hz
- **Pulse Width Range**: 1000-2000 μs
- **Protocol**: Standard PWM (Oneshot125 compatible)
- **Idle**: 1170 μs
- **Cutoff**: 1000 μs

---

## RC Receiver Wiring

### PPM Mode Wiring

```
RC Receiver                     Teensy 4.0
                         
CH1 (Roll) ──┐                    
CH2 (Pitch) ─┤                    
CH3 (Throttle)─┤── Combined    
CH4 (Yaw) ───┤    PPM Signal ──── D10
CH5 (Mode) ──┤                    
CH6 (Arm) ───┘                    
                                  
VCC ─────────────── 5V BEC        
GND ─────────────── GND           
```

### Channel Mapping

| Channel | Function | Range | Center |
|---------|----------|-------|--------|
| 1 | Roll | 1000-2000 | 1500 |
| 2 | Pitch | 1000-2000 | 1500 |
| 3 | Throttle | 1000-2000 | 1000 (min) |
| 4 | Yaw | 1000-2000 | 1500 |
| 5 | Flight Mode | 1000-2000 | - |
| 6 | Arm/Disarm | 1000-2000 | - |

### Mode Switch Setup

**3-Position Switch** (Channel 5):
```
Position 1 (1000-1333): STABILIZE (Mode 0)
Position 2 (1334-1666): LOITER (Mode 1) or ALT_HOLD (Mode 2)
Position 3 (1667-2000): RTL (Mode 3)
```

**Arm Switch** (Channel 6):
```
< 1500: Disarmed
> 1500: Armed
```

---

## Telemetry Radio Wiring

### 433 MHz Radio Connection

```
Telemetry Radio          Teensy 4.0

VCC (5V) ────────────── 5V BEC
GND ─────────────────── GND
TX ──────────────────── D16 (RX4)
RX ──────────────────── D17 (TX4)
```

### Serial Configuration

- **Baud Rate**: 115200
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Protocol**: JSON packets
- **Interval**: 200ms (5 Hz)

### Ground Station Setup

1. Connect matching 433MHz radio to PC via USB
2. Open serial terminal (115200 baud)
3. You should see JSON telemetry data
4. Use GCS software to parse and display

---

## GPS Module Wiring

### GPS Connection

```
GPS Module              Power / Teensy

VCC (5V) ───────────── 5V BEC
GND ────────────────── GND
TX ─────────────────── (RX pin - configure in code)
RX ─────────────────── (TX pin - configure in code)
```

**Note**: GPS serial port configuration not shown in provided code snippet. Verify actual pins used.

### GPS Mounting

- Mount away from ESCs/motors (EMI)
- Antenna facing up
- Clear view of sky
- Use standoffs or foam pad

---

## Power Supply

### LiPo Battery Connection

```
LiPo Battery 4S
    │
    ├── (+) XT60 Connector
    │
    └── (-) XT60 Connector
         │
         ▼
    Power Distribution Board
         │
         ├──► 4x ESCs (direct connection)
         │
         ├──► 5V BEC #1 (Flight Controller)
         │
         └──► 5V BEC #2 (Receiver/Radio)
```

### BEC Requirements

**BEC #1** (Flight Controller):
- Input: 12-17V (4S)
- Output: 5V @ 3A minimum
- Powers: Teensy, sensors, GPS

**BEC #2** (RC System):
- Input: 12-17V (4S)
- Output: 5V @ 2A minimum
- Powers: Receiver, telemetry radio

### Ground Connection

**CRITICAL**: All grounds must be connected:
- Battery ground
- PDB ground
- ESC grounds
- Teensy ground
- Sensor grounds
- Receiver ground
- Telemetry ground

---

## Assembly Steps

### Step 1: Mount Flight Controller

1. Find center of frame
2. Use vibration dampening mounts
3. Arrow pointing forward
4. Ensure level mounting

### Step 2: Install Power Distribution

1. Mount PDB securely
2. Solder battery connector
3. Add XT60 pigtail with switch (optional)
4. Install BECs

### Step 3: Connect ESCs

1. Solder ESC power wires to PDB
2. Connect motors to ESCs
3. Route signal wires to FC
4. Secure with zip ties

### Step 4: Wire Sensors

1. Connect MPU9250 to I2C
2. Connect BMP280 to I2C
3. Keep wires short and neat
4. Test with multimeter (3.3V, continuity)

### Step 5: Connect Receiver

1. Mount receiver
2. Connect PPM output to D10
3. Connect power (5V, GND)
4. Bind to transmitter

### Step 6: Connect Telemetry

1. Mount 433MHz radio
2. Connect to Serial4 (D16, D17)
3. Connect power
4. Install antenna

### Step 7: Connect GPS

1. Mount GPS module
2. Connect serial wires
3. Connect power
4. Position antenna

### Step 8: Final Checks

1. Verify all connections
2. Check for shorts (multimeter)
3. Verify voltage at each component
4. Secure all wires

---

## Verification Tests

### Pre-Power Tests

1. **Visual Inspection**:
   - [ ] No bare wire exposed
   - [ ] No shorts visible
   - [ ] All connectors secure
   - [ ] Polarity correct

2. **Continuity Tests**:
   - [ ] Ground continuity
   - [ ] No shorts between power and ground
   - [ ] Signal wires connected

3. **Voltage Tests** (before connecting battery):
   - Use bench power supply at 5V
   - Verify 5V at Teensy VIN
   - Verify 3.3V at sensor VCC

### First Power-On

1. **Without Propellers**:
   - [ ] Remove all propellers
   - [ ] Connect battery
   - [ ] Check LED lights up
   - [ ] Listen for beeps

2. **Serial Monitor Check**:
   - [ ] Connect USB
   - [ ] Open serial monitor (115200 baud)
   - [ ] Verify initialization messages
   - [ ] Check sensor detection

3. **Sensor Tests**:
   ```
   Expected output:
   MPU9250 detected: 0x71
   Magnetometer detected: 0x48
   BMP280 initialized
   Calibrating... (3 seconds)
   Calibration complete
   Ready!
   ```

4. **Receiver Test**:
   - [ ] Move sticks
   - [ ] Verify ReceiverValue[0-5] changes
   - [ ] Test mode switch
   - [ ] Test arm switch

5. **Motor Direction Test**:
   - [ ] Arm drone (low throttle)
   - [ ] Increase throttle slightly
   - [ ] Verify motors spin
   - [ ] Check rotation direction
   - [ ] Swap 2 motor wires if wrong

---

## Troubleshooting

### No Power

**Symptoms**: No LED, no serial output

**Checks**:
1. Battery charged?
2. Battery connector secure?
3. BEC working? (measure 5V output)
4. Fuse blown on PDB?
5. Check continuity from BEC to Teensy VIN

### Sensor Not Detected

**Symptoms**: "Sensor not found" in serial

**MPU9250 Issues**:
1. Check I2C wiring (SDA, SCL)
2. Verify 3.3V power
3. Check address (should be 0x68)
4. Try I2C scanner sketch

**BMP280 Issues**:
1. Check SDO pin (should be GND for 0x76)
2. Check CSB pin (should be HIGH for I2C)
3. Verify connections

### Motors Don't Spin

**Symptoms**: Armed, throttle up, no motors

**Checks**:
1. ESC powered? (check voltage)
2. Signal wires connected?
3. ESCs calibrated?
4. Receiver showing throttle > 1030?
5. Check serial: `ReceiverValue[2]`

### Unstable Flight

**Symptoms**: Oscillations, won't hover

**Checks**:
1. Vibrations? (add dampening)
2. Props on correct motors?
3. Motor directions correct?
4. PID gains too high?
5. FC mounted level?

### GPS Not Working

**Symptoms**: No fix, SAT=0

**Checks**:
1. Outdoors with clear sky?
2. GPS powered?
3. Serial connection correct?
4. Antenna connected?
5. Wait 2-5 minutes for cold start

---

## Wire Color Conventions

### Recommended Colors

| Wire | Color | Purpose |
|------|-------|---------|
| Battery + | Red | Positive power |
| Battery - | Black | Ground |
| 5V | Red | Regulated 5V |
| 3.3V | Orange | Regulated 3.3V |
| GND | Black | Ground |
| I2C SDA | Blue | I2C data |
| I2C SCL | Yellow | I2C clock |
| UART TX | Green | Serial transmit |
| UART RX | White | Serial receive |
| PWM | Yellow/White | PWM signals |

---

## Safety Warnings

⚠️ **DANGER - HIGH CURRENT**:
- 4S LiPo can deliver 100+ amps
- Short circuit = fire hazard
- Always disconnect battery when wiring
- Use insulated tools
- Keep fire extinguisher nearby

⚠️ **PROPELLER SAFETY**:
- Remove props during all testing
- Props can cause serious injury
- Never reach near spinning props
- Always disarm before approaching

⚠️ **VOLTAGE LEVELS**:
- Teensy pins are 3.3V tolerant
- Never connect 5V directly to 3.3V pins
- Use level shifters if needed
- Check voltage before connecting

---

## Maintenance

### Regular Checks

**Before Each Flight**:
- [ ] All connections tight
- [ ] No damaged wires
- [ ] Props secure and undamaged
- [ ] Battery voltage good
- [ ] No loose screws

**Monthly**:
- [ ] Check solder joints
- [ ] Clean sensors
- [ ] Check for wire fatigue
- [ ] Re-apply thread locker if needed

### After Crashes

- [ ] Visual inspection of all components
- [ ] Check motor bearings
- [ ] Verify FC still level
- [ ] Test all functions on bench
- [ ] Recalibrate if needed

---

This wiring guide should be followed carefully to ensure safe and reliable operation of your flight controller!
