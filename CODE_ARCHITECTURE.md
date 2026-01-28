# Code Architecture Documentation

## Overview

This document provides a detailed breakdown of the `nasional.ino` code architecture for the TD Flight Controller.

---

## File Structure

```
nasional.ino (1540 lines)
├── Lines 1-44:     Hardware Definitions & Constants
├── Lines 46-80:    Data Structures
├── Lines 82-199:   Global Variables
├── Lines 201-282:  Sensor Initialization
├── Lines 284-502:  Sensor Reading Functions
├── Lines 504-691:  Madgwick AHRS Algorithm
├── Lines 693-856:  Kalman Filter Implementation
├── Lines 858-1020: Flight Mode Processing
├── Lines 1022-1154: PID Controllers
├── Lines 1156-1302: Communication & Telemetry
└── Lines 1304-1540: Setup & Main Loop
```

---

## Key Components

### 1. Hardware Definitions (Lines 10-44)

**MPU9250 Registers**:
```cpp
#define MPU9250_ADDRESS 0x68  // IMU I2C address
#define AK8963_ADDRESS 0x0C   // Magnetometer I2C address
// ... register definitions
```

**Pin Assignments**:
```cpp
#define MOTOR_PIN1 2          // Front motor
#define MOTOR_PIN2 3          // Left motor
#define MOTOR_PIN3 4          // Right motor
#define MOTOR_PIN4 5          // Back motor
#define BUZZER_PIN 6
#define RECEIVER_PIN 10       // PPM input
```

### 2. Data Structures (Lines 46-80)

**FlightMode Enum**:
```cpp
enum FlightMode {
  STABILIZE = 0,    // Manual with stabilization
  LOITER = 1,       // GPS position hold
  ALT_HOLD = 2,     // Altitude hold
  RTL = 3           // Return to launch
};
```

**GPSData Structure**:
```cpp
struct GPSData {
  double latitude, longitude;
  float altitude, ground_speed, course;
  int satellites;
  bool fix_valid;
  unsigned long last_update;
};
```

**CalibrationData Structure**:
```cpp
struct CalibrationData {
  float accel_offset_x, accel_offset_y, accel_offset_z;
  float gyro_offset_x, gyro_offset_y, gyro_offset_z;
  float mag_offset_x, mag_offset_y, mag_offset_z;
  float mag_scale_x, mag_scale_y, mag_scale_z;
};
```

### 3. Sensor Initialization (Lines 204-282)

**initMPU9250()** (Lines 204-220):
- Verifies WHO_AM_I register (0x71)
- Resets device
- Configures accelerometer: ±2g
- Configures gyroscope: ±250 dps
- Sets sample rate: 100 Hz
- Configures low-pass filter: 44 Hz
- Enables I2C bypass for magnetometer

**initMagnetometer()** (Lines 222-231):
- Verifies WHO_AM_I register (0x48)
- Resets device
- Configures: 16-bit mode, 100 Hz

**initBarometer()** (Lines 233-282):
- Resets BMP280
- Configures oversampling:
  - Temperature: x2
  - Pressure: x16
- Sets filter coefficient: 4
- Reads calibration coefficients
- Sets baseline pressure

### 4. Sensor Reading (Lines 284-502)

**sensor_signals()** (Lines 284-430):
```cpp
void sensor_signals() {
  // 1. Read raw IMU data (6 bytes accel + 6 bytes gyro)
  // 2. Apply calibration offsets
  // 3. Convert to physical units
  // 4. Update Madgwick filter
  // 5. Calculate Roll, Pitch, Yaw from quaternions
}
```

**Flow**:
1. Read 14 bytes from MPU9250 (accel + temp + gyro)
2. Combine high/low bytes
3. Apply calibration offsets
4. Scale to g's and rad/s
5. Read magnetometer if enabled
6. Run Madgwick update
7. Extract Euler angles from quaternion

**barometer_signals()** (Lines 432-502):
```cpp
void barometer_signals() {
  // 1. Read pressure and temperature
  // 2. Apply compensation formulas
  // 3. Calculate altitude
  // 4. Apply smoothing filter
}
```

**BMP280 Compensation**:
- Uses factory calibration coefficients
- Temperature compensation first
- Then pressure compensation
- Altitude from barometric formula:
  ```
  h = 44330 * (1 - (P/P0)^0.1903)
  ```

### 5. Madgwick AHRS (Lines 504-691)

**madgwickQuaternionUpdate()** (Lines 504-691):

**Inputs**:
- `gx, gy, gz`: Gyro rates (rad/s)
- `ax, ay, az`: Accelerometer (g's)
- `mx, my, mz`: Magnetometer (μT) [optional]

**Process**:
1. Normalize accelerometer vector
2. Normalize magnetometer vector (if used)
3. Compute objective function and gradient
4. Normalize gradient
5. Integrate rate of change of quaternion
6. Normalize quaternion
7. Update global q0, q1, q2, q3

**Parameters**:
- `beta = 0.3`: Fusion gain (tunable)
- `dt = 0.004`: Sample period (250 Hz)

**Output**:
- Updated quaternion (q0, q1, q2, q3)
- Converted to Euler angles:
  ```cpp
  Roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1² + q2²))
  Pitch = asin(2*(q0*q2 - q3*q1))
  Yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2² + q3²))
  ```

### 6. Kalman Filter (Lines 693-856)

**kalman_2d()** (Lines 693-856):

**State Vector**:
```
S = [altitude, vertical_velocity]ᵀ
```

**State Transition**:
```
F = [1  dt]  where dt = 0.004s (250 Hz)
    [0  1 ]
```

**Control Input**:
```
G = [0.5*dt²]  (acceleration integration)
    [dt     ]

u = AccZ_inertial (vertical acceleration)
```

**Measurement**:
```
H = [1  0]  (observe altitude only)

z = altitude_barometer
```

**Process**:
1. **Prediction**:
   ```
   S = F * S + G * u
   P = F * P * Fᵀ + Q
   ```

2. **Correction**:
   ```
   K = P * Hᵀ * (H * P * Hᵀ + R)⁻¹
   S = S + K * (z - H * S)
   P = (I - K * H) * P
   ```

**Noise Parameters**:
- Process noise (Q): 0.05 (tuned for stability)
- Measurement noise (R): 100.0 (barometer variance)

**Outputs**:
- `AltitudeKalman`: Filtered altitude (cm)
- `VelocityVerticalKalman`: Estimated velocity (cm/s)

### 7. PID Controllers (Lines 1022-1154)

**pid_equation()** (Lines 1022-1044):
```cpp
void pid_equation(float Error, float P, float I, float D, 
                  float PrevError, float PrevIterm) {
  // P term
  float Pterm = P * Error;
  
  // I term with anti-windup
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  Iterm = constrain(Iterm, -400, 400);
  
  // D term
  float Dterm = D * (Error - PrevError) / 0.004;
  
  // Output
  PIDReturn[0] = Pterm + Iterm + Dterm;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}
```

**Key Features**:
- Trapezoidal integration for I-term
- Anti-windup limiting (±400)
- Derivative on error (not measurement)
- Fixed dt = 0.004s (250 Hz)

**Control Cascade**:

```
1. Position PID (GPS):
   Error = Target Position - Current Position
   Output = Desired Tilt Angle

2. Altitude PID:
   Error = Target Altitude - Current Altitude
   Output = Desired Vertical Velocity
   
3. Velocity PID:
   Error = Desired Velocity - Kalman Velocity
   Output = Throttle Correction

4. Angle PID:
   Error = Desired Angle - Current Angle
   Output = Desired Rate

5. Rate PID:
   Error = Desired Rate - Current Rate
   Output = Motor Command
```

### 8. Flight Modes (Lines 858-1020)

**processLoiterMode()** (Lines 858-932):

**Logic**:
1. Check GPS validity (fix + satellites ≥ 4)
2. Set target position if just entered mode
3. Calculate position error (meters)
4. Convert to bearing and distance
5. Calculate North/East errors
6. Run position PID
7. Output desired roll/pitch angles
8. Limit angles to ±MAX_LOITER_LEAN (15°)

**Position Calculations**:
```cpp
// Haversine formula for distance
float dlat = (target_lat - current_lat) * 111319.9;  // meters
float dlon = (target_lon - current_lon) * 111319.9 * cos(lat_rad);
float distance = sqrt(dlat² + dlon²);
```

**processAltitudeControl()** (Lines 934-1020):

**Logic**:
1. Check if in altitude-holding mode (LOITER or ALT_HOLD)
2. Set target altitude if just entered mode
3. Allow throttle stick to adjust target (deadband ±50)
4. Calculate altitude error
5. Run altitude PID → desired vertical velocity
6. Run velocity PID → throttle correction
7. Add to base throttle
8. Reset PIDs if mode exited

**Throttle Mapping**:
```cpp
if (abs(throttle_input) > 50) {
  // Adjust target altitude
  DesiredAltitude += throttle_input * 0.02;  // 2cm per unit
}

InputThrottle = BaseThrottle + AltitudeThrottleCorrection;
```

### 9. Communication (Lines 1156-1302)

**parseGPS()** (Lines 1156-1229):
- Reads NMEA sentences from GPS serial
- Parses $GPGGA and $GPRMC messages
- Extracts: lat, lon, altitude, speed, course, satellites
- Updates `gps` structure
- Sets `fix_valid` flag

**sendTelemetry()** (Lines 1231-1270):
- Sends JSON packet every 200ms
- Includes all flight data:
  - Attitude (roll, pitch, yaw)
  - Rates (roll rate, pitch rate, yaw rate)
  - Altitude and velocity
  - GPS position and status
  - Flight mode and armed status
  - Motor outputs

**handleSerial()** (Lines 1272-1302):
- Parses incoming JSON commands
- Supported commands:
  - ARM/DISARM
  - MODE change
  - SET_PID (live tuning)
  - STATUS request

### 10. Main Loop (Lines 1390-1540)

**Structure** (250 Hz = 4000 μs period):

```cpp
void loop() {
  // 1. Sensor Processing (Lines 1391-1410)
  sensor_signals();
  barometer_signals();
  kalman_2d();
  parseGPS();
  
  // 2. Input Processing (Lines 1411-1414)
  handleSerial();
  read_receiver();
  handleArming();
  handleModeSwitch();
  
  // 3. Safety Check (Lines 1417-1427)
  if (!isArmed) {
    applyMotorSafety();
    sendTelemetry();
    waitForNextLoop();
    return;
  }
  
  // 4. Control Processing (Lines 1429-1500)
  // Apply gyro calibration
  // Process flight mode
  // Process altitude control
  // Yaw control
  // Angle PIDs
  // Rate PIDs
  
  // 5. Motor Mixing (Lines 1501-1528)
  // X-configuration mixing
  // Apply constraints
  // Write PWM outputs
  
  // 6. Telemetry (Lines 1530-1534)
  sendTelemetry();
  
  // 7. Loop Timing (Lines 1536-1539)
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
```

**Timing Breakdown** (approximate):
- Sensor reading: ~500 μs
- Madgwick update: ~400 μs  
- Kalman filter: ~300 μs
- PID calculations: ~200 μs
- Motor mixing: ~100 μs
- Communication: ~100 μs
- Spare time: ~2400 μs

---

## Critical Timing Requirements

### 250 Hz Loop (4000 μs)
- **Must** complete all processing within 4ms
- Enforced by: `while (micros() - LoopTimer < 4000);`
- Missed loops = unstable flight

### Sensor Sample Rates
- MPU9250: 100 Hz internal, read at 250 Hz
- BMP280: Continuous mode, read at 250 Hz
- GPS: 5 Hz typical, parsed as available
- Receiver: PPM decoded in interrupt

### Communication Rates
- Telemetry TX: 200ms intervals (5 Hz)
- Telemetry RX: Processed every loop (250 Hz)
- GPS RX: Processed every loop (buffered)

---

## Memory Usage

### Global Variables (~2KB)
- Float arrays: ~1KB
- Structures: ~500 bytes
- Kalman matrices: ~400 bytes
- Buffers: ~200 bytes

### Stack Usage (~1KB)
- Function locals
- Temporary calculations

### Teensy 4.0 has 1MB RAM, so memory is not a constraint.

---

## Power Consumption

**Teensy 4.0 at 600 MHz**:
- Active: ~100-150 mA @ 5V
- Sleep: Not used

**Sensors**:
- MPU9250: ~3.5 mA
- BMP280: ~2.7 mA
- Total system: ~160 mA @ 5V = 0.8W

---

## Future Improvements

### Performance
- [ ] Optimize matrix operations in Kalman filter
- [ ] Use DMA for I2C transfers
- [ ] Implement overrun detection

### Features
- [ ] Auto-landing in RTL mode
- [ ] Waypoint navigation
- [ ] Optical flow for indoor flight
- [ ] Blackbox logging to SD card
- [ ] Battery voltage monitoring
- [ ] Failsafe improvements

### Code Quality
- [ ] Split into multiple files (.h headers)
- [ ] Add unit tests
- [ ] Improve error handling
- [ ] Add runtime diagnostics
- [ ] Document all magic numbers

---

## Debugging Tips

### Serial Output
```cpp
// Add at start of loop() for debugging:
Serial.print("Angles: ");
Serial.print(AngleRoll); Serial.print(" ");
Serial.print(AnglePitch); Serial.print(" ");
Serial.println(MadgwickYaw);
```

### LED Indicators
```cpp
// Use built-in LED for status:
digitalWrite(13, isArmed ? HIGH : LOW);
```

### Timing Check
```cpp
// Measure loop time:
unsigned long startTime = micros();
// ... code ...
unsigned long elapsed = micros() - startTime;
if (elapsed > 4000) {
  Serial.println("LOOP OVERRUN!");
}
```

### PID Monitoring
```cpp
// Watch PID outputs:
Serial.print("Rate PIDs: ");
Serial.print(InputRoll); Serial.print(" ");
Serial.print(InputPitch); Serial.print(" ");
Serial.println(InputYaw);
```

---

## Configuration Parameters

### Critical Values to Verify

**Sensor Calibration** (Lines 76-79):
```cpp
CalibrationData cal_data;
// Verify these match your hardware!
```

**PID Gains** (Lines 160-185):
```cpp
// Start conservative, tune up
float PRateRoll = 1.3;
// ... etc
```

**Safety Limits** (Lines 37-42):
```cpp
#define MAX_LOITER_LEAN 15.0f      // Max tilt in loiter (degrees)
#define GPS_TIMEOUT_MS 2000        // GPS data timeout
#define MIN_SATELLITES 4           // Min sats for loiter
```

**Motor Limits** (Lines 1509-1522):
```cpp
int ThrottleIdle = 1170;           // Idle throttle
int ThrottleCutOff = 1000;         // Motors off
```

---

## Common Issues & Solutions

### Issue: Motors don't spin
**Check**:
- `ReceiverValue[5] > 1500` (armed)
- `ReceiverValue[2] > 1030` (throttle up)
- ESC calibration done

### Issue: Unstable flight
**Check**:
- PID gains (start lower)
- Vibrations (add dampening)
- Calibration (redo on level surface)
- Loop timing (add timing checks)

### Issue: GPS loiter drifts
**Check**:
- Satellite count (`gps.satellites >= 6`)
- Position PID gains
- Magnetometer calibration
- Wind conditions

### Issue: Altitude drifts
**Check**:
- Barometer baseline set at ground
- Kalman filter converged (first 5 seconds)
- Altitude PID gains
- Temperature stability

---

This document should be updated as the code evolves!
