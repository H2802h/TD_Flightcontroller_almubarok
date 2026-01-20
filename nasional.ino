#include <Wire.h>
#include <PulsePosition.h>
#include <math.h>
#include <BasicLinearAlgebra.h>

//////////////////////////////////////////////////////////////////////////////
// HARDWARE DEFINITIONS
//////////////////////////////////////////////////////////////////////////////

// MPU9250 I2C addresses and registers
#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define SMPLRT_DIV 0x19
#define INT_PIN_CFG 0x37
#define WHO_AM_I 0x75
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define MAG_XOUT_L 0x03
#define MAG_CNTL1 0x0A
#define MAG_CNTL2 0x0B

// Pin definitions
#define MOTOR_PIN1 2
#define MOTOR_PIN2 3
#define MOTOR_PIN3 4
#define MOTOR_PIN4 5
#define BUZZER_PIN 6
#define RECEIVER_PIN 10

// Constants
#define GPS_BAUD 9600
#define TELEMETRY_INTERVAL 200
#define BUFFER_SIZE 64
#define MAX_POSITION_ERROR 5.0f
#define MAX_LOITER_LEAN 15.0f
#define GPS_TIMEOUT_MS 2000
#define MIN_SATELLITES 4

//////////////////////////////////////////////////////////////////////////////
// ENUMS AND STRUCTURES
//////////////////////////////////////////////////////////////////////////////

enum FlightMode {
  STABILIZE = 0,
  LOITER = 1,
  ALT_HOLD = 2,
  RTL = 3
};

struct GPSData {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0;
  float ground_speed = 0.0;
  float course = 0.0;
  int satellites = 0;
  bool fix_valid = false;
  unsigned long last_update = 0;
};

struct LoiterPosition {
  double target_lat = 0.0;
  double target_lon = 0.0;
  float target_alt = 0.0;
  bool position_set = false;
  unsigned long set_time = 0;
};

struct CalibrationData {
  float accel_offset_x = -0.0511, accel_offset_y = -0.0116, accel_offset_z = 0.0102;
  float gyro_offset_x = -1.0869, gyro_offset_y = -0.0208, gyro_offset_z = -0.6061;
  float mag_offset_x = 248.50, mag_offset_y = -104.00, mag_offset_z = 12.00;
  float mag_scale_x = 1.0047, mag_scale_y = 1.0567, mag_scale_z = .9449;
};

//////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
//////////////////////////////////////////////////////////////////////////////

// System state
bool isArmed = false;
FlightMode current_mode = STABILIZE;
FlightMode previous_mode = STABILIZE;
uint32_t LoopTimer;

// Sensor objects and data
PulsePositionInput ReceiverInput(RISING);
GPSData gps;
LoiterPosition loiter_pos;
CalibrationData cal_data;

// Sensor scaling and calibration
float accel_scale = 2.0 / 32768.0;
float gyro_scale = 250.0 / 32768.0 * (M_PI / 180.0);
float mag_scale = 4912.0 / 32768.0;

// Receiver data
float ReceiverValue[10] = { 0 };
int ChannelNumber = 0;

// Sensor fusion variables
float MadgwickPitch = 0, MadgwickRoll = 0, MadgwickYaw = 0;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
unsigned long last_update = 0;
float beta = 0.3f;
bool use_magnetometer = false;

// Barometer variables (BME280/BMP280 compatible) - CALIBRATED VALUES
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7, dig_P8, dig_P9;
float AltitudeBarometer, AltitudeBarometerStartUp;

// Improved barometer calibration and filtering
float pressureBaseline = 0.0f;
bool baselineSet = false;
float barometerAlpha = 0.85f;  // Low-pass filter coefficient
float altitudeSmooth = 0.0f;
int barometerSampleCount = 0;
const int BAROMETER_STARTUP_SAMPLES = 200;

// Simple filters
float altitude_smooth = 1.3f;
float velocity_smooth = 0.01f;
float alpha_altitude = 0.95f;
float alpha_velocity = 0.8f;

// Kalman filter matrices for altitude
float AltitudeKalman, VelocityVerticalKalman;
BLA::Matrix<2, 2> F, P, Q, I;
BLA::Matrix<2, 1> G, S, K;
BLA::Matrix<1, 2> H;
BLA::Matrix<1, 1> Acc, R, L, M, L_inv;

// Flight control variables
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float AccX, AccY, AccZ, AccZInertial;
float AngleRoll, AnglePitch;
float DesiredAngleRoll, DesiredAnglePitch, DesiredAngleYaw = 0.0f;
float ErrorAngleRoll, ErrorAnglePitch, ErrorAngleYaw;

// Control inputs
float InputRoll, InputThrottle, InputPitch, InputYaw;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;

// Motor outputs
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// PID controllers - Rate
float PRateRoll = 1.3, IRateRoll = 0.01, DRateRoll = 0.005;
float PRatePitch = 1.3, IRatePitch = 0.01, DRatePitch = 0.003;
float PRateYaw = 3.0, IRateYaw = 0.0, DRateYaw = 0.0;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

// PID controllers - Angle
float PAngleRoll = 2.5, IAngleRoll = 0.1, DAngleRoll = 0.003;
float PAnglePitch = 2.5, IAnglePitch = 0.01, DAnglePitch = 0.003;
float PAngleYaw = 3.0, IAngleYaw = 0.001, DAngleYaw = 0.002;
float PrevErrorAngleRoll, PrevErrorAnglePitch, PrevErrorAngleYaw;
float PrevItermAngleRoll, PrevItermAnglePitch, PrevItermAngleYaw;

// GPS/Position PID controllers
float PPositionX = 0.8, IPositionX = 0.02, DPositionX = 0.1;
float PPositionY = 0.8, IPositionY = 0.02, DPositionY = 0.1;
float ErrorPositionX = 0, ErrorPositionY = 0;
float PrevErrorPositionX = 0, PrevErrorPositionY = 0;
float PrevItermPositionX = 0, PrevItermPositionY = 0;

// Altitude PID controllers - CALIBRATED VALUES
float PAltitude = 2.0f, IAltitude = 0.1f, DAltitude = 0.03f;
float PVelocity = 3.0f, IVelocity = 0.1f, DVelocity = 0.05f;
float DesiredAltitude = 0.0f, ErrorAltitude, PrevErrorAltitude, PrevItermAltitude;
float DesiredVelocityVertical = 0.0f, ErrorVelocityVertical, PrevErrorVelocityVertical, PrevItermVelocityVertical;
float AltitudeThrottleCorrection = 0.0f, BaseThrottle = 0.0f;

// Control parameters
float yaw_deadband = 20.0f;
float yaw_rate_sensitivity = 0.15f;
float throttle_deadband = 50.0f;

// Communication
char SerialBuffer[BUFFER_SIZE];
unsigned long lastTelemetryUpdate = 0;
int bufferIndex = 0;
String gps_sentence = "";
bool sentence_complete = false;
float PIDReturn[] = { 0, 0, 0 };

//////////////////////////////////////////////////////////////////////////////
// SENSOR INITIALIZATION AND READING
//////////////////////////////////////////////////////////////////////////////

bool initMPU9250() {
  uint8_t whoami = readByte(MPU9250_ADDRESS, WHO_AM_I);
  if (whoami != 0x71) return false;

  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);  // Reset
  delay(100);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Clock source
  delay(100);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);   // +/-2g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);    // +/-250dps
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x09);     // 100Hz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);         // 44Hz LPF
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03);  // 44Hz LPF
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);    // Bypass enable
  delay(100);
  return true;
}

bool initMagnetometer() {
  uint8_t whoami = readByte(AK8963_ADDRESS, 0x00);
  if (whoami != 0x48) return false;

  writeByte(AK8963_ADDRESS, MAG_CNTL2, 0x01);  // Reset
  delay(100);
  writeByte(AK8963_ADDRESS, MAG_CNTL1, 0x16);  // 16-bit, 100Hz
  delay(100);
  return true;
}

void initBarometer() {
  // Reset BMP280
  Wire.beginTransmission(0x76);
  Wire.write(0xE0);
  Wire.write(0xB6);  // Reset command
  Wire.endTransmission();
  delay(100);

  // Configure BMP280 with optimized settings for drone use
  Wire.beginTransmission(0x76);
  Wire.write(0xF4);
  Wire.write(0x6F);  // Temperature x2, Pressure x16 oversampling, Normal mode
  Wire.endTransmission();

  Wire.beginTransmission(0x76);
  Wire.write(0xF5);
  Wire.write(0x10);  // Standby 125ms, Filter coefficient 4, SPI disabled
  Wire.endTransmission();

  delay(200);  // Allow sensor to stabilize

  // Read calibration data
  uint8_t data[24], i = 0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 24);

  while (Wire.available() && i < 24) {
    data[i] = Wire.read();
    i++;
  }

  // Parse calibration coefficients
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
}

void initGPS() {
  Serial3.begin(GPS_BAUD);
  delay(1000);

  // Set update rate to 5Hz
  Serial3.println("$PMTK220,200*2C");
  delay(100);

  // Set output to RMC and GGA only
  Serial3.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  delay(100);
}

void writeByte(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readByte(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  return Wire.read();
}

void readAccelerometer(int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6);

  *x = (Wire.read() << 8) | Wire.read();
  *y = (Wire.read() << 8) | Wire.read();
  *z = (Wire.read() << 8) | Wire.read();
}

void readGyroscope(int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6);

  *x = (Wire.read() << 8) | Wire.read();
  *y = (Wire.read() << 8) | Wire.read();
  *z = (Wire.read() << 8) | Wire.read();
}

bool readMagnetometer(int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(AK8963_ADDRESS);
  Wire.write(MAG_XOUT_L);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDRESS, 7);

  if (Wire.available() < 7) return false;

  *x = Wire.read() | (Wire.read() << 8);
  *y = Wire.read() | (Wire.read() << 8);
  *z = Wire.read() | (Wire.read() << 8);
  Wire.read();  // ST2 register
  return true;
}

//////////////////////////////////////////////////////////////////////////////
// SENSOR PROCESSING
//////////////////////////////////////////////////////////////////////////////

void barometer_signals() {
  // Read raw data from BMP280
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 6);

  if (Wire.available() < 6) {
    return;
  }

  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();

  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

  // Validate raw readings
  if (adc_P == 0 || adc_T == 0 || adc_P > 1000000 || adc_T > 1000000) {
    return;
  }

  // Temperature compensation - improved calculation
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;

  // Pressure calculation with improved precision
  unsigned long int p;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);

  if (var1 == 0) {
    return;
  }

  p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12)) * 3125);
  if (p < 0x80000000) {
    p = (p << 1) / ((unsigned long int)var1);
  } else {
    p = (p / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
  p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));

  float pressure = (float)p / 100.0f;  // Convert to hPa

  // Validate pressure range
  if (pressure < 300.0f || pressure > 1200.0f) {
    return;
  }

  // Set baseline during startup
  if (!baselineSet) {
    if (barometerSampleCount < BAROMETER_STARTUP_SAMPLES) {
      pressureBaseline += pressure;
      barometerSampleCount++;
      AltitudeBarometer = 0.0f;
      return;
    } else {
      pressureBaseline /= BAROMETER_STARTUP_SAMPLES;
      baselineSet = true;
    }
  }

  // Calculate altitude using calibrated baseline
  if (baselineSet) {
    // More accurate altitude calculation with temperature compensation
    float pressureRatio = pressure / pressureBaseline;
    float rawAltitude = 4433000.0f * (1.0f - pow(pressureRatio, 0.1903f));  // in cm

    // Apply low-pass filter for smoothing
    if (altitudeSmooth == 0.0f) {
      altitudeSmooth = rawAltitude;
    } else {
      altitudeSmooth = barometerAlpha * altitudeSmooth + (1.0f - barometerAlpha) * rawAltitude;
    }

    AltitudeBarometer = altitudeSmooth;
  }
}

void updatePressureBaseline() {
  static unsigned long lastUpdate = 0;
  static float pressureSum = 0;
  static int sampleCount = 0;

  if (!isArmed || !baselineSet) return;

  if (millis() - lastUpdate > 30000) {  // Check every 30 seconds
    if ((current_mode == LOITER || current_mode == ALT_HOLD) && abs(VelocityVerticalKalman) < 15 && abs(ErrorAltitude) < 30) {

      // Read current pressure directly
      Wire.beginTransmission(0x76);
      Wire.write(0xF7);
      Wire.endTransmission();
      Wire.requestFrom(0x76, 3);

      if (Wire.available() >= 3) {
        uint32_t press_msb = Wire.read();
        uint32_t press_lsb = Wire.read();
        uint32_t press_xlsb = Wire.read();
        unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);

        // Quick pressure calculation for baseline update
        float currentPressure = (float)adc_P / 25600.0f;  // Simplified conversion

        pressureSum += currentPressure;
        sampleCount++;

        if (sampleCount >= 10) {
          float newBaseline = pressureSum / sampleCount;
          pressureBaseline = 0.95f * pressureBaseline + 0.05f * newBaseline;

          pressureSum = 0;
          sampleCount = 0;
        }
      }
    }
    lastUpdate = millis();
  }
}

void kalman_2d() {
  // Improved Kalman filter with better tuning
  Acc = { AccZInertial };
  S = F * S + G * Acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;

  // More robust matrix inversion
  if (L(0, 0) > 0.001f) {
    L_inv = L;
    L_inv(0, 0) = 1.0f / L(0, 0);
    K = P * ~H * L_inv;

    M = { AltitudeBarometer };
    S = S + K * (M - H * S);
    P = (I - K * H) * P;

    AltitudeKalman = S(0, 0);
    VelocityVerticalKalman = S(1, 0);

    // Apply reasonable limits
    AltitudeKalman = constrain(AltitudeKalman, -200.0f, 2000.0f);
    VelocityVerticalKalman = constrain(VelocityVerticalKalman, -200.0f, 200.0f);
  }
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void madgwickAHRS6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void quaternionToEuler() {
  MadgwickRoll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
  MadgwickPitch = asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
  MadgwickYaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}

void sensor_signals() {
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  int16_t mx_raw, my_raw, mz_raw;

  readAccelerometer(&ax_raw, &ay_raw, &az_raw);
  readGyroscope(&gx_raw, &gy_raw, &gz_raw);

  bool mag_valid = false;
  if (use_magnetometer) {
    mag_valid = readMagnetometer(&mx_raw, &my_raw, &mz_raw);
  }

  // Apply calibration and scaling
  float ax = (ax_raw + cal_data.accel_offset_x) * accel_scale;
  float ay = (ay_raw + cal_data.accel_offset_y) * accel_scale;
  float az = (az_raw + cal_data.accel_offset_z) * accel_scale;

  float gx = (gx_raw + cal_data.gyro_offset_x) * gyro_scale;
  float gy = (gy_raw + cal_data.gyro_offset_y) * gyro_scale;
  float gz = (gz_raw + cal_data.gyro_offset_z) * gyro_scale;

  unsigned long now = micros();
  float dt = (now - last_update) / 1000000.0f;
  last_update = now;

  // Use 6DOF AHRS (magnetometer disabled for stability)
  madgwickAHRS6DOF(gx, gy, gz, ax, ay, az, dt);
  quaternionToEuler();

  AngleRoll = MadgwickRoll;
  AnglePitch = MadgwickPitch;

  RateRoll = gx * (180.0 / M_PI);
  RatePitch = gy * (180.0 / M_PI);
  RateYaw = gz * (180.0 / M_PI);

  AccX = ax;
  AccY = ay;
  AccZ = az;

  // Calculate inertial Z acceleration for Kalman filter - IMPROVED
  float rollRad = AngleRoll * (M_PI / 180.0f);
  float pitchRad = AnglePitch * (M_PI / 180.0f);

  // More robust inertial frame transformation
  if (abs(AngleRoll) < 45.0f && abs(AnglePitch) < 45.0f) {
    AccZInertial = AccZ * cos(rollRad) * cos(pitchRad) - AccX * sin(pitchRad) + AccY * sin(rollRad) * cos(pitchRad);
  } else {
    // Use simpler calculation for large angles
    AccZInertial = AccZ;
  }

  // Remove 1G and convert to cm/s² with calibration
  AccZInertial = (AccZInertial - 1.0f) * 981.0f;

  // Apply low-pass filter to reduce noise
  static float AccZInertial_prev = 0.0f;
  AccZInertial = 0.8f * AccZInertial_prev + 0.2f * AccZInertial;
  AccZInertial_prev = AccZInertial;
}

//////////////////////////////////////////////////////////////////////////////
// GPS PROCESSING
//////////////////////////////////////////////////////////////////////////////

void parseGPS() {
  while (Serial3.available()) {
    char c = Serial3.read();

    if (c == '\n') {
      sentence_complete = true;
    } else if (c != '\r') {
      gps_sentence += c;
    }

    if (sentence_complete) {
      if (gps_sentence.startsWith("$GPRMC") || gps_sentence.startsWith("$GNRMC")) {
        parseGPRMC(gps_sentence);
      } else if (gps_sentence.startsWith("$GPGGA") || gps_sentence.startsWith("$GNGGA")) {
        parseGPGGA(gps_sentence);
      }

      gps_sentence = "";
      sentence_complete = false;
    }
  }
}

void parseGPRMC(String sentence) {
  int commaCount = 0;
  String fields[15] = { "" };

  for (unsigned int i = 0; i < sentence.length(); i++) {
    if (sentence[i] == ',') {
      commaCount++;
      if (commaCount < 15) fields[commaCount] = "";
    } else if (commaCount < 15) {
      fields[commaCount] += sentence[i];
    }
  }

  if (commaCount < 9 || fields[3].length() == 0 || fields[5].length() == 0) return;
  if (fields[2] != "A") return;  // No valid fix

  // Parse latitude
  float lat_deg = fields[3].substring(0, 2).toFloat();
  float lat_min = fields[3].substring(2).toFloat();
  if (lat_deg >= 0 && lat_deg <= 90 && lat_min >= 0 && lat_min < 60.0) {
    gps.latitude = (double)(lat_deg + lat_min / 60.0);
    if (fields[4] == "S") gps.latitude = -gps.latitude;
  } else {
    return;
  }

  // Parse longitude
  float lon_deg = fields[5].substring(0, 3).toFloat();
  float lon_min = fields[5].substring(3).toFloat();
  if (lon_deg >= 0 && lon_deg <= 180 && lon_min >= 0 && lon_min < 60.0) {
    gps.longitude = (double)(lon_deg + lon_min / 60.0);
    if (fields[6] == "W") gps.longitude = -gps.longitude;
  } else {
    return;
  }

  gps.ground_speed = fields[7].toFloat() * 0.514444;  // knots to m/s
  gps.course = fields[8].toFloat();
  gps.fix_valid = true;
  gps.last_update = millis();
}

void parseGPGGA(String sentence) {
  int commaCount = 0;
  String fields[15] = { "" };

  for (int i = 0; i < sentence.length(); i++) {
    if (sentence[i] == ',') {
      commaCount++;
    } else if (commaCount < 15) {
      fields[commaCount] += sentence[i];
    }
  }

  if (commaCount >= 9) {
    gps.satellites = fields[7].toInt();
    if (fields[9].length() > 0) {
      gps.altitude = fields[9].toFloat() * 100;  // Convert to cm
    }
  }
}

bool isGPSValid() {
  return (gps.fix_valid && gps.satellites >= MIN_SATELLITES && (millis() - gps.last_update) < GPS_TIMEOUT_MS);
}

float calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000;  // Earth radius in meters
  float dLat = (lat2 - lat1) * M_PI / 180.0;
  float dLon = (lon2 - lon1) * M_PI / 180.0;
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

float calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = (lon2 - lon1) * M_PI / 180.0;
  float y = sin(dLon) * cos(lat2 * M_PI / 180.0);
  float x = cos(lat1 * M_PI / 180.0) * sin(lat2 * M_PI / 180.0) - sin(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * cos(dLon);
  float bearing = atan2(y, x) * 180.0 / M_PI;
  return fmod((bearing + 360.0), 360.0);
}

void convertGPSToLocal(double target_lat, double target_lon, float* x_error, float* y_error) {
  float distance = calculateDistance(gps.latitude, gps.longitude, target_lat, target_lon);
  float bearing = calculateBearing(gps.latitude, gps.longitude, target_lat, target_lon);

  *y_error = distance * cos(bearing * M_PI / 180.0);
  *x_error = distance * sin(bearing * M_PI / 180.0);
}

//////////////////////////////////////////////////////////////////////////////
// RECEIVER INPUT
//////////////////////////////////////////////////////////////////////////////

void read_receiver() {
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
    int maxChannel = min(ChannelNumber, 10);
    for (int i = 1; i <= maxChannel; i++) {
      ReceiverValue[i - 1] = ReceiverInput.read(i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// FLIGHT CONTROL FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;

  // More conservative I-term calculation
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;

  // Dynamic I-term limiting based on error and system state
  float ILimit = 100.0f;  // Much more conservative base limit

  // Reduce I-term when error is large or changing rapidly
  if (abs(Error) > 30.0f) ILimit = 50.0f;
  if (abs(Error - PrevError) > 10.0f) ILimit = 25.0f;  // Reduce during rapid changes

  Iterm = constrain(Iterm, -ILimit, ILimit);

  // Add derivative filtering to reduce noise
  static float prevDterm = 0.0f;
  float Dterm = D * (Error - PrevError) / 0.004;
  Dterm = 0.8f * prevDterm + 0.2f * Dterm;  // Low-pass filter
  prevDterm = Dterm;
  float PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = constrain(PIDOutput, -300, 300);  // More conservative output limit

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid() {
  PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
  PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
  PrevErrorAngleRoll = PrevErrorAnglePitch = PrevErrorAngleYaw = 0;
  PrevItermAngleRoll = PrevItermAnglePitch = PrevItermAngleYaw = 0;
}

void resetPositionPID() {
  ErrorPositionX = ErrorPositionY = 0;
  PrevErrorPositionX = PrevErrorPositionY = 0;
  PrevItermPositionX = PrevItermPositionY = 0;
}

void resetAltitudePID() {
  ErrorAltitude = PrevErrorAltitude = PrevItermAltitude = 0;
  ErrorVelocityVertical = PrevErrorVelocityVertical = PrevItermVelocityVertical = 0;
  AltitudeThrottleCorrection = 0;
}

//////////////////////////////////////////////////////////////////////////////
// FLIGHT MODES
//////////////////////////////////////////////////////////////////////////////

void handleModeSwitch() {
  float mode_input = ReceiverValue[7];  // Channel 8

  FlightMode new_mode = STABILIZE;
  if (mode_input < 1300) {
    new_mode = STABILIZE;
  } else if (mode_input < 1500) {
    new_mode = ALT_HOLD;
  } else if (mode_input > 1700) {
    new_mode = LOITER;
  } else {
    new_mode = RTL;
  }

  if (new_mode != current_mode) {
    previous_mode = current_mode;
    current_mode = new_mode;
    onModeChange();
  }
}

void onModeChange() {
  switch (current_mode) {
    case STABILIZE:
      reset_pid();
      break;

    case ALT_HOLD:
      DesiredAltitude = AltitudeKalman;
      BaseThrottle = ReceiverValue[2];
      reset_pid();
      resetAltitudePID();
      tone(BUZZER_PIN, 1200, 200);
      break;

    case LOITER:
      if (isGPSValid()) {
        setLoiterPosition();
        DesiredAltitude = AltitudeKalman;
        BaseThrottle = ReceiverValue[2];
        reset_pid();
        resetPositionPID();
        resetAltitudePID();
        tone(BUZZER_PIN, 1200, 200);
      } else {
        current_mode = ALT_HOLD;
        DesiredAltitude = AltitudeKalman;
        BaseThrottle = ReceiverValue[2];
        resetAltitudePID();
      }
      break;

    case RTL:
      current_mode = STABILIZE;
      break;
  }
}

void setLoiterPosition() {
  if (isGPSValid()) {
    loiter_pos.target_lat = gps.latitude;
    loiter_pos.target_lon = gps.longitude;
    loiter_pos.target_alt = AltitudeKalman;
    loiter_pos.position_set = true;
    loiter_pos.set_time = millis();
  }
}

void processLoiterMode() {
  if (!isGPSValid() || !loiter_pos.position_set) {
    current_mode = ALT_HOLD;
    return;
  }

  float x_error, y_error;
  convertGPSToLocal(loiter_pos.target_lat, loiter_pos.target_lon, &x_error, &y_error);

  x_error = constrain(x_error, -MAX_POSITION_ERROR, MAX_POSITION_ERROR);
  y_error = constrain(y_error, -MAX_POSITION_ERROR, MAX_POSITION_ERROR);

  pid_equation(x_error, PPositionX, IPositionX, DPositionX, PrevErrorPositionX, PrevItermPositionX);
  float desired_roll = -PIDReturn[0];
  PrevErrorPositionX = PIDReturn[1];
  PrevItermPositionX = PIDReturn[2];

  pid_equation(y_error, PPositionY, IPositionY, DPositionY, PrevErrorPositionY, PrevItermPositionY);
  float desired_pitch = PIDReturn[0];
  PrevErrorPositionY = PIDReturn[1];
  PrevItermPositionY = PIDReturn[2];

  desired_roll = constrain(desired_roll, -MAX_LOITER_LEAN, MAX_LOITER_LEAN);
  desired_pitch = constrain(desired_pitch, -MAX_LOITER_LEAN, MAX_LOITER_LEAN);

  DesiredAngleRoll = desired_roll;
  DesiredAnglePitch = desired_pitch;

  ErrorPositionX = x_error;
  ErrorPositionY = y_error;
}

void processAltitudeControl() {
  if (current_mode != ALT_HOLD && current_mode != LOITER) return;

  // Manual altitude adjustments with REDUCED sensitivity for stability
  float throttle_input = ReceiverValue[2];
  float throttle_center = 1500.0f;
  float altitude_adjustment = 0;

  if (throttle_input > (throttle_center + throttle_deadband)) {
    altitude_adjustment = (throttle_input - throttle_center - throttle_deadband) * 0.03f;  // Reduced from 0.08f
  } else if (throttle_input < (throttle_center - throttle_deadband)) {
    altitude_adjustment = (throttle_input - throttle_center + throttle_deadband) * 0.03f;  // Reduced from 0.08f
  }

  DesiredAltitude += altitude_adjustment * 0.004f;
  DesiredAltitude = constrain(DesiredAltitude, -50.0f, 500.0f);  // Tighter limits

  // Altitude control PID with IMPROVED stability
  ErrorAltitude = DesiredAltitude - AltitudeKalman;
  
  // Add deadband to reduce small oscillations
  if (abs(ErrorAltitude) < 2.0f) {
    ErrorAltitude = 0.0f;
  }
  
  pid_equation(ErrorAltitude, PAltitude, IAltitude, DAltitude, PrevErrorAltitude, PrevItermAltitude);
  DesiredVelocityVertical = PIDReturn[0];
  PrevErrorAltitude = PIDReturn[1];
  PrevItermAltitude = PIDReturn[2];

  DesiredVelocityVertical = constrain(DesiredVelocityVertical, -80.0f, 80.0f);  // Reduced from 150

  // Velocity control PID with deadband
  ErrorVelocityVertical = DesiredVelocityVertical - VelocityVerticalKalman;
  
  // Add velocity deadband
  if (abs(ErrorVelocityVertical) < 5.0f) {
    ErrorVelocityVertical = 0.0f;
  }
  
  pid_equation(ErrorVelocityVertical, PVelocity, IVelocity, DVelocity, PrevErrorVelocityVertical, PrevItermVelocityVertical);
  AltitudeThrottleCorrection = PIDReturn[0];
  PrevErrorVelocityVertical = PIDReturn[1];
  PrevItermVelocityVertical = PIDReturn[2];

  // MUCH more conservative throttle correction
  AltitudeThrottleCorrection = constrain(AltitudeThrottleCorrection, -150.0f, 150.0f);  // Reduced from 250
  
  // Apply additional smoothing to throttle correction
  static float prevThrottleCorrection = 0.0f;
  AltitudeThrottleCorrection = 0.7f * prevThrottleCorrection + 0.3f * AltitudeThrottleCorrection;
  prevThrottleCorrection = AltitudeThrottleCorrection;
  
  InputThrottle = BaseThrottle + AltitudeThrottleCorrection;
}

//////////////////////////////////////////////////////////////////////////////
// ARMING AND SAFETY
//////////////////////////////////////////////////////////////////////////////

void handleArming() {
  // Arm: Throttle low, Yaw right
  if (!isArmed && ReceiverValue[2] < 1050 && ReceiverValue[3] > 1900) {
    isArmed = true;
    reset_pid();
    resetPositionPID();
    resetAltitudePID();
    DesiredAngleYaw = MadgwickYaw;
    playArmingBeep();
  }

  // Disarm: Throttle low, Yaw left
  if (isArmed && ReceiverValue[2] < 1050 && ReceiverValue[3] < 1100) {
    isArmed = false;
    current_mode = STABILIZE;
    tone(BUZZER_PIN, 500, 200);
  }
}

void playArmingBeep() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 1000, 100);
    delay(150);
  }
}

void applyMotorSafety() {
  if (!isArmed) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = 1000;
    analogWrite(MOTOR_PIN1, MotorInput1);
    analogWrite(MOTOR_PIN2, MotorInput2);
    analogWrite(MOTOR_PIN3, MotorInput3);
    analogWrite(MOTOR_PIN4, MotorInput4);
  }
}

//////////////////////////////////////////////////////////////////////////////
// COMMUNICATION
//////////////////////////////////////////////////////////////////////////////

void handleSerial() {
  while (Serial4.available() > 0) {
    char c = Serial4.read();
    if (c == '\n') {
      SerialBuffer[bufferIndex] = '\0';
      processPIDCommand(SerialBuffer);
      bufferIndex = 0;
    } else if (bufferIndex < BUFFER_SIZE - 1) {
      SerialBuffer[bufferIndex++] = c;
    }
  }
}

void processPIDCommand(char* data) {
  char* token = strtok(data, ",");
  if (token == NULL) return;

  if (strcmp(token, "CONNECT") == 0) {
    Serial4.println("DRONE_READY");
  } else if (strcmp(token, "GET_RATEPID") == 0) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "RATEPID,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             PRateRoll, IRateRoll, DRateRoll, PRatePitch, IRatePitch, DRatePitch,
             PRateYaw, IRateYaw, DRateYaw);
    Serial4.println(buffer);
  } else if (strcmp(token, "RATEPID") == 0) {
    token = strtok(NULL, ",");
    if (token) PRateRoll = atof(token);
    token = strtok(NULL, ",");
    if (token) IRateRoll = atof(token);
    token = strtok(NULL, ",");
    if (token) DRateRoll = atof(token);
    token = strtok(NULL, ",");
    if (token) PRatePitch = atof(token);
    token = strtok(NULL, ",");
    if (token) IRatePitch = atof(token);
    token = strtok(NULL, ",");
    if (token) DRatePitch = atof(token);
    token = strtok(NULL, ",");
    if (token) PRateYaw = atof(token);
    token = strtok(NULL, ",");
    if (token) IRateYaw = atof(token);
    token = strtok(NULL, ",");
    if (token) DRateYaw = atof(token);
    Serial4.println("PID_OK");
  }
  // Add other PID commands as needed...
}

//////////////////////////////////////////////////////////////////////////////
// OPTIMIZED TELEMETRY SYSTEM
//////////////////////////////////////////////////////////////////////////////

void sendTelemetry() {
  static unsigned long lastFast = 0;
  static unsigned long lastMedium = 0;
  static unsigned long lastSlow = 0;
  static unsigned long lastVerySlow = 0;
  unsigned long now = millis();
  
  // FAST DATA - 20Hz (every 50ms) - Critical flight data
  if (now - lastFast >= 50) {
    lastFast = now;
    sendFlightData();
  }
  
  // MEDIUM DATA - 5Hz (every 200ms) - Motors and control inputs
  if (now - lastMedium >= 200) {
    lastMedium = now;
    sendMotorData();
  }
  
  // SLOW DATA - 2Hz (every 500ms) - GPS and altitude
  if (now - lastSlow >= 500) {
    lastSlow = now;
    sendGPSData();
    sendAltitudeData();
  }
  
  // VERY SLOW DATA - 0.2Hz (every 5 seconds) - PID params and RC channels
  if (now - lastVerySlow >= 5000) {
    lastVerySlow = now;
    sendPIDParams();
    sendRCChannels();
    sendStatusData();  // ← TAMBAHKAN INI
  }
}

void sendFlightData() {
  // Critical flight data - compressed keys
  Serial4.print("{\"t\":\"F\",");
  Serial4.print("\"r\":"); Serial4.print(AngleRoll, 1); Serial4.print(",");
  Serial4.print("\"p\":"); Serial4.print(AnglePitch, 1); Serial4.print(",");
  Serial4.print("\"y\":"); Serial4.print(MadgwickYaw, 1); Serial4.print(",");
  Serial4.print("\"dr\":"); Serial4.print(DesiredAngleRoll, 1); Serial4.print(",");
  Serial4.print("\"dp\":"); Serial4.print(DesiredAnglePitch, 1); Serial4.print(",");
  Serial4.print("\"dy\":"); Serial4.print(DesiredAngleYaw, 1); Serial4.print(",");
  Serial4.print("\"rr\":"); Serial4.print(RateRoll, 1); Serial4.print(",");
  Serial4.print("\"rp\":"); Serial4.print(RatePitch, 1); Serial4.print(",");
  Serial4.print("\"ry\":"); Serial4.print(RateYaw, 1); Serial4.print(",");
  Serial4.print("\"mode\":"); Serial4.print((int)current_mode); Serial4.print(",");
  Serial4.print("\"arm\":"); Serial4.print(isArmed ? 1 : 0);
  Serial4.println("}");
}

void sendMotorData() {
  // Motor outputs and throttle
  Serial4.print("{\"t\":\"MOT\",");
  Serial4.print("\"m1\":"); Serial4.print((int)MotorInput1); Serial4.print(",");
  Serial4.print("\"m2\":"); Serial4.print((int)MotorInput2); Serial4.print(",");
  Serial4.print("\"m3\":"); Serial4.print((int)MotorInput3); Serial4.print(",");
  Serial4.print("\"m4\":"); Serial4.print((int)MotorInput4); Serial4.print(",");
  Serial4.print("\"thr\":"); Serial4.print((int)InputThrottle);
  Serial4.println("}");
}

void sendGPSData() {
  // GPS information
  Serial4.print("{\"t\":\"GPS\",");
  Serial4.print("\"fix\":"); Serial4.print(gps.fix_valid ? 1 : 0); Serial4.print(",");
  Serial4.print("\"sat\":"); Serial4.print(gps.satellites); Serial4.print(",");
  Serial4.print("\"lat\":"); Serial4.print(gps.latitude, 6); Serial4.print(",");
  Serial4.print("\"lon\":"); Serial4.print(gps.longitude, 6); Serial4.print(",");
  Serial4.print("\"spd\":"); Serial4.print(gps.ground_speed, 2); Serial4.print(",");
  Serial4.print("\"crs\":"); Serial4.print(gps.course, 1);
  Serial4.println("}");
}

void sendAltitudeData() {
  // Altitude and vertical velocity
  Serial4.print("{\"t\":\"ALT\",");
  Serial4.print("\"alt\":"); Serial4.print(AltitudeKalman, 1); Serial4.print(",");
  Serial4.print("\"vz\":"); Serial4.print(VelocityVerticalKalman, 1); Serial4.print(",");
  Serial4.print("\"dalt\":"); Serial4.print(DesiredAltitude, 1); Serial4.print(",");
  Serial4.print("\"baro\":"); Serial4.print(AltitudeBarometer, 1);
  Serial4.println("}");
}

void sendPIDParams() {
  // PID parameters - only sent every 5 seconds
  Serial4.print("{\"t\":\"PID\",");
  Serial4.print("\"PR\":"); Serial4.print(PRateRoll, 2); Serial4.print(",");
  Serial4.print("\"IR\":"); Serial4.print(IRateRoll, 3); Serial4.print(",");
  Serial4.print("\"DR\":"); Serial4.print(DRateRoll, 3); Serial4.print(",");
  Serial4.print("\"PAR\":"); Serial4.print(PAngleRoll, 2); Serial4.print(",");
  Serial4.print("\"IAR\":"); Serial4.print(IAngleRoll, 3); Serial4.print(",");
  Serial4.print("\"DAR\":"); Serial4.print(DAngleRoll, 3); Serial4.print(",");
  Serial4.print("\"PP\":"); Serial4.print(PRatePitch, 2); Serial4.print(",");
  Serial4.print("\"IP\":"); Serial4.print(IRatePitch, 3); Serial4.print(",");
  Serial4.print("\"DP\":"); Serial4.print(DRatePitch, 3); Serial4.print(",");
  Serial4.print("\"PAP\":"); Serial4.print(PAnglePitch, 2); Serial4.print(",");
  Serial4.print("\"IAP\":"); Serial4.print(IAnglePitch, 3); Serial4.print(",");
  Serial4.print("\"DAP\":"); Serial4.print(DAnglePitch, 3); Serial4.print(",");
  Serial4.print("\"PY\":"); Serial4.print(PRateYaw, 2); Serial4.print(",");
  Serial4.print("\"IY\":"); Serial4.print(IRateYaw, 3); Serial4.print(",");
  Serial4.print("\"DY\":"); Serial4.print(DRateYaw, 3); Serial4.print(",");
  Serial4.print("\"PAY\":"); Serial4.print(PAngleYaw, 2); Serial4.print(",");
  Serial4.print("\"IAY\":"); Serial4.print(IAngleYaw, 3); Serial4.print(",");
  Serial4.print("\"DAY\":"); Serial4.print(DAngleYaw, 3);
  Serial4.println("}");
}

void sendRCChannels() {
  // RC channels - only first 8 channels
  Serial4.print("{\"t\":\"RC\",");
  Serial4.print("\"ch\":[");
  for (int i = 0; i < 8; i++) {
    Serial4.print((int)ReceiverValue[i]);
    if (i < 7) Serial4.print(",");
  }
  Serial4.println("]}");
}

void sendStatusData() {
  // Status update dengan semua parameter penting
  Serial4.print("{\"t\":\"STATUS\",");
  
  // System
  Serial4.print("\"mode\":"); Serial4.print((int)current_mode); Serial4.print(",");
  Serial4.print("\"arm\":"); Serial4.print(isArmed ? 1 : 0); Serial4.print(",");
  Serial4.print("\"loop\":"); Serial4.print(LoopTimer); Serial4.print(",");
  
  // Attitude
  Serial4.print("\"AngleRoll\":"); Serial4.print(AngleRoll, 2); Serial4.print(",");
  Serial4.print("\"AnglePitch\":"); Serial4.print(AnglePitch, 2); Serial4.print(",");
  Serial4.print("\"MadgwickYaw\":"); Serial4.print(MadgwickYaw, 2); Serial4.print(",");
  
  // Rates
  Serial4.print("\"RateRoll\":"); Serial4.print(RateRoll, 2); Serial4.print(",");
  Serial4.print("\"RatePitch\":"); Serial4.print(RatePitch, 2); Serial4.print(",");
  Serial4.print("\"RateYaw\":"); Serial4.print(RateYaw, 2); Serial4.print(",");
  
  // Accelerometer
  Serial4.print("\"AccX\":"); Serial4.print(AccX, 3); Serial4.print(",");
  Serial4.print("\"AccY\":"); Serial4.print(AccY, 3); Serial4.print(",");
  Serial4.print("\"AccZ\":"); Serial4.print(AccZ, 3); Serial4.print(",");
  
  // Kalman (jika ada)
  Serial4.print("\"KalmanAngleRoll\":"); Serial4.print(AngleRoll, 2); Serial4.print(",");
  Serial4.print("\"KalmanAnglePitch\":"); Serial4.print(AnglePitch, 2); Serial4.print(",");
  
  // Altitude
  Serial4.print("\"VelocityVertical\":"); Serial4.print(VelocityVerticalKalman, 1); Serial4.print(",");
  Serial4.print("\"DesiredAlt\":"); Serial4.print(DesiredAltitude, 1);
  
  Serial4.println("}");
}


void processCommand(char* cmd) {
  // Connection check
  if (strcmp(cmd, "CONNECT") == 0) {
    Serial4.println("{\"t\":\"ACK\",\"msg\":\"READY\"}");
  }
  // Request full status
  else if (strcmp(cmd, "GET_STATUS") == 0) {
    sendStatusData();
  }
  // Request PID parameters
  else if (strcmp(cmd, "GET_PID") == 0) {
    sendPIDParams();
  }
  // Request GPS data
  else if (strcmp(cmd, "GET_GPS") == 0) {
    sendGPSData();
  }
  // Set PID parameters
  else if (strncmp(cmd, "SET_PID_", 8) == 0) {
    processPIDUpdate(cmd);
  }
}

void processPIDUpdate(char* cmd) {
  // Format: SET_PID_PR:1.5 or SET_PID_IR:0.02
  char* colon = strchr(cmd, ':');
  if (!colon) return;
  
  *colon = '\0';
  char* param = cmd + 8;  // Skip "SET_PID_"
  float value = atof(colon + 1);
  
  // Update appropriate PID value
  if (strcmp(param, "PR") == 0) PRateRoll = value;
  else if (strcmp(param, "IR") == 0) IRateRoll = value;
  else if (strcmp(param, "DR") == 0) DRateRoll = value;
  else if (strcmp(param, "PAR") == 0) PAngleRoll = value;
  else if (strcmp(param, "IAR") == 0) IAngleRoll = value;
  else if (strcmp(param, "DAR") == 0) DAngleRoll = value;
  else if (strcmp(param, "PP") == 0) PRatePitch = value;
  else if (strcmp(param, "IP") == 0) IRatePitch = value;
  else if (strcmp(param, "DP") == 0) DRatePitch = value;
  else if (strcmp(param, "PAP") == 0) PAnglePitch = value;
  else if (strcmp(param, "IAP") == 0) IAnglePitch = value;
  else if (strcmp(param, "DAP") == 0) DAnglePitch = value;
  else if (strcmp(param, "PY") == 0) PRateYaw = value;
  else if (strcmp(param, "IY") == 0) IRateYaw = value;
  else if (strcmp(param, "DY") == 0) DRateYaw = value;
  else if (strcmp(param, "PAY") == 0) PAngleYaw = value;
  else if (strcmp(param, "IAY") == 0) IAngleYaw = value;
  else if (strcmp(param, "DAY") == 0) DAngleYaw = value;
  
  Serial4.print("{\"t\":\"ACK\",\"param\":\"");
  Serial4.print(param);
  Serial4.print("\",\"val\":");
  Serial4.print(value, 3);
  Serial4.println("}");
}

//////////////////////////////////////////////////////////////////////////////
// SETUP AND MAIN LOOP
//////////////////////////////////////////////////////////////////////////////

void setup() {
  // Pin initialization
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Serial initialization
  Serial.begin(115200);
  Serial4.begin(115200);

  // Receiver initialization
  ReceiverInput.begin(RECEIVER_PIN);
  read_receiver();

  // I2C initialization
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // MPU9250 initialization
  if (!initMPU9250()) {
    while (1)
      ;
  }

  if (!initMagnetometer()) {
    use_magnetometer = false;
  }

  // Barometer initialization with calibration
  initBarometer();

  // GPS initialization
  initGPS();

  // Kalman filter initialization with tuned parameters
  H = { 1.0f, 0.0f };  // Observe altitude directly, not velocity
  I = { 1.0f, 0.0f,
        0.0f, 1.0f };

  // Very conservative noise parameters
  Q = { 0.05f, 0.0f,
        0.0f, 0.05f };
  R = { 100.0f };

  // Initial state
  P = { 5.0f, 0.0f,
        0.0f, 5.0f };
  S = { 0.0f, 0.0f };

  // Initialize Kalman filter matrices for 250Hz operation
  float dt = 0.004f;  // 4ms = 250Hz
  F = { 1.0f, dt, 0.0f, 1.0f };
  G = { 0.5f * dt * dt, dt };

  // Sensor calibration with improved barometer handling
  int calibration_samples = 3000;
  for (RateCalibrationNumber = 0; RateCalibrationNumber < calibration_samples; RateCalibrationNumber++) {
    sensor_signals();
    barometer_signals();

    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AltitudeBarometerStartUp += AltitudeBarometer;

    delay(1);
  }

  RateCalibrationRoll /= calibration_samples;
  RateCalibrationPitch /= calibration_samples;
  RateCalibrationYaw /= calibration_samples;
  AltitudeBarometerStartUp /= calibration_samples;

  // Motor PWM setup
  analogWriteFrequency(MOTOR_PIN1, 250);
  analogWriteFrequency(MOTOR_PIN2, 250);
  analogWriteFrequency(MOTOR_PIN3, 250);
  analogWriteFrequency(MOTOR_PIN4, 250);
  analogWriteResolution(12);

  LoopTimer = micros();
  last_update = micros();
  digitalWrite(13, LOW);
}

void loop() {
  // Core sensor processing
  sensor_signals();
  barometer_signals();
  AltitudeBarometer -= AltitudeBarometerStartUp;
  kalman_2d();
  updatePressureBaseline();

  // if (baselineSet) {
  //   kalman_2d();

  //   // Apply additional smoothing to velocity
  //   velocity_smooth = alpha_velocity * velocity_smooth + (1.0f - alpha_velocity) * VelocityVerticalKalman;
  // }
  Serial.print("Altitude [cm]: ");
  Serial.print(AltitudeKalman);
  Serial.print(" Vertical velocity [cm/s]: ");
  Serial.println(VelocityVerticalKalman);
  // GPS and communication
  parseGPS();
  handleSerial();
  read_receiver();
  // Safety and mode handling
  handleArming();
  handleModeSwitch();

  // Early exit if disarmed
  if (!isArmed) {
    applyMotorSafety();
    if (millis() - lastTelemetryUpdate >= TELEMETRY_INTERVAL) {
      lastTelemetryUpdate = millis();
      sendTelemetry();
    }
    while (micros() - LoopTimer < 4000)
      ;
    LoopTimer = micros();
    return;
  }

  // Apply gyro calibration
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Flight mode control
  if (current_mode == LOITER) {
    processLoiterMode();
  } else {
    DesiredAngleRoll = 0.10 * (ReceiverValue[0] - 1500);
    DesiredAnglePitch = 0.10 * (ReceiverValue[1] - 1500);
  }

  processAltitudeControl();

  // Set throttle for non-altitude modes
  if (current_mode == STABILIZE) {
    InputThrottle = ReceiverValue[2];
  }

  // Yaw control with improved handling
  float yaw_input = ReceiverValue[3] - 1500;
  if (abs(yaw_input) > yaw_deadband) {
    DesiredRateYaw = yaw_rate_sensitivity * yaw_input;
    DesiredAngleYaw += DesiredRateYaw * 0.004f;
    // Wrap angle
    while (DesiredAngleYaw > 180.0f) DesiredAngleYaw -= 360.0f;
    while (DesiredAngleYaw < -180.0f) DesiredAngleYaw += 360.0f;
  } else {
    ErrorAngleYaw = DesiredAngleYaw - MadgwickYaw;
    while (ErrorAngleYaw > 180.0f) ErrorAngleYaw -= 360.0f;
    while (ErrorAngleYaw < -180.0f) ErrorAngleYaw += 360.0f;
    pid_equation(ErrorAngleYaw, PAngleYaw, IAngleYaw, DAngleYaw, PrevErrorAngleYaw, PrevItermAngleYaw);
    DesiredRateYaw = PIDReturn[0];
    PrevErrorAngleYaw = PIDReturn[1];
    PrevItermAngleYaw = PIDReturn[2];
  }

  // Angle control loops
  ErrorAngleRoll = DesiredAngleRoll - AngleRoll;
  ErrorAnglePitch = DesiredAnglePitch - AnglePitch;

  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAnglePitch = PIDReturn[2];

  // Rate control loops
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  // Motor mixing (X configuration)
  InputThrottle = constrain(InputThrottle, 0, 1800);
  MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
  MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
  MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
  MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);

  // Apply limits with improved safety
  int ThrottleIdle = 1170;
  int ThrottleCutOff = 1000;

  if (ReceiverValue[2] < 1030) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    reset_pid();
    resetPositionPID();
    resetAltitudePID();
  } else {
    MotorInput1 = constrain(MotorInput1, ThrottleIdle, 2000);
    MotorInput2 = constrain(MotorInput2, ThrottleIdle, 2000);
    MotorInput3 = constrain(MotorInput3, ThrottleIdle, 2000);
    MotorInput4 = constrain(MotorInput4, ThrottleIdle, 2000);
  }

  // Apply motor outputs
  analogWrite(MOTOR_PIN1, MotorInput1);
  analogWrite(MOTOR_PIN2, MotorInput2);
  analogWrite(MOTOR_PIN3, MotorInput3);
  analogWrite(MOTOR_PIN4, MotorInput4);

  // Send telemetry
  if (millis() - lastTelemetryUpdate >= TELEMETRY_INTERVAL) {
    lastTelemetryUpdate = millis();
    sendTelemetry();
  }

  // Maintain loop timing (250Hz)
  while (micros() - LoopTimer < 4000)
    ;
  LoopTimer = micros();
}