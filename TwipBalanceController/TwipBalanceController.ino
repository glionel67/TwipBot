/**
 * \file TwipBalanceController.ino
 * \brief Twip balance control
 * \author Lionel GENEVE
 * \date 01/01/2021
 * \version 1.0
 */

// ---------- HEADERS ----------
#include <avr/interrupt.h> 
#include <avr/io.h>
#include <math.h>

#include "Com.h"

#define DISABLE_MPU9250_FIFO
#include "mpu9250.h"

#include <MadgwickAHRS.h>

// ---------- DEFINES ----------
#define RX_BUFFER_SIZE (25)
#define RPM_2_RADS (M_PI / 30.f)
#define WHEEL_RADIUS (0.045f) // [m]
#define WHEELBASE (0.135f) // [m]

// ---------- STRUCTS ----------
enum { LEFT=0, RIGHT, N_MOTORS };

// ---------- GLOBAL VARIABLES ----------
// UART communication
static uint8_t rx_buffer[RX_BUFFER_SIZE] = {0, };
static uint8_t rx_in = 0, rx_ct = 0, rx_flag = 0;
static uint8_t nb_rx_data = 0;

// Command variables
int16_t rpm_ref[N_MOTORS] = {0, }; // [rpm]
uint8_t control_mode = 1; // 0 = MANUAL, 1 = AUTOMATIC

float linear_speed_target_ms = 0.f;
float angular_speed_target_rads = 0.f;
float linear_speed_meas_ms = 0.f;
float angular_speed_meas_rads = 0.f;
float linear_speed_error = 0.f;
float angular_speed_error = 0.f;

// IMU variables
Mpu9250 imu(&Wire, 0x69);
float ax = 0.f, ay = 0.f, az = 0.f; // [g]
float gx = 0.f, gy = 0.f, gz = 0.f; // [deg/s]
//float mx = 0.f, my = 0.f, mz = 0.f; // [uT]

float axBias = -0.01f, ayBias = 0.05f, azBias = 0.01f;
float gxBias = -0.5f, gyBias = -0.1f, gzBias = 0.25f;

float axCorr = 0.f, ayCorr = 0.f, azCorr = 0.f;
float gxCorr = 0.f, gyCorr = 0.f, gzCorr = 0.f;

// Filter variables
Madgwick filter;
float roll = 0.f, pitch = 0.f, yaw = 0.f; // [deg]
const double max_pitch_angle  = 20.0; // [deg]
const double pitch_angle_offset  = 0.0; // [deg]

// Balance control
float pitch_target = 0.f;
float pitch_dot_target = 0.f;
float pitch_error = 0.f;
float pitch_dot_error = 0.f;

// LQR controller gains
const float Kvul = 0.2858;
const float Kwul = 0.052;
const float Ktul = 2.6088;
const float Kttul = 0.2052;

const float Kvur = 0.2858;
const float Kwur = -0.0526;
const float Ktur = 2.6088;
const float Kttur = 0.2052;

// Motor controller variables
int16_t rpm_meas[N_MOTORS] = {0, }; // [rpm]
int16_t mot_curr[N_MOTORS] = {0, }; // [mA]
int16_t ubat = 0; // [mV]
int16_t ibat = 0; // [mA]

// Time variables
unsigned long time1 = 0, time2 = 0, deltaTime = 0; // [us]
unsigned long meanTime = 0; // [us]
unsigned int counter = 0;
const int sampleRate = 100; // 100 [Hz]

// Utils
String str;

void setup() {
  // Disable interrupts during initialization
  //cli();
  
  // Init serial
  Serial.begin(115200);
  //while (!Serial);
  Serial1.begin(115200);
  //while (!Serial1);
  delay(1000);

  // Init I2C
  Wire.begin();
  Wire.setClock(400000);

  // Init IMU
  if (!imu.Begin()) {
    while (1) {
      Serial.println("Failed to init IMU MPU9250!");
      delay(1000);
    }
  }
  else {
    Serial.println("IMU MPU9250 initialized!");
  }

  // Set IMU sample rate divider (to get 100 Hz)
  if (!imu.ConfigSrd(9)) {
    while (1) {
      Serial.println("Failed to config IMU MPU9250 sample rate!");
      delay(1000);
    }
  }
  /*if (!imu.EnableDrdyInt()) {
    while (1) {
      Serial.println("Failed to enable IMU MPU9250 interrupt!");
      delay(1000);
    }
  }*/

  // Init filter
  filter.begin(sampleRate);

  delay(500);
  Serial.println("Starting main loop...");
  
  // Enable interrupts after initialization
  //sei();

  time1 = micros();
}  // setup


void loop() {
  if (imu.Read()) {
    time2 = micros();
    deltaTime = time2 - time1;
    time1 = time2;
    meanTime += deltaTime;
    counter++;

    // Get IMU data
    imu.accel(&ax, &ay, &az); // [g]
    imu.gyro(&gx, &gy, &gz); // [deg/s]
    //imu.mag(&mx, &my, &mz); // [uT]

    // Compensate for bias
    axCorr = ax - axBias;
    ayCorr = ay - ayBias;
    azCorr = az - azBias;

    gxCorr = gx - gxBias;
    gyCorr = gy - gyBias;
    gzCorr = gz - gzBias;

    // Filter data
    filter.updateIMU(gxCorr, gyCorr, gzCorr, axCorr, ayCorr, azCorr);
    
    roll = filter.getRoll(); // [deg]
    pitch = filter.getPitch(); // [deg]
    yaw = filter.getYaw(); // [deg]

    // Balance controller
    balanceControl();
  
    // Send commands
    sendCommand();
  }

  if (1 == rx_flag) {
    decodeRxUart1Data();
    rx_ct = rx_flag = 0;
  }

  // Display every tenth iteration
  if (0 == (counter % 10)) {
    meanTime /= 10;
    
    Serial.print("dt="); Serial.print(meanTime);
    Serial.print(" us, a=("); Serial.print(ax, 2); Serial.print(","); Serial.print(ay, 2); Serial.print(","); Serial.print(az, 2); Serial.print(") [g]");
    Serial.print(", g=("); Serial.print(gx, 2); Serial.print(","); Serial.print(gy, 2); Serial.print(","); Serial.print(gz, 2); Serial.print(") [*/s]");
    Serial.print(", r="); Serial.print(roll, 2); Serial.print(", p="); Serial.print(pitch, 2); Serial.print(", y="); Serial.print(yaw, 2); Serial.print(" [*]");
    Serial.println("");

    meanTime = 0;
    //counter = 0;
  }
}  // loop


inline float deg2rad(float deg) { return deg * M_PI / 180.0; }


void serialEvent1() {
  rx_in = (uint8_t)Serial1.read();
  
  if (0 == rx_flag) {
    if (rx_ct > 0) {
      rx_buffer[rx_ct] = rx_in;
      rx_ct++;
      if ('\n' == rx_in) rx_flag = 1;
      if (rx_ct > 20) { // ERROR!!!
        rx_ct = rx_flag = 0;
      }
    }
    else if ('$' == rx_in) {
      rx_buffer[0] = rx_in;
      rx_ct = 1;
    }
    else { }
  }
}  // serialEvent1


/*void imu_isr() {
  if (imu.Read()) {
    // Get IMU data
    imu.accel(&ax, &ay, &az); // [g]
    imu.gyro(&gx, &gy, &gz); // [deg/s]
    imu.mag(&mx, &my, &mz); // [uT]
  }
}  // imu_isr */


void decodeRxUart1Data() {
  State_u state;
  
  memcpy(state.raw, rx_buffer, 20);
  
  /*Serial.print("B="); Serial.print(rx_buffer[0], HEX); Serial.print(rx_buffer[1], HEX); Serial.print(rx_buffer[2], HEX);
  Serial.print(rx_buffer[3], HEX); Serial.print(rx_buffer[4], HEX); Serial.print(rx_buffer[5], HEX);
  Serial.print(rx_buffer[18], HEX); Serial.print(rx_buffer[19], HEX);
  Serial.println("");*/
  if (state.fields.header[0] == '$' && state.fields.header[1] == 'A' 
    && state.fields.header[2] == 'C' && state.fields.header[3] == 'D'
    && state.fields.header[4] == 'C' && state.fields.footer[0] == '\r' 
    && state.fields.footer[1] == '\n') {
      // TODO: compute and check checksum!
      //rpm_meas[LEFT] = (int16_t)state.fields.payload[0] || (int16_t)(state.fields.payload[1] << 8);
      //rpm_meas[RIGHT] = (int16_t)state.fields.payload[2] || (int16_t)(state.fields.payload[3] << 8);
      rpm_meas[LEFT] = (int16_t)(state.fields.payload[0] + 256 * state.fields.payload[1]);
      rpm_meas[RIGHT] = (int16_t)(state.fields.payload[2] + 256 * state.fields.payload[3]);
      
      //mot_curr[LEFT] = (int16_t)state.fields.payload[4] || (int16_t)(state.fields.payload[5] << 8);
      //mot_curr[RIGHT] = (int16_t)state.fields.payload[6] || (int16_t)(state.fields.payload[7] << 8);
      mot_curr[LEFT] = (int16_t)(state.fields.payload[4] + 256 * state.fields.payload[5]);
      mot_curr[RIGHT] = (int16_t)(state.fields.payload[6] + 256 * state.fields.payload[7]);
      
      //ubat = (int16_t)state.fields.payload[8] || (int16_t)(state.fields.payload[9] << 8);
      ubat = (int16_t)(state.fields.payload[8] + 256 * state.fields.payload[9]);
      
      //ibat = (int16_t)state.fields.payload[10] || (int16_t)(state.fields.payload[11] << 8);
      ibat = (int16_t)(state.fields.payload[10] + 256 * state.fields.payload[11]);
      Serial.print("Ubat="); Serial.print(ubat); Serial.print(" mV, ibat="); Serial.print(ibat); Serial.println(" mA");
  }
  else { // ERROR!!!
    Serial.println("Failed to decode RX data!");
  }
}  // decodeRxUart1Data


void sendCommand() {
  Command_u command;

  // Set header and footer
  command.fields.header[0] = '$';
  command.fields.header[1] = 'A';
  command.fields.header[2] = 'B';
  command.fields.header[3] = 'B';
  command.fields.header[4] = 'A';
  
  command.fields.footer[0] = '\r';
  command.fields.footer[1] = '\n';

  // Update payload with command
  command.fields.payload[0] = control_mode;

  command.fields.payload[1] = (uint8_t)(rpm_ref[LEFT] & 0xFF);
  command.fields.payload[2] = (uint8_t)((rpm_ref[LEFT] >> 8) & 0xFF);

  command.fields.payload[3] = (uint8_t)(rpm_ref[RIGHT] & 0xFF);
  command.fields.payload[4] = (uint8_t)((rpm_ref[RIGHT] >> 8) & 0xFF);

  // Compute checksum (XOR)
  command.fields.checksum = 0;
  for (uint8_t i = 0; i < 5; i++)
    command.fields.checksum ^= command.fields.payload[i];

  // Send data
  //Serial.write(command.raw, 13);
  Serial1.write(command.raw, 13);
}  // sendCommand


void balanceControl() {
  //static float dir = 1.f;
  //static float cmd = 0.f;

  float cmd_left = 0.f, cmd_right = 0.f;

  if (pitch > max_pitch_angle || pitch < -max_pitch_angle) { // ERROR!!!
    resetBalance();
    Serial.println("Balance control ERROR!");
  }

  linear_speed_meas_ms = 0.5f * (rpm_meas[LEFT] + rpm_meas[RIGHT]) * RPM_2_RADS * WHEEL_RADIUS;
  angular_speed_meas_rads = (rpm_meas[RIGHT] - rpm_meas[LEFT]) * RPM_2_RADS * WHEEL_RADIUS / WHEELBASE;

  linear_speed_error = linear_speed_target_ms - linear_speed_meas_ms;
  angular_speed_error = angular_speed_target_rads - angular_speed_meas_rads;

  pitch_error = pitch_target - pitch;
  pitch_dot_error = pitch_dot_target - gyCorr;

  cmd_left = Kvul * linear_speed_error + Kwul * angular_speed_meas_rads + Ktul * deg2rad(pitch_error) + Kttul * deg2rad(pitch_dot_error);
  cmd_right = Kvur * linear_speed_error + Kwur * angular_speed_meas_rads + Ktur * deg2rad(pitch_error) + Kttur * deg2rad(pitch_dot_error);
  Serial.print("cmdL="); Serial.print(cmd_left); Serial.print(",cmdR="); Serial.println(cmd_right);

  rpm_ref[LEFT] = (int16_t)round(10.0 * cmd_left); // [rpm]
  rpm_ref[RIGHT] = (int16_t)round(10.0 * cmd_right); // [rpm]
  Serial.print("refL="); Serial.print(rpm_ref[LEFT]); Serial.print(",refR="); Serial.println(rpm_ref[RIGHT]);

  // Test
  /*cmd += dir;
  rpm_ref[LEFT] = (int16_t)round(cmd);
  rpm_ref[RIGHT] = (int16_t)round(cmd);
  if (cmd > 300) dir = -1;
  else if (cmd < -300) dir = 1;*/
}  // balanceControl


void resetBalance() {
  // TODO
  
}  // resetBalance
