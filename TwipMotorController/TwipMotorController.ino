/**
 * \file TwipMotorController.ino
 * \brief Twip motor control using the motor shield DualMC33926.
 * \author Lionel GENEVE
 * \date 01/01/2021
 * \version 1.0
 */

// ---------- HEADERS ----------
#include "DualMC33926MotorShield.h"

#include "Battery.h"
#include "Encoder.h"
#include "PidController.h"
#include "Com.h"

// ---------- DEFINES ----------
#define BACKWARD 0
#define FORWARD 1

#define RX_BUFFER_SIZE (50)
//#define TX_BUFFER_SIZE (28)

// ---------- STRUCTS ----------
enum
{
  MANUAL_CTRL = 0,
  AUTOMATIC_CTRL,
  BRAKE_CTRL,
  TORQUE_CTRL,
  N_CTRL_MODES
} ControlModes_e; // Control modes: manual, automatic, brake

// ---------- GLOBAL VARIABLES ----------
// UART communication
static uint8_t rx_buffer[RX_BUFFER_SIZE] = {0, };
static uint8_t rx_in = 0, rx_ct = 0, rx_flag = 0;
static uint8_t nb_rx_data = 0;
//static uint8_t tx_buffer[TX_BUFFER_SIZE] = {0, };

Command_u command;

// Motor controller
DualMC33926MotorShield mc;
static const int16_t PWM_MAX = 400;
static const int16_t RPM_MAX = 430;
//static int8_t dir = 1; // For test purpose
//static uint8_t loop_counter = 0; // For test purpose
static const float trq_cst = 0.0917f; // Motor torque constant (kt, torque = kt * current)

static int16_t rpm_ref[N_MOTORS] = {0, }; // [rpm]
static int16_t rpm_meas[N_MOTORS] = {0, }; // [rpm]
static int16_t cmd_ref[N_MOTORS] = {0, };
static int16_t enc_counts[N_MOTORS] = {0, };
static int16_t mot_currs[N_MOTORS] = {0, }; // [mA]

static uint8_t control_mode = MANUAL_CTRL;

// PID controller
PidController leftPid;
PidController rightPid;

// Battery monitoring
static int16_t vbat = 0; // [mV]
static int16_t ibat = 0; // [mA]
static uint8_t low_bat = 0; // 0 = false, 1 = true

// Timing
static const unsigned int loopPeriod = 20000; // Control loop period [us]
static unsigned long previousTime = 0; // [us]
static unsigned long currentTime = 0; // [us]
static int32_t deltaTime = 0; // Elapsed time [us]
static float deltaTimeF = 0.0; // Control loop [s]

static unsigned long startTime = 0; // [us]
static unsigned long stopTime = 0; // [us]
static unsigned int meanTime = 0; // [us]

// Dummy
static unsigned int counter = 0;
String str;

// Function prototypes
void stopIfFault();
void sendState();

// ---------- SETUP ----------
void setup() {
  // Disable interrupts during initialization
  cli();
  
  // Initialize GPIOs
  
  // Initialize UART communication
  Serial.begin(115200);
  //while (!Serial) { delay(100); }
  Serial1.begin(115200);
  //while (!Serial1) { delay(100); }
  delay(500);

  // Initialize battery
  initBattery();

  // Initialize encoders
  initEncoders();

  // Initialize motor control
  mc.init();

  // Initialize PID controller
  leftPid.init(KF, KP, KI, KD, CMD_MAX);
  rightPid.init(KF, KP, KI, KD, CMD_MAX);

  // Initialize variables
  rpm_ref[LEFT] = rpm_ref[RIGHT] = 0;
  cmd_ref[LEFT] = cmd_ref[RIGHT] = 0;
  rpm_meas[LEFT] = rpm_meas[RIGHT] = 0;
  enc_counts[LEFT] = enc_counts[RIGHT] = 0;
  mot_currs[LEFT] = mot_currs[RIGHT] = 0;

  nb_rx_data = 0;
  rx_flag = rx_ct = 0;
  control_mode = AUTOMATIC_CTRL; //MANUAL_CTRL;

  low_bat = 0;
  vbat = getVbat(); // [mV]
  ibat = getIbat(); // [mA]
  str = "Vbat: " + String(vbat)  + " mV, Ibat: " + String(ibat) + "mA\n";
  Serial.print(str);
  
  delay(1000);

  // Enable interrupts after initialization
  sei();
} // setup

// ---------- LOOP ----------
void loop() {
  // Record processing time [us]
  startTime = micros; // [us]
  
  // Check for new command message
  if (1 == rx_flag) {
    decodeRxUart1Data();
    rx_ct = rx_flag = 0;
    sendState();
  }

  // Check for control loop
  currentTime = micros(); // [us]
  deltaTime = currentTime - previousTime; // [us]
  if (deltaTime >= loopPeriod) {
    // Get battery voltage [mV] and current [mA]
    vbat = getVbat(); // [mV]
    ibat = getIbat(); // [mA]

    // Check battery low voltage
    low_bat = checkVbat();
    if (1 == low_bat) {
      Serial.print("Low battery!\n");
      // TODO: handle low battery --> shutdown everything
    }

    // Get encoder readings [rpm]
    getCountsAndReset(&enc_counts[LEFT], &enc_counts[RIGHT]);
    countsToRpm(enc_counts[LEFT], enc_counts[RIGHT], 
                deltaTime / 1000, &rpm_meas[LEFT], &rpm_meas[RIGHT]);

    // Get motor currents
    mot_currs[LEFT] = mc.getM1CurrentMilliamps(); // [mA]
    mot_currs[RIGHT] = mc.getM2CurrentMilliamps(); // [mA]

    // Compute control
    if (MANUAL_CTRL == control_mode) {
      // Saturate PWM reference
      cmd_ref[LEFT] = (float)rpm_ref[LEFT];
      cmd_ref[LEFT] = max(min(cmd_ref[LEFT], PWM_MAX), -PWM_MAX);
      
      cmd_ref[RIGHT] = (float)rpm_ref[RIGHT];
      cmd_ref[RIGHT] = max(min(cmd_ref[RIGHT], PWM_MAX), -PWM_MAX);
    }
    else if (AUTOMATIC_CTRL == control_mode) {
      // Saturate RPM reference
      rpm_ref[LEFT] = max(min(rpm_ref[LEFT], RPM_MAX), -RPM_MAX);
      rpm_ref[RIGHT] = max(min(rpm_ref[RIGHT], RPM_MAX), -RPM_MAX);
      
      // PID controller
      deltaTimeF = ((float)deltaTime) * 1e-6; // [s]
      cmd_ref[LEFT] = (int16_t)leftPid.apply((float)rpm_ref[LEFT], (float)rpm_meas[LEFT], deltaTimeF);
      cmd_ref[RIGHT] = (int16_t)rightPid.apply((float)rpm_ref[RIGHT], (float)rpm_meas[RIGHT], deltaTimeF);
    }
    else {
      Serial.print("Invalid control mode!\n");
    }

    // Apply control
    mc.setM1Speed(-cmd_ref[LEFT]);
    mc.setM2Speed(cmd_ref[RIGHT]);

    stopIfFault();

    // Send current state information via UART
    //sendState();
    // For Arduino serial plotter
    //Serial.print(rpm_ref[LEFT]); Serial.print(" ");
    //Serial.print(rpm_meas[LEFT]); Serial.print(" ");
    //Serial.println(mot_currs[LEFT]);
    //Serial.print(mot_currs[LEFT]); Serial.print(" ");
    //Serial.print(rpm_ref[RIGHT]); Serial.print(" ");
    //Serial.print(rpm_meas[RIGHT]); Serial.print(" ");
    //Serial.println(mot_currs[RIGHT]);

    previousTime = currentTime;
    counter++;
    
    stopTime = micros();
    meanTime += stopTime - startTime;

    if (0 == (counter % 10)) {
      meanTime /= 10;
      str = "dt: " + String(meanTime) + "us";
      str += ", VB: " + String(vbat) + "mV";
      str += ", IB: " + String(ibat) + "mA";
      str += ", IL: " + String(mot_currs[LEFT]) + "mA";
      str += ", IR: " + String(mot_currs[RIGHT]) + "mA";
      str += ", SL: " + String(rpm_meas[LEFT]) + "RPM";
      str += ", SR: " + String(rpm_meas[RIGHT]) + "RPM";
      Serial.println(str);
      meanTime = 0;
      //counter = 0;
    }
  }
}  // loop


void serialEvent1() {
  rx_in = (uint8_t)Serial1.read();
  
  if (0 == rx_flag) {
    if (rx_ct > 0) {
      rx_buffer[rx_ct] = rx_in;
      rx_ct++;
      if ('\n' == rx_in) rx_flag = 1;
      if (rx_ct > 13) { // ERROR!!!
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


void stopIfFault() {
  if (mc.getFault()) {
    while (1) {
      Serial.println("Motor fault!");
      delay(1000);   
    }
  }
}  // stopIfFault


void sendState() {
  State_u state;
  
  state.fields.header[0] = '$';
  state.fields.header[1] = 'A';
  state.fields.header[2] = 'C';
  state.fields.header[3] = 'D';
  state.fields.header[4] = 'C';
  
  state.fields.footer[0] = '\r';
  state.fields.footer[1] = '\n';

  // Update payload with current state
  //state.fields.payload[0] = (uint8_t)(ts & 0xFF);
  //state.fields.payload[1] = (uint8_t)((ts >> 8) & 0xFF);
  //state.fields.payload[2] = (uint8_t)((ts >> 16) & 0xFF);
  //state.fields.payload[3] = (uint8_t)((ts >> 24) & 0xFF);
  
  state.fields.payload[0] = (uint8_t)(rpm_meas[LEFT] & 0xFF);
  state.fields.payload[1] = (uint8_t)((rpm_meas[LEFT] >> 8) & 0xFF);
  
  state.fields.payload[2] = (uint8_t)(rpm_meas[RIGHT] & 0xFF);
  state.fields.payload[3] = (uint8_t)((rpm_meas[RIGHT] >> 8) & 0xFF);
  
  state.fields.payload[4] = (uint8_t)(mot_currs[LEFT] & 0xFF);
  state.fields.payload[5] = (uint8_t)((mot_currs[LEFT] >> 8) & 0xFF);
  
  state.fields.payload[6] = (uint8_t)(mot_currs[RIGHT] & 0xFF);
  state.fields.payload[7] = (uint8_t)((mot_currs[RIGHT] >> 8) & 0xFF);

  state.fields.payload[8] = (uint8_t)(vbat & 0xFF);
  state.fields.payload[9] = (uint8_t)((vbat >> 8) & 0xFF);

  state.fields.payload[10] = (uint8_t)(ibat & 0xFF);
  state.fields.payload[11] = (uint8_t)((ibat >> 8) & 0xFF);

  // Compute checksum (XOR)
  state.fields.checksum = 0;
  for (uint8_t i = 0; i < 12; i++)
    state.fields.checksum ^= state.fields.payload[i];

  // Send data
  //Serial.write(state.raw, 20);
  Serial1.write(state.raw, 20);
}  // sendState


void decodeRxUart1Data() {
  memcpy(command.raw, rx_buffer, 13);

  if (command.fields.header[0] == '$' && command.fields.header[1] == 'A' 
    && command.fields.header[2] == 'B' && command.fields.header[3] == 'B'
    && command.fields.header[4] == 'A' && command.fields.footer[0] == '\r' 
    && command.fields.footer[1] == '\n') {
      // TODO: compute and check checksum!

      control_mode = command.fields.payload[0]; // 0 = MANUAL, 1 = AUTOMATIC
      rpm_ref[LEFT] = (int16_t)(command.fields.payload[1] + 256 * command.fields.payload[2]);
      rpm_ref[RIGHT] = (int16_t)(command.fields.payload[3] + 256 * command.fields.payload[4]);

    str = "Mode=" + String(control_mode) + ", L=" + String(rpm_ref[LEFT]) + " rpm, R=" + String(rpm_ref[RIGHT]) + " rpm";
    Serial.println(str);
  }
  else { // ERROR!!!
    Serial.println("Failed to decode RX data!");
  }
}  // decodeRxUart1Data
