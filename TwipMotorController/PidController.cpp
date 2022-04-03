/**
 * \file PidController.h
 * \author Lionel GENEVE
 * \date 01/01/2021
 * \version 1.0
 * \brief PID controller implementation
 */
#include "PidController.h"

#include "Arduino.h"

static const float MAX_DELTA_TIME = 0.1; // [s] (<-> 100 ms)
static const float MIN_ERROR = 1.0; // [RPM]

PidController::PidController()
{
  
}

PidController::~PidController()
{
  
}

void PidController::init(float _kf, float _kp, float _ki, float _kd, float _cmdSat)
{
  kf = _kf;
  kp = _kp;
  ki = _ki;
  kd = _kd;
  cmdSaturation = _cmdSat;
  integralSaturation = cmdSaturation;
  minError = MIN_ERROR;
  
  reset();
} // init

void PidController::reset(void)
{
  error = 0.f;
  prevError = 0.f;
  integral = 0.f;
  cmd = 0.f;
  prevCmd = 0.f;
} // reset

float PidController::apply(float _ref, float _meas, float _dt)
{
  // Check validity of sampling interval
  if (_dt <= 0 || _dt >= MAX_DELTA_TIME)
  {
    reset();
    _dt = MAX_DELTA_TIME;
  }

  // Compute PID error
  error = _ref - _meas;

  // Check validity of error
  if (isnan(error))
  {
    Serial.print("!!!NaN error in PID controller!!!\n");
    error = 0.001;
  }

  // Feedforward term
  cmd = kf * _ref;

  // Proportional term
  cmd += kp * error;

  // Integral term with integral freeze control and integral saturation (simplified anti-windup)
  if (fabsf(error) > minError)
  {
    // Integrate error if not saturated
    if (fabsf(integral) < integralSaturation)
    {
      integral += _dt * error;
    }
    cmd += ki * integral;
  }
  
  // Derivative term (TODO: low pass filter derivative to smooth the noise)
  cmd += kd * (error - prevError) / _dt;
  
  // Command saturation with anti wind-up
  if (cmd > cmdSaturation)
  {
    //integral -= _dt * error;  // anti-windup
    cmd = cmdSaturation;
  }
  else if (cmd < -cmdSaturation)
  {
    //integral -= _dt * error;  // anti-windup
    cmd = -cmdSaturation;
  }

  // Check output command validity
  if (isnan(cmd))
  {
    Serial.print("!!!NaN cmd in PID controller!!!\n");
    cmd = 0.0;
  }

  // Save previous values
  prevError = error;
  prevCmd = cmd;
  
  return cmd;
}
