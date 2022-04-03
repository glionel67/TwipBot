/**
 * \file PidController.h
 * \author Lionel GENEVE
 * \date 01/01/2021
 * \version 1.0
 * \brief PID controller implementation
 */
#pragma once

#include <stdint.h>
#include <math.h>

const float CMD_MAX = 400.0f; 

const float KF = 0.93f; // KF = speed feedforward gain
const float KP = 2.0f; // KP = proportional gain
const float KI = 0.2f; // KI = integral gain
const float KD = 0.0f; // KD = derivative gain

/**
 * \class PidController
 * \brief Implementation of a PID controller
 */
class PidController
{
public:
  PidController();
  ~PidController();
  void init(float _kf, float _kp, float _ki, float _kd, float _cmdSat);
  void reset(void);
  float apply(float _ref, float _meas, float _dt);
private:
  float kf;
  float kp;
  float ki;
  float kd;
  float error;
  float prevError;
  float cmd;
  float prevCmd;
  float integral;
  float minError;
  float cmdSaturation;
  float integralSaturation;
}; // class PidController
