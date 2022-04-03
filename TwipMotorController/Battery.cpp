/**
 * \file Battery.cpp
 * \author Lionel GENEVE
 * \date 01/01/2021
 * \version 1.0
 * \brief Functions to get battery voltage and current
 */
 
#include "Battery.h"

#include <Arduino.h>

// For filtering vbat at Fc=14 Hz, Te=100ms
#define VBAT_LPF_GAIN_NUM (9)
#define VBAT_LPF_GAIN_DEN (10)

static const int16_t LOW_BAT_THRESH = 4000; // in [mV]
static const uint16_t LOW_BAT_COUNTER_THRESH = 600;

static int16_t vbat = 0, vbat_filt = 0; // in [mV]
static int16_t ibat = 0, ibat_filt = 0; // in [mA]

static uint16_t low_battery_counter = 0;
static uint8_t is_battery_low = 0; // 0 = false, 1 = true

void initBattery(void)
{
  // Initialize GPIOs
  pinMode(VBAT, INPUT);
  pinMode(IBAT, INPUT);

  // Initialize variables
  is_battery_low = 0, low_battery_counter = 0;
  vbat = vbat_filt = 0;
  ibat = ibat_filt = 0;

  getVbat();
  getIbat();
}

int16_t getVbat(void)
{
  // Get raw ADC value
  vbat = analogRead(VBAT);
  // Convert raw to mV
  vbat = (int16_t)(((int32_t)vbat * 10000) / (int32_t)1024);
  // Filter (LPF)
  vbat_filt += (int16_t)((VBAT_LPF_GAIN_NUM * ((int32_t)(vbat - vbat_filt))) / VBAT_LPF_GAIN_DEN);
  return vbat_filt;
}

int16_t getIbat(void)
{
  // Get raw ADC value = value in mA
  ibat = analogRead(IBAT);
  // Filter (LPF)
  ibat_filt += (int16_t)((VBAT_LPF_GAIN_NUM * ((int32_t)(ibat - ibat_filt))) / VBAT_LPF_GAIN_DEN);
  return ibat_filt;
}

uint8_t checkVbat(void) // Check battery discharge
{
  if (vbat_filt < LOW_BAT_THRESH)
  {
    if (low_battery_counter < LOW_BAT_COUNTER_THRESH)
      low_battery_counter++;
  }
  else
  {
    if (low_battery_counter > 0)
      low_battery_counter--;
  }
  
  if (low_battery_counter >= LOW_BAT_COUNTER_THRESH)
    is_battery_low = 1;
  else
    is_battery_low = 0;
  
  return is_battery_low;
}
