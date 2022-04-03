/**
 * \file Battery.h
 * \author Lionel GENEVE
 * \date 01/01/2021
 * \version 1.0
 * \brief Functions to get battery voltage and current
 * Pont diviseur de tension pour mesure de la tension batterie:
 * R1=2K (1%), R2=2K (1%) --> R1/(R1+R2)=0.5 --> 7.2V*0.5=3.6V
 */
#pragma once

#include <stdint.h>

/**
 * GPIO mapping
 */
#define VBAT (A2) // Battery voltage analogic input (VBAT)
#define IBAT (A3) // Battery current analogic input (IBAT)

/**
 * Functions
 */
void initBattery(void);

/*
 * \fn getVbat
 * \brief Return the battery voltage in [mV]
 */
int16_t getVbat(void);

/*
 * \fn getIbat
 * \brief Return the battery current in [mA]
 */
int16_t getIbat(void);

/*
 * \fn checkVbat
 * \brief Check if the battery voltage is not too low
 */
uint8_t checkVbat(void);
