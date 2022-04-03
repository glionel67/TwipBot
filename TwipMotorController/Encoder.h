/**
 * \file Encoder.h
 * \author Lionel GENEVE
 * \date 01/01/2021
 * \version 1.0
 * \brief Functions to get encoder counts from left and right motors
 */
#pragma once

#include <stdint.h>

/**
 * GPIO mapping
 */
#define ENC1A (5) // PC6
#define ENC1B (6) // PD7
#define ENC1I (2) // PD1: INT1

#define ENC2A (11) // PB7
#define ENC2B (13) // PC7
#define ENC2I (3) // PD0: INT0

enum { LEFT=0, RIGHT, N_MOTORS }; // Left=A, right=B

/**
 * Functions
 */
void initEncoders(void);

int16_t getCountsLeft(void);

int16_t getCountsAndResetLeft(void);

int16_t getCountsRight(void);

int16_t getCountsAndResetRight(void);

void getCountsAndReset(int16_t* leftCounts, int16_t* rightCounts);

void countsToRpm(int16_t leftCounts, int16_t rightCounts, int32_t dt, int16_t* leftRpm, int16_t* rightRpm);
