/**
 * \file Encoder.cpp
 * \author Lionel GENEVE
 * \date 01/01/2021
 * \version 1.0
 * \brief Functions to get encoder counts from left and right motors
 */
#include "Encoder.h"

#include <Arduino.h>

#define GET_ENC1A ((PINC & (1 << 6)) ? 1 : 0)
#define GET_ENC1B ((PIND & (1 << 7)) ? 1 : 0)
#define GET_ENC2A ((PINB & (1 << 7)) ? 1 : 0)
#define GET_ENC2B ((PINC & (1 << 7)) ? 1 : 0)

static volatile uint8_t encA[N_MOTORS] = {0, };
static volatile uint8_t encB[N_MOTORS] = {0, };
static volatile uint8_t encOld[N_MOTORS] = {0, };
static volatile uint8_t encNew[N_MOTORS] = {0, };
static volatile int16_t encCount[N_MOTORS] = {0, };

static volatile const int16_t encTab[4][4] = {
                    {0, -1, 1, 0},
                    {1, 0, 0, -1},
                    {-1, 0, 0, 1},
                    {0, 1, -1, 0}};

static const int32_t GEAR_RATIO = 75;  // Motor gear ratio
static const int32_t COUNTS_TO_RPM = 5000; // = 60000/12

void isr_enc1(void)
{
  encA[LEFT] = GET_ENC1A; //(uint8_t)digitalRead(ENC1A);
  encB[LEFT] = GET_ENC1B; //(uint8_t)digitalRead(ENC1B);
  encNew[LEFT] = encA[LEFT] + 2 * encB[LEFT];
  encCount[LEFT] -= encTab[encOld[LEFT]][encNew[LEFT]];
  encOld[LEFT] = encNew[LEFT];
  //Serial.print("Enc1\n");
}

void isr_enc2(void)
{
  encA[RIGHT] = GET_ENC2A; //(uint8_t)digitalRead(ENC2A);
  encB[RIGHT] = GET_ENC2B; //(uint8_t)digitalRead(ENC2B);
  encNew[RIGHT] = encA[RIGHT] + 2 * encB[RIGHT];
  encCount[RIGHT] += encTab[encOld[RIGHT]][encNew[RIGHT]];
  encOld[RIGHT] = encNew[RIGHT];
  //Serial.print("Enc2\n");
}

void initEncoders(void)
{
  // Initialize GPIOs
  pinMode(ENC1A, INPUT_PULLUP);
  pinMode(ENC1B, INPUT_PULLUP);
  pinMode(ENC1I, INPUT);
  pinMode(ENC2A, INPUT_PULLUP);
  pinMode(ENC2B, INPUT_PULLUP);
  pinMode(ENC2I, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC1I), isr_enc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2I), isr_enc2, CHANGE);

  // Initialize variables
  encA[LEFT] = GET_ENC1A;
  encB[LEFT] = GET_ENC1B;
  encOld[LEFT] = encA[LEFT] + 2 * encB[LEFT];
  encA[RIGHT] = GET_ENC2A;
  encB[RIGHT] = GET_ENC2B;
  encOld[RIGHT] = encA[RIGHT] + 2 * encB[RIGHT];
  encCount[LEFT] = encCount[RIGHT] = 0;
}

int16_t getCountsLeft(void)
{
  cli();
  int16_t counts = encCount[LEFT];
  sei();
  return counts;
}

int16_t getCountsAndResetLeft(void)
{
  cli();
  int16_t counts = encCount[LEFT];
  encCount[LEFT] = 0;
  sei();
  return counts;
}

int16_t getCountsRight(void)
{
  cli();
  int16_t counts = encCount[RIGHT];
  sei();
  return counts;
}

int16_t getCountsAndResetRight(void)
{
  cli();
  int16_t counts = encCount[RIGHT];
  encCount[RIGHT] = 0;
  sei();
  return counts;
}

void getCountsAndReset(int16_t* leftCounts, int16_t* rightCounts)
{
  cli();
  *leftCounts = encCount[LEFT];
  *rightCounts = encCount[RIGHT];
  encCount[LEFT] = 0;
  encCount[RIGHT] = 0;
  sei();
}

void countsToRpm(int16_t leftCounts, int16_t rightCounts, int32_t dt, 
    int16_t* leftRpm, int16_t* rightRpm)
{
  int32_t num = 0, den = 0;
  
  den = dt * GEAR_RATIO; // !! dt in milliseconds [ms]

  num = leftCounts * COUNTS_TO_RPM;
  (*leftRpm) = (int16_t)(num / den);

  num = rightCounts * COUNTS_TO_RPM;
  (*rightRpm) = (int16_t)(num / den);
}
