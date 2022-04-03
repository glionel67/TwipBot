/**
 * \file Com.h
 * \author Lionel GENEVE
 * \date 01/01/2021
 * \version 1.0
 * \brief Functions used for the communication between this motor controller and the high-level controller
 */
#pragma once

#include <stdint.h>

typedef union {
  uint8_t raw[20];
  struct __attribute__ ((packed)) {
    uint8_t header[5];
    uint8_t payload[12];
    uint8_t checksum;
    uint8_t footer[2];
  } fields;
} State_u;

typedef union {
  uint8_t raw[13];
  struct __attribute__ ((packed)) {
    uint8_t header[5];
    uint8_t payload[5];
    uint8_t checksum;
    uint8_t footer[2];
  } fields;
} Command_u;

typedef union {
  uint8_t raw[12];
  struct __attribute__ ((packed)) {
    //uint32_t timestamp;
    int16_t rpm_meas_left;
    int16_t rpm_meas_right;
    int16_t mot_curr_left;
    int16_t mot_curr_right;
    int16_t ubat;
    int16_t ibat;
  } fields;
} StatePayload_u;

typedef union {
  uint8_t raw[5];
  struct __attribute__ ((packed)) {
    uint8_t ctrl_mode;
    int16_t rpm_ref_left;
    int16_t rpm_ref_right;
  } fields;
} CommandPayload_u;
