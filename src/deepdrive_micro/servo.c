/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * 8-channel Radio Control PPM example
 * Output on GPIO3
 *

 *  synchro    |  ch0 value 1-2ms    |  ch1 value 1-2ms  |...|  ch8 value 1-2ms  |
 *  >=5ms       _______               _______                 _______              _______
 * _______...__| 0.5ms |_____________| 0.5ms |___________|...| 0.5ms |____________| 0.5ms |_...
 *
 */
#include <stdio.h>

#include "servo.h"

void set_value_and_log(uint pin, uint value_usec) {
    printf("Channel %d is set to value %5.3f ms\r\n", channel, (float)value_usec / 1000.0f);
    ppm_set_value(channel, value_usec);

}