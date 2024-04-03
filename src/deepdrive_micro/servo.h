#if !defined(__SERVO_H)
#define __SERVO_H

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "ppm.pio.h"

void set_value_and_log(uint pin, uint value_usec);

#endif // __SERVO_H
