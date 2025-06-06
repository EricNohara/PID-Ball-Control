#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(uint8_t chan);
void motor_set_duty(uint8_t chan, uint16_t duty_us);

#endif // MOTOR_H
