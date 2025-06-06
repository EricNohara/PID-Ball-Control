#include "motor.h"
#include <p33Fxxxx.h>
#include <xc.h>
#include <libpic30.h>
#include "types.h"

#define FCY 12800000UL

static uint8_t timer2_initialized = 0; //static flag to make sure TMR2 is initialized once

// motor_init: Initializes Timer2 (if not already started) and configures the Output Compare module
// for the specified channel (chan==7 for Y servo, chan==8 for X servo)
void motor_init(uint8_t chan) {
    // Initialize Timer2 for PWM period: 20ms period -> PR2 = (40e-3 * (FCY/64))
    // With FCY=12.8MHz and prescaler 1:64, tick period = 64/12800000 = 5us so PR2 = 40e-3/5e-6 = 8000.
    T2CON = 0;
    TMR2 = 0;
    PR2 = 4000;
    T2CONbits.TCKPS = 0b10;  // 1:64 prescaler
    T2CONbits.TON = 1;       // Start Timer2
    timer2_initialized = 1;

    // Configure the appropriate Output Compare (OC) module in PWM mode (OCMx mode 6)
    if (chan == 7) {
        // OC7 configuration for Y servo
        CLEARBIT(TRISDbits.TRISD6); //configure OC7 pin as output
        OC7CON = 0; //resets the OC7 control register
        OC7R = 0;     // Start with pulse width of 900us (900us/5us per count = 180)
        OC7RS = 0; //sets the output compare module to PWM mode using TMR2
        OC7CONbits.OCM = 0b110;  // PWM mode using Timer2
    } else if (chan == 8) {
        CLEARBIT(TRISDbits.TRISD7);
        // OC8 configuration for X servo
        OC8CON = 0;
        OC8R = 0;     // 900us initial pulse width
        OC8RS = 0;
        OC8CONbits.OCM = 0b110;  // PWM mode using Timer2
    }
}

// motor_set_duty: Sets the pulse width (in microseconds) for the specified motor channel.
// The pulse width is converted into timer counts (with one count = 5 microseconds).
void motor_set_duty(uint8_t chan, uint16_t duty_counts) {
    if (chan == 7) { // Y is connected to channel 7
        OC7RS = duty_counts;
    } else if (chan == 8) {
        OC8RS = duty_counts; // X is connected to channel 8
    }
}
