#include "touch.h"
#include <p33Fxxxx.h>
#include <xc.h>
#define FCY 12800000UL
#include <libpic30.h>
#include "types.h"

/**
 * @brief Initialize the touchscreen ADC and associated pins.
 *
 * This function configures ADC1 for 10-bit operation with auto-conversion.
 * It sets AN15 (X-dimension) and AN9 (Y-dimension) as analog inputs.
 */
void touch_init(void) {
    // Disable ADC before configuring
    AD1CON1bits.ADON = 0;

    // Set ADC for 10-bit operation and integer output
    AD1CON1bits.AD12B = 1;    // 12-bit mode: for 12 bit samples
    AD1CON1bits.FORM = 0;     // Integer output
    AD1CON1bits.SSRC = 0x7;   // Auto conversion trigger
    AD1CON2 = 0;              // Disable scanning (single channel mode)

    // Set ADC timing: sample time and conversion clock.
    AD1CON3bits.ADRC = 0;     // Use internal clock
    AD1CON3bits.SAMC = 0x1F;  // Auto-sample time bits (adjust if needed)
    AD1CON3bits.ADCS = 2;     // Tad = 3 * Tcy (conversion clock divier)

    // Configure AN15 and AN9 as analog inputs:
    // For dsPIC33F, AN0-15 are configured via AD1PCFGL.
    //used to read touchscreen values
    AD1PCFGLbits.PCFG15 = 0;  // Set AN15 to analog (for X)
    AD1PCFGLbits.PCFG9  = 0;  // Set AN9 to analog (for Y)

    //set up the I/O pins E1, E2, E3 to be output pins
    //control the volatage applied to the touchscreen for selecting dimensions
    CLEARBIT(TRISEbits.TRISE1); //I/O pin set to output
    CLEARBIT(TRISEbits.TRISE2); //I/O pin set to output
    CLEARBIT(TRISEbits.TRISE3); //I/O pin set to output

    // Enable ADC
    AD1CON1bits.ADON = 1; //start sampling when requested
}

/**
 * @brief Select the touchscreen dimension to sample.
 *
 * Sets the ADC channel for sampling based on the selected dimension.
 *
 * @param dimension TOUCH_DIM_X for X-dimension (AN15) or TOUCH_DIM_Y for Y-dimension (AN9).
 */

    // Bottom (X+) Top (X-) Right (Y+) Left (Y-)
    // E1=1 Hi-Z Hi-Z No effect No effect
    // E1=0 Vdd GRN No effect No effect
    // E2=1 No effect No effect Hi-Z No effect
    // E2=0 No effect No effect Vdd No effect
    // E3=1 No effect No effect No effect Hi-Z
    // E3=0 No effect No effect No effect GRN
//change the configuration of the touchscreen so that the ADC can sample the desired dimension
void touch_select_dim(uint8_t dimension) {
    if (dimension == TOUCH_DIM_X) {
        // X axis sampling: X+ VDD, X- GND, Y+ Hi-Z, Y- Hi-Z
        //set up the I/O pins E1, E2, E3 so that the touchscreen X-coordinate pin connects to ADC
        CLEARBIT(LATEbits.LATE1); //drives E1 low 
        //drive E2 and E3 high to get VDD to the opposite X
        SETBIT(LATEbits.LATE2); 
        SETBIT(LATEbits.LATE3); 

        //select analog channel AN15 to read the voltage corresponding to X-dimension
        AD1CHS0bits.CH0SA = 15;  // AN15 = Right/Y+ = ADC input
    } else if (dimension == TOUCH_DIM_Y) {
        // Y axis sampling: X+ Hi-Z, X- Hi-Z, Y+ VDD, Y- GND
        //set up the I/O pins E1, E2, E3 so that the touchscreen X-coordinate pin connects to ADC
        SETBIT(LATEbits.LATE1); //set E1 high
        //drive E2 and E3 low to set proper voltage gradient for Y
        CLEARBIT(LATEbits.LATE2);
        CLEARBIT(LATEbits.LATE3);

        //sets the ADC to sample AN9
        AD1CHS0bits.CH0SA = 9;  // AN9 = Top/X- = ADC input
    }

    // Note also that it takes about 10ms for the touchscreen output signal to be stable when the
    // touchscreen is switched from one operation mode to another.
    __delay_ms(10);
}

/**
 * @brief Read a sample from the touchscreen.
 *
 * Assumes that touch_select_dim() has been called to select the desired dimension.
 *
 * @return uint16_t ADC sample result from the touchscreen.
 */
uint16_t touch_read(void) {
    // Start sampling
    AD1CON1bits.SAMP = 1;
   
    // Wait until conversion is complete
    while (!AD1CON1bits.DONE);
    AD1CON1bits.DONE = 0;  // Clear the DONE flag
    return ADC1BUF0;       // Return the ADC conversion result
}

