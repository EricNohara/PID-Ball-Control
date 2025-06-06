/****************************************************/
/*                                                  */
/*   CS-454/654 Embedded Systems Development        */
/*   Instructor: Renato Mancuso <rmancuso@bu.edu>   */
/*   Boston University                              */
/*                                                  */
/*   Description: lab3 timer initialization func.   */
/*                                                  */
/****************************************************/


#include "timer.h"
#include "types.h"

//period in ms
void set_timer1(uint16_t period){
    CLEARBIT(T1CONbits.TON); // Disable Timer
    CLEARBIT(T1CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T1CONbits.TGATE); // Disable Gated Timer mode
    TMR1 = 0x00; // Clear timer register
    T1CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    PR1 = (12800000 / 256) * period / 1000; // Load the period value - 12800000 / 256 * 0.05 => for 50 ms period
    IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
    CLEARBIT(IFS0bits.T1IF); // Clear Timer1 Interrupt Flag
    SETBIT(IEC0bits.T1IE); // Enable Timer1 interrupt
    SETBIT(T1CONbits.TON); // Start Timer
}


