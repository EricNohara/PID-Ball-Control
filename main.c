#include <p33Fxxxx.h>
#include <xc.h>
#define FCY 12800000UL
#include <stdio.h>
#include <libpic30.h>
#include <stdbool.h>
#include <stdint.h>

#include "lcd.h"
#include "led.h"
#include "motor.h"    // Provides motor_init() and motor_set_duty()
#include "touch.h"    // Provides touch_init(), touch_select_dim(), and touch_read()
#include "timer.h"    // For initializing timer 2

/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT);

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);

#define PWM_PERIOD 4000 //timer counts per full 20 ms cycle
#define PWM_PERIOD_MS 20
#define MIN_SERVO_VAL 0.9
#define MAX_SERVO_VAL 2.1
#define SERVO_RANGE (MAX_SERVO_VAL - MIN_SERVO_VAL)
#define MID_SERVO_VAL 1.5
#define DT 0.05f // 50 ms sampling
#define FILTER_ORDER 3
#define MAX_ERRORS 20

//x-axis PID gains (tune)
#define KP_X 3.0f
#define KD_X 1.2f
#define KI_X 0.01f

//y-axis PID gains (tune)
#define KP_Y 0.9f
#define KD_Y 0.7f
#define KI_Y 0.01f


//range variables
#define MAX_PID_X 10000
#define MIN_PID_X -10000
#define MAX_PID_Y 7500
#define MIN_PID_Y -7500

#define CAL_SAMPLES 20 // how many filtered readings to average
#define CAL_DELAY_MS 50 // ms between each reading

//#define X_CENTER_RAW 1970u
//#define Y_CENTER_RAW 1552u

//3rd order butterworth filter coefficients
//raw ADC readings from the touchscreen are noisy, use filtering so the PID loop sees a clean position
const float b[FILTER_ORDER+1] = {0.09853116092392705f,0.29559348277178116f, 0.29559348277178116f, 0.09853116092392705f};
const float a[FILTER_ORDER+1] = {1.0f,-0.5772405248063026f, 0.42178704868956163f, -0.056297236491842595f};

//state variables
//for derivative term (needs last error to compute slope)
static float errorSumX = 0, lastErrorX = 0;
static float errorSumY = 0, lastErrorY = 0;

//hold the calibrated center ADC values
static uint16_t X_SETPOINT = 0, Y_SETPOINT = 0;
//static uint16_t X_SETPOINT = X_CENTER_RAW;
//static uint16_t Y_SETPOINT = Y_CENTER_RAW;
static uint16_t min_pwm = 0, max_pwm = 0;

//signals every 50ms that it's time for a PID update
static volatile bool timeout_flag = false;

//cicular buffer for integral term
//uses a running sum of recent errors to avoid windup
static uint16_t X_ERRORS[MAX_ERRORS] = {0,0,0,0,0,0,0,0,0,0};
static uint16_t Y_ERRORS[MAX_ERRORS] = {0,0,0,0,0,0,0,0,0,0};

typedef enum {
    X_LEFT = 1,
    X_RIGHT = 2,
    Y_UP = 3,
    Y_DOWN = 4
} Extreme;

//converts a pulse width in ms (0.9-2.1 ms) into the PIC's PWM duty ticks
//PIC PWM modules count off time instead of on time (formula inverts and scales accordingly)
uint16_t calculate_pwm(float pulse_width_ms){  
    float adjusted_val = PWM_PERIOD_MS - pulse_width_ms;
    return (uint16_t) (adjusted_val * (PWM_PERIOD / PWM_PERIOD_MS)); // return it in terms of Period value
}

//Select X or Y on the touchscreen ADC and then read
static uint16_t get_sample(uint8_t dim) {
    touch_select_dim(dim);
    return touch_read();
}

//smooth the readings but keep phase delay minimal 
static float butterworth_filter_x(uint16_t raw_x){
    uint8_t i;
    
    static float in_x[FILTER_ORDER + 1];
    static float out_x[FILTER_ORDER + 1];
    
    //shift input history
    for(i = FILTER_ORDER; i > 0; i--){
        in_x[i] = in_x[i - 1];
        out_x[i] = out_x[i - 1];
    }
    
    in_x[0] = (float) raw_x;
    
    //compute new output
    float retval = 0;
    
    for (i = 0; i < FILTER_ORDER + 1; i++) {
        retval += in_x[i] * b[i];
        if (i > 0) retval -= out_x[i] * a[i];
    }
    
    out_x[0] = retval;
    return retval;
}

static float butterworth_filter_y(uint16_t raw_y){
    uint8_t i;
    
    static float in_y[FILTER_ORDER + 1];
    static float out_y[FILTER_ORDER + 1];
    
    //shift input history
    for(i = FILTER_ORDER; i > 0; i--){
        in_y[i] = in_y[i - 1];
        out_y[i] = out_y[i - 1];
    }
    
    in_y[0] = (float) raw_y;
    
    //compute new output
    float retval = 0;
    
    for (i = 0; i < FILTER_ORDER + 1; i++) {
        retval += in_y[i] * b[i];
        if (i > 0) retval -= out_y[i] * a[i];
    }
    
    out_y[0] = retval;
    return retval;
}

//reads and filters the touch input repeatedly, averages across 20 samples
//averaging removes outliers or transient noise
static float read_stable_filtered(uint8_t dim){
    int8_t i;
    float sum = 0.0f;
    for (i = 0; i < CAL_SAMPLES; i++){
        uint16_t raw = get_sample(dim);
        float f = (dim == TOUCH_DIM_X) ? butterworth_filter_x(raw) : butterworth_filter_y(raw);
        sum += f;
        __delay_ms(CAL_DELAY_MS);
    }
    return sum / (float)CAL_SAMPLES;
}

//set the two servo motors to specific PWM duty cycle values based on the desired extreme position for the ball
//sends a full tilt command to one servo axis and waits 5s
void set_servo_extreme(Extreme extreme) {
    switch (extreme) {
        //both motors get min_pwm (placing the ball into one corner)
        case X_RIGHT:
            motor_set_duty(8, max_pwm);
            break;
        case X_LEFT:
            motor_set_duty(8, min_pwm);
            break;
        case Y_DOWN:
            motor_set_duty(7, max_pwm);
            break;
        case Y_UP:
            motor_set_duty(7, min_pwm);
            break;
        default:
            return;
    }

    // delay of 5 seconds for the ball to settle in the new corner
    __delay_ms(5000);
}

//calibrate X and Y setpoints by driving to extremes and averaging
static void calibrate_setpoints(void){
    // warm up the filter
    int8_t i;
    for (i = 0; i < 10; i++) {
        uint16_t rx = get_sample(TOUCH_DIM_X);
        (void) butterworth_filter_x(rx);
        uint16_t ry = get_sample(TOUCH_DIM_Y);
        (void) butterworth_filter_y(ry);
    }
    
    //converts the servo pwm to PWM duty values
    min_pwm = calculate_pwm(MIN_SERVO_VAL);
    max_pwm = calculate_pwm(MAX_SERVO_VAL);

    //x setpoint: push ball to x-edges by tilting x servo (channel 8)
    set_servo_extreme(X_LEFT);
    uint16_t X_MIN = get_sample(TOUCH_DIM_X);
//    X_MIN = (uint16_t) butterworth_filter_x(X_MIN);
//    int16_t X_MIN = read_stable_filtered(TOUCH_DIM_X);
    
    set_servo_extreme(X_RIGHT);
    uint16_t X_MAX = get_sample(TOUCH_DIM_X);
//    X_MAX = (uint16_t) butterworth_filter_x(X_MAX);
//    int16_t X_MAX = read_stable_filtered(TOUCH_DIM_X);
    
    //midpoint = average, then weak offset (board specific)
    X_SETPOINT = (X_MIN + X_MAX) / 2;
    X_SETPOINT -= 100;
    
    //y setpoint: push ball to y edges using y servo (channel 7)
    set_servo_extreme(Y_UP);
    uint16_t Y_MIN = get_sample(TOUCH_DIM_Y);
//    Y_MIN = (uint16_t) butterworth_filter_y(Y_MIN);
//    uint16_t Y_MIN = read_stable_filtered(TOUCH_DIM_Y);
    
    set_servo_extreme(Y_DOWN);
    uint16_t Y_MAX = get_sample(TOUCH_DIM_Y);
//    Y_MAX = (uint16_t) butterworth_filter_y(Y_MAX);
//    uint16_t Y_MAX = read_stable_filtered(TOUCH_DIM_Y);
    
    Y_SETPOINT = (Y_MIN + Y_MAX) / 2;
    Y_SETPOINT -= 400;
}

//takes PID contronller output u in [-max, +max] and linearly maps it to a servo pulse around the 1.5 ms midpoint
//positive u moves the servo one way and negative the other
float get_pulse(float un, uint16_t max_pid) {
    float pulse_n;
    if (un < 0) {
        pulse_n = MID_SERVO_VAL + (un / max_pid) * (MID_SERVO_VAL - MIN_SERVO_VAL);
    } else {
        pulse_n = MID_SERVO_VAL + (un / max_pid) * (MAX_SERVO_VAL - MID_SERVO_VAL);
    }

    if(pulse_n < MIN_SERVO_VAL) pulse_n = MIN_SERVO_VAL;
    if(pulse_n > MAX_SERVO_VAL) pulse_n = MAX_SERVO_VAL;
    return pulse_n;
}

//sums the last 20 error values
uint16_t get_sum(uint16_t * array) {
    uint8_t i;
    uint16_t total = 0;
    
    for (i = 0; i < MAX_ERRORS; i++) {
        total += array[i];
    }
    
    return total;
   
}

//timer 1 ISR (runs every 50ms)
void __attribute__((interrupt,auto_psv)) _T1Interrupt(void){
    IFS0bits.T1IF = 0;
    timeout_flag = true;
}

int main(void)
{
    // Initialize peripherals
    __C30_UART=1;
    lcd_initialize();
    lcd_clear();
    motor_init(7);    // Initialize motor channel 7 (Y servo, e.g., OC7)
    motor_init(8);    // Initialize motor channel 8 (X servo, e.g., OC8)
    touch_init();     // Initialize touchscreen interface and ADC pins
    set_timer1(50);  // Initialize the timer 1 to period of 50 ms

    //calibrate setpoints + display them
    calibrate_setpoints();
    
    uint8_t idx;
//    X_SETPOINT = X_CENTER_RAW;
//    Y_SETPOINT = Y_CENTER_RAW;
    
    // main execution waits for timer 1 to fire before making adjustments
    uint8_t loop_iteration = 0;
    while(1) {
        if (timeout_flag) {
            timeout_flag = false;
            
            //x axis
            uint16_t raw_x = get_sample(TOUCH_DIM_X);
            float fx = butterworth_filter_x(raw_x);
            float errorX = (float)X_SETPOINT - fx;
            
            // shift X_ERRORS + add newest error
            for (idx = MAX_ERRORS; idx > 0; idx--) {
                X_ERRORS[idx] = X_ERRORS[idx - 1];
            }
            
            X_ERRORS[0] = errorX;
            
            
            float dx = (errorX - lastErrorX) / DT; // derivative
            float p = KP_X * errorX;
            float d = KD_X * dx;
            float i = KI_X * get_sum(X_ERRORS);
            float ux = p + i + d;
            lastErrorX = errorX;

            //map to servo pulse
            float pulse_x = get_pulse(ux, MAX_PID_X);
            motor_set_duty(8, calculate_pwm(pulse_x));
            
            //y axis
            uint16_t raw_y = get_sample(TOUCH_DIM_Y);
            float fy = butterworth_filter_y(raw_y);
            float errorY = (float)Y_SETPOINT - fy;
            
            // shift X_ERRORS + add newest error
            //ready for integral sum
            for (idx = MAX_ERRORS; idx > 0; idx--) {
                Y_ERRORS[idx] = Y_ERRORS[idx - 1];
            }
            
            Y_ERRORS[0] = errorY;
          
            float dy = (errorY - lastErrorY) / DT;
            p = KP_Y * errorY;
            d = KD_Y * dy;
            i = KI_Y * get_sum(Y_ERRORS);
            float uy = p + i + d;
            lastErrorY = errorY;


            //map to servo pulse
            float pulse_y = get_pulse(uy, MAX_PID_Y);
            motor_set_duty(7, calculate_pwm(pulse_y));
            
//            lcd_locate(1,0);
//            lcd_printf("X: %.1f, %.1f, %.1f", KP_X, KI_X, KD_X);
//            lcd_locate(1,1);
//            lcd_printf("Y: %.1f, %.1f, %.1f", KP_Y, KI_Y, KD_Y);
//            lcd_locate(1,2);
//            lcd_printf("SETPT=(%u, %u)", X_SETPOINT, Y_SETPOINT);
//           
//            if (loop_iteration == 30) {
//                //update display
//                lcd_clear_row(3);
//                lcd_locate(1,3);
//                lcd_printf("POS=(%u,%u)", (uint16_t)fx, (uint16_t)fy);
//                
////                lcd_locate(1,4);
////                lcd_printf("FIL=(%.1f,%.1f)", fx, fy);
////                
////                lcd_locate(1,5);
////                lcd_printf("UX=%.1f,UY=%.1f", ux, uy);
////                
////                lcd_locate(1,6);
////                lcd_printf("PX=%.1f,PY=%.1f", pulse_x, pulse_y);
//                
//                loop_iteration = 0;
//            }
//            
//            loop_iteration++;
        }
    }
    return 0;
}
