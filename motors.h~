#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

/*** BEGIN DEFINES ***/
//#define SERVO_REVERSE

/*
 * ESC PPM output rate -
 * Do not set lower than 122 Hz without a slower/higher t0/t1 clkdiv
 * as the period needs to fit in the 16-bit timer.
 */
//#define ESC_RATE 300  // in Hz
//#define ESC_RATE 400  // in Hz (at SINGLE_COPTER Only)
#define ESC_RATE 450  // in Hz
//#define ESC_RATE 495  // in Hz

// NOTE: Set to 50 for analog servos, 250 for digital servos.
#define SERVO_RATE 50  // in Hz
/*** END DEFINES ***/

/*** BEGIN HELPER MACROS ***/
#define PWM_LOW_PULSE_US ((1000000 / ESC_RATE) - 2000)

#ifdef SERVO_REVERSE
#undef SERVO_REVERSE
#define SERVO_REVERSE -
#else
#define SERVO_REVERSE
#endif
/*** END HELPER MACROS ***/

/*** BEGIN VARIABLES ***/
extern int16_t MotorOut1;
extern int16_t MotorOut2;
extern int16_t MotorOut3;
extern int16_t MotorOut4;
extern int16_t MotorOut5;
extern int16_t MotorOut6;
extern int16_t MotorStartTCNT1;
#if defined(SINGLE_COPTER) || defined(DUAL_COPTER) || defined(TWIN_COPTER) || defined(TRI_COPTER)
extern uint8_t servo_skip;
extern uint16_t servo_skip_divider;
#endif
/*** END VARIABLES ***/

/*** BEGIN PROTOTYPES ***/
void motorsSetup(void);
void motorLoop(void);
void motorsIdentify(void);
void motorsThrottleCalibration(void);
void output_motor_ppm(void);
/*** END PROTOTYPES ***/

#endif