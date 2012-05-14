/*
 * KK board flight controller software for AVR microcontrollers
 * Written by Dane Gardner
 *
 * Based on  Simon Kirby https://github.com/sim-/kk
 * Based on XXcontrol_KR_v1.5 by Minsoo Kim
 * Based on XXcontrol by Mike Barton
 * Based on excellent assembly code by Rolf R Bakke (kapteinkuk)
 * With ideas from Rune Hasvold (CyCrow) and OlliW on rcgroups
 * Thanks, everyone!
 *
 * NO WARRANTY EXPRESSED OR IMPLIED. USE AT YOUR OWN RISK. Always test
 * without propellers! Please do not ship derivative works without
 * source; keep this code open as Rolf first so kindly released his
 * design and code to the community.
 *
 * Should fit on 48, 88, 168, and 328. I've tested TRICOPTER mode on an
 * ATmega88A. You may wish to use avrdude -t to "dump calibration" and
 * check timings on a digital scope. Temperature and voltage shift the
 * oscillator frequency a little, and each chip responds differently.
 * See doc8271.pdf page 401.
 *
 * I notice a few microseconds of output jitter still with the internal
 * oscillator. It seems that only an external resonator or crystal will
 * solve this, but those pins are currently used for Rx and the LED, and
 * the Rx pin cannot be moved to another pin that does not share another
 * PCINT unless RESET is used for that purpose. If using all hardware
 * PPM, it may be acceptable to have a more expensive interrupt handler
 * that simply logs the interrupt time and pin states and do the rest of
 * the processing in RxGetChannels().
 *
 * See http://www.kkmulticopter.com/
 *
 * Hardware PPM supported on motor outputs M1, M2, M5 and M6; software
 * PPM on M3 and M4 (Rx interrupts can cause some jitter). M3 and M4
 * outputs will be copied to M5 and M6, when not otherwise used, to allow
 * use of full hardware PPM.
 *
 * General motor output setup:
 *
 * Single
 *             M1 CCW
 *             |
 *             |
 *
 *             M2 (Servo)
 *             |
 *             |
 *      M5 ---- ---- M3
 *     (Servo) | (Servo)
 *             |
 *             M4 (Servo)
 *
 * Dual
 *             M1 CCW
 *             |
 *             M2 CW
 *             |
 *
 *             |
 *             |
 *   M5/M3 ----+----
 *     (Servo) |
 *             |
 *          M6/M4 (Servo)
 *
 * Twin
 *          / --- \
 *        /    |    \
 *      M1 CW  |     M2 CCW
 *             |
 *      M3     |       M4
 *    (Servo)  |   (Servo)
 *             |
 *             |
 *             M5 (Tail Servo, Optional)
 *             M6 (Tail Servo Reverse, Optional)
 *
 * Tri
 *       M1 CW     M2 CCW
 *         \       /
 *          \.---./
 *           |   |
 *           `---'
 *             |
 *             |M4/M6 (Tail Servo)
 *             M3/M5 CCW
 *
 * Quad-+
 *            M1 CW
 *             |
 *             |
 *             |
 *           .---.
 * M2 CCW----|   |----M3/M5 CCW
 *           `---'
 *             |
 *             |
 *             |
 *           M4/M6 CW
 *
 * Quad-X
 *
 *        M1 CW    M2 CCW
 *         \        /
 *           \.--./
 *            |  |
 *           /`--'\
 *         /        \
 *     M4/M6 CCW   M3/M5 CW
 *
 * Hex
 *             M1 CW
 *             |
 *     M6 CCW  |     M2 CCW
 *       \     |     /
 *         \ .---. /
 *          -|   |-
 *         / `---' \
 *       /     |     \
 *     M5 CW   |     M3 CW
 *             |
 *             M4 CCW
 *
 * Y6
 *
 *      M1,4        M2,5    M1->3 = CW
 *         \       /        M4->6 = CCW
 *          \.---./
 *           |   |
 *           `---'
 *             |
 *             |
 *            M3,6
 *
 */

/* Multicopter Type */
//#define SINGLE_COPTER
//#define DUAL_COPTER
//#define TWIN_COPTER
//#define TRI_COPTER
//#define QUAD_COPTER
#define QUAD_X_COPTER
//#define Y4_COPTER
//#define HEX_COPTER
//#define Y6_COPTER

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "typedefs.h"
#include "io_cfg.h"

#include "gyros.h"
#include "settings.h"
#include "receiver.h"
#include "motors.h"

bool Armed;

static int16_t integral[3];          // PID integral term
static int16_t last_error[3];        // Last proportional error

static void setup()
{
  MCUCR = _BV(PUD);  // Disable hardware pull-up

  receiverSetup();
  gyrosSetup();
  motorsSetup();
  settingsSetup();

  LED_DIR   = OUTPUT;
  LED    = 0;

  /*
   * This suits my ATmega88A: no Tx trim, output timings perfect.
   * See doc8271.pdf page ~401; beware the step at 128. -Simon
   */
  if(OSCCAL == 0x9d)
    OSCCAL = 0x9f;

  /*
   * timer2 8bit - run at 8MHz / 1024 = 7812.5KHz, just used for arming
   */
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);

  Armed = false;

  /*
   * Flash the LED once at power on
   */
  LED = 1;
  _delay_ms(150);
  LED = 0;

  sei();

  _delay_ms(1500);

  ReadGainPots();
  ReadGainPots();
  bool pitchMin = (GainInADC[PITCH] < (ADC_MAX * 5) / 100);    // 5% threshold
  bool rollMin =  (GainInADC[ROLL]  < (ADC_MAX * 5) / 100);    // 5% threshold
  bool yawMin =   (GainInADC[YAW]   < (ADC_MAX * 5) / 100);    // 5% threshold

  if(pitchMin && rollMin && yawMin) { settingsClearAll(); }             // Clear config
  else if(pitchMin && yawMin)       { motorsIdentify(); }               // Motor identification
//  else if(pitchMin && rollMin)      { }                                 // Future use
//  else if(rollMin && yawMin)        { }                                 // Future use
  else if(pitchMin)                 { receiverStickCenter(); }          // Stick Centering Test
  else if(rollMin)                  { gyrosReverse(); }                 // Gyro direction reversing
  else if(yawMin)                   { motorsThrottleCalibration(); }    // ESC throttle calibration
}

static inline void loop()
{
  static uint16_t Change_Arming = 0;
  static uint8_t Arming_TCNT2 = 0;
  int16_t error, emax = 1023;
  int16_t imax, derivative;

  RxGetChannels();

  if(RxInCollective <= 0) {
    // Check for stick arming (Timer2 at 8MHz/1024 = 7812.5KHz)
    Change_Arming += (uint8_t)(TCNT2 - Arming_TCNT2);
    Arming_TCNT2 = TCNT2;

    if(Armed) {
      if(RxInYaw < STICK_THROW || abs(RxInPitch) > STICK_THROW)
        Change_Arming = 0;    // re-set count
    } else {
      if(RxInYaw > -STICK_THROW || abs(RxInPitch) > STICK_THROW)
        Change_Arming = 0;    // re-set count
    }

    // 3Sec / 0.000128 = 23437 = 0x5B8D or
    // 2.5Sec / 0.000128 = 19531 = 0x4C4B
    // 0.5Sec / 0.000128 = 3906 = 0x0F42
    if(Change_Arming > 0x0F42) {
      Armed = !Armed;
      if(Armed)
        CalibrateGyros();
    }
  }

  ReadGyros();

  LED = Armed;

  gyroADC[ROLL]-= gyroZero[ROLL];
  gyroADC[PITCH]-= gyroZero[PITCH];
  gyroADC[YAW]-= gyroZero[YAW];

  //--- Start mixing by setting collective to motor outputs

  RxInCollective = (RxInCollective * 10) >> 3;  // 0-800 -> 0-1000

#ifndef SINGLE_COPTER
  if(RxInCollective > MAX_COLLECTIVE)
    RxInCollective = MAX_COLLECTIVE;
#endif

#ifdef SINGLE_COPTER
  MotorOut1 = RxInCollective;
  MotorOut2 = 840;  // 840
  MotorOut3 = 840;  // 840
  MotorOut4 = 945;  // 840 + 840/8
  MotorOut5 = 945;  // 840 + 840/8
#elif defined(DUAL_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = 500;
  MotorOut4 = 500;
#elif defined(TWIN_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = 500;
  MotorOut4 = 500;
  MotorOut5 = 500;  // Optional
  MotorOut6 = 500;  // Optional
#elif defined(TRI_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = RxInCollective;
  MotorOut4 = 500;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = RxInCollective;
  MotorOut4 = RxInCollective;
#elif defined(Y4_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = RxInCollective * 3 / 4;    // 25% Down
  MotorOut4 = RxInCollective * 3 / 4;    // 25% Down
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
  MotorOut1 = RxInCollective;
  MotorOut2 = RxInCollective;
  MotorOut3 = RxInCollective;
  MotorOut4 = RxInCollective;
  MotorOut5 = RxInCollective;
  MotorOut6 = RxInCollective;
#endif

  imax = RxInCollective;
  if(imax < 0)
    imax = 0;
  imax>>= 3;  /* 1000 -> 200 */

  /* Calculate roll output - Test without props!! */

  RxInRoll = ((int32_t)RxInRoll * (uint32_t)GainInADC[ROLL]) >> STICK_GAIN_SHIFT;
  gyroADC[ROLL] = ((int32_t)gyroADC[ROLL] * (uint32_t)GainInADC[ROLL]) >> GYRO_GAIN_SHIFT;
  if(Config.RollGyroDirection == GYRO_NORMAL)
    gyroADC[ROLL] = -gyroADC[ROLL];

  if(Armed) {
    if(0) {
      error = RxInRoll - gyroADC[ROLL];
      if(error > emax)
        error = emax;
      else if(error < -emax)
        error = -emax;
      integral[ROLL]+= error;
      if(integral[ROLL] > imax)
        integral[ROLL] = imax;
      else if(integral[ROLL] < -imax)
        integral[ROLL] = -imax;
      derivative = error - last_error[ROLL];
      last_error[ROLL] = error;
      RxInRoll+= error + (integral[ROLL] >> 2) + (derivative >> 2);
    } else {
      RxInRoll-= gyroADC[ROLL];
    }
  }

#ifdef SINGLE_COPTER
  MotorOut2+= RxInRoll;
  MotorOut4-= RxInRoll;
#elif defined(DUAL_COPTER)
  MotorOut4+= RxInRoll;
#elif defined(TWIN_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;  // Approximation of sin(60) without div
  MotorOut1+= RxInRoll;
  MotorOut2-= RxInRoll;
#elif defined(TRI_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;  // (.875 versus .86602540)
  MotorOut1+= RxInRoll;
  MotorOut2-= RxInRoll;
#elif defined(QUAD_COPTER)
  MotorOut2+= RxInRoll;
  MotorOut3-= RxInRoll;
#elif defined(QUAD_X_COPTER)
  RxInRoll = RxInRoll >> 1;
  MotorOut1+= RxInRoll;
  MotorOut2-= RxInRoll;
  MotorOut3-= RxInRoll;
  MotorOut4+= RxInRoll;
#elif defined(Y4_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;
  MotorOut1+= RxInRoll;
  MotorOut2-= RxInRoll;
#elif defined(HEX_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;
  MotorOut2-= RxInRoll;
  MotorOut3-= RxInRoll;
  MotorOut5+= RxInRoll;
  MotorOut6+= RxInRoll;
#elif defined(Y6_COPTER)
  RxInRoll = (RxInRoll * 7) >> 3;
  MotorOut1+= RxInRoll;
  MotorOut2+= RxInRoll;
  MotorOut3-= RxInRoll;
  MotorOut4-= RxInRoll;
#endif

  /* Calculate pitch output - Test without props!! */

  RxInPitch = ((int32_t)RxInPitch * (uint32_t)GainInADC[PITCH]) >> STICK_GAIN_SHIFT;
  gyroADC[PITCH] = ((int32_t)gyroADC[PITCH] * (uint32_t)GainInADC[PITCH]) >> GYRO_GAIN_SHIFT;
  if(Config.PitchGyroDirection == GYRO_NORMAL)
    gyroADC[PITCH] = -gyroADC[PITCH];

  if(Armed) {
    if(0) {
      error = RxInPitch - gyroADC[PITCH];
      if(error > emax)
        error = emax;
      else if(error < -emax)
        error = -emax;
      integral[PITCH]+= error;
      if(integral[PITCH] > imax)
        integral[PITCH] = imax;
      else if(integral[PITCH] < -imax)
        integral[PITCH] = -imax;
      derivative = error - last_error[PITCH];
      last_error[PITCH] = error;
      RxInPitch+= error + (integral[PITCH] >> 2) + (derivative >> 2);
    } else {
      RxInPitch-= gyroADC[PITCH];
    }
  }

#ifdef SINGLE_COPTER
  MotorOut3+= RxInPitch;
  MotorOut5-= RxInPitch;
#elif defined(DUAL_COPTER)
  MotorOut3+= RxInPitch;
#elif defined(TWIN_COPTER)
  MotorOut3-= SERVO_REVERSE RxInPitch;
  MotorOut4+= SERVO_REVERSE RxInPitch;
  // Stick Only, Optional
  RxInOrgPitch = abs(RxInOrgPitch);
  MotorOut5+= RxInOrgPitch;      // Tain Servo-Optional, Down Only
  MotorOut6-= RxInOrgPitch;      // Tain Servo-Optional, Down Only (Reverse)
#elif defined(TRI_COPTER)
  MotorOut3-= RxInPitch;
  RxInPitch = (RxInPitch >> 1);      // cosine of 60
  MotorOut1+= RxInPitch;
  MotorOut2+= RxInPitch;
#elif defined(QUAD_COPTER)
  MotorOut1+= RxInPitch;
  MotorOut4-= RxInPitch;
#elif defined(QUAD_X_COPTER)
  RxInPitch = (RxInPitch >> 1);      // cosine of 60
  MotorOut1+= RxInPitch;
  MotorOut2+= RxInPitch;
  MotorOut3-= RxInPitch;
  MotorOut4-= RxInPitch;
#elif defined(Y4_COPTER)
  MotorOut1+= RxInPitch;
  MotorOut2+= RxInPitch;
  MotorOut3-= RxInPitch;
  MotorOut4-= RxInPitch;
#elif defined(HEX_COPTER)
  MotorOut1+= RxInPitch;
  MotorOut4-= RxInPitch;
  RxInPitch = (RxInPitch >> 2);
  MotorOut2+= RxInPitch;
  MotorOut3-= RxInPitch;
  MotorOut5-= RxInPitch;
  MotorOut6+= RxInPitch;
#elif defined(Y6_COPTER)
  MotorOut5-= RxInPitch;
  MotorOut6-= RxInPitch;
  RxInPitch = (RxInPitch >> 1);      // cosine of 60
  MotorOut1+= RxInPitch;
  MotorOut2+= RxInPitch;
  MotorOut3+= RxInPitch;
  MotorOut4+= RxInPitch;
#endif

  /* Calculate yaw output - Test without props!! */

  RxInYaw = ((int32_t)RxInYaw * (uint32_t)GainInADC[YAW]) >> STICK_GAIN_SHIFT;
  gyroADC[YAW] = ((int32_t)gyroADC[YAW] * (uint32_t)GainInADC[YAW]) >> GYRO_GAIN_SHIFT;
  if(Config.YawGyroDirection == GYRO_NORMAL)
    gyroADC[YAW] = -gyroADC[YAW];

  if(Armed) {
    error = RxInYaw - gyroADC[YAW];
    if(error > emax)
      error = emax;
    else if(error < -emax)
      error = -emax;
    integral[YAW]+= error;
    if(integral[YAW] > imax)
      integral[YAW] = imax;
    else if(integral[YAW] < -imax)
      integral[YAW] = -imax;
    derivative = error - last_error[YAW];
    last_error[YAW] = error;
    RxInYaw+= error + (integral[YAW] >> 4) + (derivative >> 4);
  }

#ifdef SINGLE_COPTER
  MotorOut2+= RxInYaw;
  MotorOut3+= RxInYaw;
  MotorOut4+= RxInYaw;
  MotorOut5+= RxInYaw;
#elif defined(DUAL_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut2+= RxInYaw;
#elif defined(TWIN_COPTER)
  MotorOut3+= SERVO_REVERSE(RxInYaw >> 1);
  MotorOut4+= SERVO_REVERSE(RxInYaw >> 1);
#elif defined(TRI_COPTER)
  MotorOut4+= SERVO_REVERSE RxInYaw;
#elif defined(QUAD_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut2+= RxInYaw;
  MotorOut3+= RxInYaw;
  MotorOut4-= RxInYaw;
#elif defined(QUAD_X_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut2+= RxInYaw;
  MotorOut3-= RxInYaw;
  MotorOut4+= RxInYaw;
#elif defined(Y4_COPTER)
  if((MotorOut3 - RxInYaw) < 100)
    RxInYaw = MotorOut3 - 100;  // Yaw Range Limit
  if((MotorOut3 - RxInYaw) > 1000)
    RxInYaw = MotorOut3 - 1000;  // Yaw Range Limit

  if((MotorOut4 + RxInYaw) < 100)
    RxInYaw = 100 - MotorOut4;  // Yaw Range Limit
  if((MotorOut4 + RxInYaw) > 1000)
    RxInYaw = 1000 - MotorOut4;  // Yaw Range Limit

  MotorOut3-= RxInYaw;
  MotorOut4+= RxInYaw;
#elif defined(HEX_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut2+= RxInYaw;
  MotorOut3-= RxInYaw;
  MotorOut4+= RxInYaw;
  MotorOut5-= RxInYaw;
  MotorOut6+= RxInYaw;
#elif defined(Y6_COPTER)
  MotorOut1-= RxInYaw;
  MotorOut4-= RxInYaw;
  MotorOut5-= RxInYaw;
  MotorOut2+= RxInYaw;
  MotorOut3+= RxInYaw;
  MotorOut6+= RxInYaw;
#endif

#if defined(TRI_COPTER)
  /*
   * Rather than clipping the motor outputs and causing instability
   * at throttle saturation, we pull down the throttle of the other
   * motors. This gives priority to stabilization without a fixed
   * collective limit.
   */
  imax = MotorOut1;
  if(MotorOut2 > imax)
    imax = MotorOut2;
  if(MotorOut3 > imax)
    imax = MotorOut3;
  imax-= 1000;
  if(imax > 0) {
    MotorOut1-= imax;
    MotorOut2-= imax;
    MotorOut3-= imax;
  }
#endif

  imax = 114;
  //--- Limit the lowest value to avoid stopping of motor if motor value is under-saturated ---
  if(MotorOut1 < imax)
    MotorOut1 = imax;  // this is the motor idle level
  if(MotorOut2 < imax)
    MotorOut2 = imax;
#if defined(TRI_COPTER) || defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
  if(MotorOut3 < imax)
    MotorOut3 = imax;
#endif
#if defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
  if(MotorOut4 < imax)
    MotorOut4 = imax;
#endif
#if defined(HEX_COPTER) || defined(Y6_COPTER)
  if(MotorOut5 < imax)
    MotorOut5 = imax;
  if(MotorOut6 < imax)
    MotorOut6 = imax;
#endif

  //--- Output to motor ESC's ---
  if(RxInCollective < 1 || !Armed) {
    /* turn off motors unless armed and collective is non-zero */
#ifdef SINGLE_COPTER
    MotorOut1 = 0;
    MotorOut2 = 840;
    MotorOut3 = 840;
    MotorOut4 = 840;
    MotorOut5 = 840;
#elif defined(DUAL_COPTER)
    MotorOut1 = 0;
    MotorOut2 = 0;
    if(!Armed) {
      MotorOut3 = 500;
      MotorOut4 = 500;
    }
#elif defined(TWIN_COPTER)
    MotorOut1 = 0;
    MotorOut2 = 0;
    if(!Armed) {
      MotorOut3 = 500;
      MotorOut4 = 500;
      MotorOut5 = 500;
      MotorOut6 = 500;
    }
#elif defined(TRI_COPTER)
    MotorOut1 = 0;
    MotorOut2 = 0;
    MotorOut3 = 0;
    if(!Armed)
      MotorOut4 = 500;
#elif defined(QUAD_COPTER) || defined(QUAD_X_COPTER) || defined(Y4_COPTER)
    MotorOut1 = 0;
    MotorOut2 = 0;
    MotorOut3 = 0;
    MotorOut4 = 0;
#elif defined(HEX_COPTER) ||  defined(Y6_COPTER)
    MotorOut1 = 0;
    MotorOut2 = 0;
    MotorOut3 = 0;
    MotorOut4 = 0;
    MotorOut5 = 0;
    MotorOut6 = 0;
#endif
  }

  LED = 0;
  output_motor_ppm();
}

int main()
{
  setup();

  while(1)
    loop();
  return 1;
}
