#ifndef __JGA_ARD_H_GUARD__
#define __JGA_ARD_H_GUARD__

#include <bcm2835.h>

/**
 * Driver for JGA25-371 motors that are controlled by Arduino (AVR mic).
 * Requires I2C communication. RPi is master, Arduino is slave.
 * NOTE: There is one-directional communication from RPi to Arduino.
 */

struct JGA_ARD;
/*----------------------------------------------------------------------------*/
/* "Public" functions declaration                                             */

void JGA_ARD_move_forward_async(int distance /* cm */);
void JGA_ARD_move_backward_async(int distance /* cm */);
void JGA_ARD_turn_async(double angle /* rad */);
void JGA_ARD_stop_motors_async();

struct JGA_ARD *JGA_ARD_init(RPiGPIOPin activity_pin);
void JGA_ARD_deinit(struct JGA_ARD **driver);

void JGA_ARD_move_forward_sync(struct JGA_ARD *driver, int distance /* cm */);
void JGA_ARD_move_backward_sync(struct JGA_ARD *driver, int distance /* cm */);
void JGA_ARD_turn_sync(struct JGA_ARD *driver, double angle /* rad */);


#endif
