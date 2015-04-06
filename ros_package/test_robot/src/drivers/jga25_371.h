#ifndef __JGA25_371_H_GUARD__
#define __JGA25_371_H_GUARD__

#include <bcm2835.h>

struct JGA25_371;

/*----------------------------------------------------------------------------*/
/* "Public" functions declaration                                             */

struct JGA25_371 *JGA25_371_init(RPiGPIOPin mtr_pins[4], RPiGPIOPin sh_enc_pin);
void JGA25_371_deinit(struct JGA25_371 **driver);

/* NB: distance/angle == 0 means infinite movement.
   Caller must stop motors by himself */
void JGA25_371_move_forward(struct JGA25_371 *driver, int distance /* cm */);
void JGA25_371_move_backward(struct JGA25_371 *driver, int distance /* cm */);
void JGA25_371_turn(struct JGA25_371 *driver, double angle /* rad */);
void JGA25_371_stop_motors(struct JGA25_371 *driver);

#endif
