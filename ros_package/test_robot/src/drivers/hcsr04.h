#ifndef __HCSR04_H_GUARD__
#define __HCSR04_H_GUARD__

/*
 * Drawbacks:
 *  -- signal change pooling
 *  -- clock_gettime is used for time measurements
 */

#include <bcm2835.h>

#define HCSR04_BAD_DIST -1

struct hcsr04 {
	RPiGPIOPin trig_pin;
	RPiGPIOPin echo_pin;
};

struct hcsr04 *hcsr04_init(RPiGPIOPin trig_pin, RPiGPIOPin echo_pin);
void hcsr04_deinit(struct hcsr04 **sonar);

double hcsr04_get_distance(struct hcsr04 *sonar);

#endif
