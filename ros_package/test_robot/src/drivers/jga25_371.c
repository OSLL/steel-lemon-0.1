#include "jga25_371.h"

#include <stdlib.h>
#include <assert.h>
#include <bcm2835.h>
#include <string.h>
#include <math.h>

#include <stdio.h>

/* TODO: move robot-specific params to configuration struct */
#define __MOTOR_PINS_CNT 2
#define __FULL_TURN_CNT 14050.0
#define __WHEEL_DIAMETER 7.7
#define __ROBOT_DIAMETER 20.0
#define __FULL_TURN_DIST (M_PI * __WHEEL_DIAMETER)
#define __FWD_DIR 0
#define __BWD_DIR 1

struct JGA25_371 {
	RPiGPIOPin l_motor_pins[__MOTOR_PINS_CNT]; /* left motor pins */
	RPiGPIOPin r_motor_pins[__MOTOR_PINS_CNT]; /* right motor pins */
	RPiGPIOPin se_pin;      /* shaft encoder pins */
};

/*----------------------------------------------------------------------------*/
/* "Private" functions declaration                                            */

static void go_straight(struct JGA25_371 *driver, int distance, int direction);
static void wait_for_traversal(struct JGA25_371 *driver, float distance);

/*----------------------------------------------------------------------------*/
/* "Public" functions definition                                              */

struct JGA25_371 *JGA25_371_init(RPiGPIOPin mtr_pins[4], RPiGPIOPin sh_enc_pin)
{
	struct JGA25_371 *driver = malloc(sizeof(*driver));
	if (!driver)
		return driver;

	memcpy(driver->l_motor_pins, mtr_pins,
	       sizeof(*mtr_pins) * __MOTOR_PINS_CNT);
	memcpy(driver->r_motor_pins, mtr_pins + __MOTOR_PINS_CNT,
	       sizeof(*mtr_pins) * __MOTOR_PINS_CNT);
	driver->se_pin = sh_enc_pin;

	for (int i = 0; i < __MOTOR_PINS_CNT; ++i) {
		bcm2835_gpio_fsel(driver->l_motor_pins[i],
				  BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_fsel(driver->r_motor_pins[i],
				  BCM2835_GPIO_FSEL_OUTP);
	}
	bcm2835_gpio_fsel(driver->se_pin, BCM2835_GPIO_FSEL_INPT);
	
	JGA25_371_stop_motors(driver);
	return driver;
}

void JGA25_371_deinit(struct JGA25_371 **driver)
{
	assert(driver && *driver);
	JGA25_371_stop_motors(*driver);
	*driver= NULL;
}

void JGA25_371_move_forward(struct JGA25_371 *driver, int distance /* cm */)
{
	assert(driver);
	JGA25_371_stop_motors(driver);
	go_straight(driver, distance, __FWD_DIR);
}

void JGA25_371_move_backward(struct JGA25_371 *driver, int distance /* cm */)
{
	assert(driver);
	JGA25_371_stop_motors(driver);
	go_straight(driver, distance, __BWD_DIR);
}

void JGA25_371_turn(struct JGA25_371 *driver, double angle /* rad */)
{
	assert(driver);
	JGA25_371_stop_motors(driver);

	int r_mtr_dir = angle > 0, l_mtr_dir = r_mtr_dir ^ 1;
	bcm2835_gpio_write(driver->l_motor_pins[l_mtr_dir], HIGH);
	bcm2835_gpio_write(driver->r_motor_pins[r_mtr_dir], HIGH);
	if (angle == 0.0) {
		return;
	}

	wait_for_traversal(driver, __ROBOT_DIAMETER * fabs(angle) / 2);
	JGA25_371_stop_motors(driver);
}

void JGA25_371_stop_motors(struct JGA25_371 *driver)
{
	for (int i = 0; i < __MOTOR_PINS_CNT; ++i) {
		bcm2835_gpio_write(driver->l_motor_pins[i], LOW);
		bcm2835_gpio_write(driver->r_motor_pins[i], LOW);
	}
}

/*----------------------------------------------------------------------------*/
/* "Private" functions definition                                             */

static void go_straight(struct JGA25_371 *driver, int distance, int direction)
{
	bcm2835_gpio_write(driver->l_motor_pins[direction], HIGH);
	bcm2835_gpio_write(driver->r_motor_pins[direction], HIGH);
	if (distance == 0) {
		return;
	}
	wait_for_traversal(driver, distance);
	JGA25_371_stop_motors(driver);
}

static void wait_for_traversal(struct JGA25_371 *driver, float distance)
{
	/* TODO: try to eliminate busy wait */
	/* TODO: consider overflow */
	int cnt = distance / __FULL_TURN_DIST * __FULL_TURN_CNT;
	int prev = 0;
	while (cnt) {
		int curr = bcm2835_gpio_lev(driver->se_pin);
		if (curr == prev)
			continue;

		prev = curr;
		cnt -= curr ? 1 : 0;
	}
}

#undef __MOTOR_PINS_CNT
#undef __FULL_TURN_CNT
#undef __WHEEL_DIAMETER
#undef __FULL_TURN_DIST
#undef __ROBOT_DIAMETER
#undef __FWD_DIR
#undef __BWD_DIR
