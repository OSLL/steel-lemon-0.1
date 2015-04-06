#include "jga_ard.h"

#include <stdlib.h>
#include <assert.h>
#include <bcm2835.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>

#include <stdio.h>

struct JGA_ARD {
	RPiGPIOPin activity_pin;
};

/* TODO: move robot-specific params to configuration struct */
#define __FULL_TURN_CNT 15030.0
#define __WHEEL_DIAMETER 7.7
#define __ROBOT_DIAMETER 20.5
#define __FULL_TURN_DIST (M_PI * __WHEEL_DIAMETER)
#define __FWD_DIR 0
#define __BWD_DIR 1
#define __MTR_CNT_I2C_SLAVE_ID 4

/*----------------------------------------------------------------------------*/
/* "Private" functions declaration                                            */

static void go_straight(int distance, int direction);
static void send_i2c_mtr_req(float distance, uint8_t motors_map);
static void wait_for_motor_activity_end(struct JGA_ARD *driver);

/*----------------------------------------------------------------------------*/
/* "Public" functions definition                                              */

struct JGA_ARD *JGA_ARD_init(RPiGPIOPin activity_pin)
{
	struct JGA_ARD *driver = malloc(sizeof(*driver));
	if (!driver)
		return driver;

	driver->activity_pin = activity_pin;
	bcm2835_gpio_fsel(driver->activity_pin, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(driver->activity_pin, BCM2835_GPIO_PUD_DOWN);
	JGA_ARD_stop_motors_async(driver);
	return driver;
}

void JGA_ARD_deinit(struct JGA_ARD **driver)
{
	assert(driver && *driver);
	JGA_ARD_stop_motors_async(*driver);
	free(*driver);
	*driver= NULL;
}

void JGA_ARD_move_forward_sync(struct JGA_ARD *driver, int distance /* cm */)
{
	JGA_ARD_move_forward_async(distance);
	wait_for_motor_activity_end(driver);
}

void JGA_ARD_move_backward_sync(struct JGA_ARD *driver, int distance /* cm */)
{
        JGA_ARD_move_backward_async(distance);
        wait_for_motor_activity_end(driver);
}

void JGA_ARD_turn_sync(struct JGA_ARD *driver, double angle /* rad */)
{
        JGA_ARD_turn_async(angle);
        wait_for_motor_activity_end(driver);
}

void JGA_ARD_move_forward_async(int distance /* cm */)
{
	go_straight(distance, __FWD_DIR);
}

void JGA_ARD_move_backward_async(int distance /* cm */)
{
	go_straight(distance, __BWD_DIR);
}

void JGA_ARD_turn_async(double angle /* rad */)
{
	int r_mtr_dir = angle > 0, l_mtr_dir = r_mtr_dir ^ 1;
	int motors_map = (1 << (2 + r_mtr_dir)) | (1 << l_mtr_dir);
	if (angle == 0.0) {
		return;
	}

	send_i2c_mtr_req(__ROBOT_DIAMETER * fabs(angle) / 2, motors_map);
}

void JGA_ARD_stop_motors_async()
{
	send_i2c_mtr_req(0, 0);
}

/*----------------------------------------------------------------------------*/
/* "Private" functions definition                                             */

static void go_straight(int distance, int direction)
{
	int motors_map = (1 << (2 + direction)) | (1 << direction);
	if (distance == 0) {
		return;
	}
	send_i2c_mtr_req(distance, motors_map);
}

static void send_i2c_mtr_req(float distance, uint8_t motors_map)
{
	uint32_t cnt = distance / __FULL_TURN_DIST * __FULL_TURN_CNT;
	bcm2835_i2c_begin();
	bcm2835_i2c_setSlaveAddress(__MTR_CNT_I2C_SLAVE_ID);
	bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500);

	uint8_t buff[5]; // sizeof(cnt) + sizeof(motors_map);
	buff[0] = motors_map;
	*((uint32_t *)(buff + 1)) = cnt;
	int status = bcm2835_i2c_write((char *)buff, 5);
	assert(status == BCM2835_I2C_REASON_OK && "Motors map sending failed");
	printf("[Send over I2C] motors map: %u; cnt: %u\n",
	       buff[0] & 0xF, *((uint32_t *)(buff + 1)));
	bcm2835_i2c_end();
}

static void wait_for_motor_activity_end(struct JGA_ARD *driver)
{
	assert(driver);
	do {
		usleep(10000);
	} while (bcm2835_gpio_lev(driver->activity_pin));
}

#undef __MOTOR_PINS_CNT
#undef __FULL_TURN_CNT
#undef __WHEEL_DIAMETER
#undef __FULL_TURN_DIST
#undef __ROBOT_DIAMETER
#undef __FWD_DIR
#undef __BWD_DIR
