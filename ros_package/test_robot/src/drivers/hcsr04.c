#include "hcsr04.h"

#include <stdlib.h>
#include <time.h>

#include <assert.h>
#include <stdio.h>

struct hcsr04 *hcsr04_init(RPiGPIOPin trig_pin, RPiGPIOPin echo_pin)
{
	struct hcsr04 *hcsr04 = malloc(sizeof(*hcsr04));
	if (!hcsr04)
		return hcsr04;
	hcsr04->trig_pin = trig_pin;
	hcsr04->echo_pin = echo_pin;

	bcm2835_gpio_fsel(hcsr04->trig_pin, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(hcsr04->echo_pin, BCM2835_GPIO_FSEL_INPT);	

	return hcsr04;
}

void hcsr04_deinit(struct hcsr04 **sonar)
{
	assert(sonar && *sonar);
	*sonar = NULL;
}

static int wait_lvl(RPiGPIOPin pin, int lvl)
{
#define GUARD_MAX 100000
	int guard = 0;
	while (bcm2835_gpio_lev(pin) != lvl && guard < GUARD_MAX)
		guard++;
	return (guard == GUARD_MAX) ? -1 : 0;
#undef GUARD_MAX
}

double hcsr04_get_distance(struct hcsr04 *sonar)
{
	struct timespec start, stop;

	bcm2835_gpio_set_pud(sonar->echo_pin, BCM2835_GPIO_PUD_DOWN);
	assert(bcm2835_gpio_lev(sonar->echo_pin) == LOW);

	/* send trigger */
	bcm2835_gpio_write(sonar->trig_pin, HIGH);
	bcm2835_delayMicroseconds(100);
	bcm2835_gpio_write(sonar->trig_pin, LOW);

	/* wait echo pin HIGH */
	if (wait_lvl(sonar->echo_pin, HIGH))
		return HCSR04_BAD_DIST;
	clock_gettime(CLOCK_MONOTONIC, &start);

	/* wait echo pin LOW */
	if (wait_lvl(sonar->echo_pin, LOW))
		return HCSR04_BAD_DIST;
	clock_gettime(CLOCK_MONOTONIC, &stop);

	/* print distance */
	double dbl_time = stop.tv_sec - start.tv_sec +
		(stop.tv_nsec - start.tv_nsec) / 1000000000.0;
	return dbl_time * 340 / 2;
}

