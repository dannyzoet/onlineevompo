// Included libraries
// Included libraries
#include <webots/robot.h> //obtain main library of webots
#include <webots/differential_wheels.h>   //obtain dif. wheels library
#include <webots/distance_sensor.h>
#include <stdlib.h> //for abs
#include <math.h> // math functions
#include <stdio.h>
#include <time.h>


#define TIME_STEP 100 		// [ms] // time step of the simulation



int main() {

	srand ( time(NULL) );
	wb_robot_init();
	wb_robot_step(TIME_STEP);
	wb_robot_step(TIME_STEP);


	/* main loop */
	while (wb_robot_step(TIME_STEP) != -1) {

		wb_differential_wheels_set_speed(1000,1000);
	}
	wb_robot_cleanup();

	return 0;
}
