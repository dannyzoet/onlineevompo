/*
 * muplusone.h
 *
 *  Created on: Apr 9, 2013
 *      Author: Danny
 */

#ifndef MUPLUSONE_H_
#define MUPLUSONE_H_

// Included libraries
#include <webots/robot.h> //obtain main library of webots
#include <webots/differential_wheels.h>   //obtain dif. wheels library
#include <webots/distance_sensor.h>
#include <stdlib.h> //for abs
#include <math.h> // math functions
#include <stdio.h>
#include <time.h>

// Global defines
#define NB_DIST_SENS 8		// number of ir sensors
#define LEFT 0        		// Left side
#define RIGHT 1       		// right side
#define INCR 10
#define TIME_STEP 50 		// [ms] // time step of the simulation
#define MU 10				//Population size mu
#define NMBRWEIGHTS 18		// number of weights in the network
#define MAXSPEED 500		// motor max speed
#define SENSOR_MAX 1500		// max ir sensor value

// struct of candidate
typedef struct candidate {
	double weights[NMBRWEIGHTS];
	double fitness;
	double sigma;
} candidate;



static void init();
static double RunAndEvaluateForOneTimeStep(candidate* evaluee);
static double gaussrand();
static double mindouble(double a, double b);



#endif /* MUPLUSONE_H_ */
