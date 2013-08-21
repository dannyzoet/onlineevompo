// Included libraries
#include "neuralnetworkcontroller.h" //obtain main library of webots

int motorspeed[2] = { 0, 0 };
WbDeviceTag ps[NB_DIST_SENS];	/* proximity sensors */
int ps_value[NB_DIST_SENS]={0,0,0,0,0,0,0,0};

double pc = 0.0;				//Crossover rate pc   0-1
int tau = 175;				//Evaluation period tau, in steps
double rho = 0.005;				//Re-evaluation rate ro  0-1
double alpha = 0.9;			//Significance level for racing alpha
double beta = 2;				//Strictness level for racing beta
double sigmainitial = 1;		// initital sigma

candidate controller;

static void init() {
              //printf("Initializing\n");
	  // get distance sensors
	  char textPS[] = "ps0";
	  int it=0;
	  for (it=0;it<NB_DIST_SENS;it++) {
	    ps[it] = wb_robot_get_device(textPS);
	    textPS[2]++;
	  }

	  for(it=0;it<NB_DIST_SENS;it++) {
	    wb_distance_sensor_enable(ps[it],TIME_STEP);
	  }
}

int main() {
	srand ( time(NULL) );
	
	controller.fitness=0.0;
	controller.sigma=0.0;
controller.weights[0]=-8.467157;
controller.weights[1]=-2.981824;
controller.weights[2]=-4.295264;
controller.weights[3]=-0.402230;
controller.weights[4]=5.548127;
controller.weights[5]=-1.816437;
controller.weights[6]=-4.418658;
controller.weights[7]=-2.869396;
controller.weights[8]=2.162476;
controller.weights[9]=0.639178;
controller.weights[10]=4.183457;
controller.weights[11]=0.690546;
controller.weights[12]=-11.319569;
controller.weights[13]=-13.953774;
controller.weights[14]=5.357971;
controller.weights[15]=-1.209830;
controller.weights[16]=-6.819806;
controller.weights[17]=14.558633;
	wb_robot_init();
	init();
	wb_robot_step(TIME_STEP);
	wb_robot_step(TIME_STEP);

	/* main loop */
	while (wb_robot_step(TIME_STEP) != -1) {
		RunAndEvaluateForOneTimeStep(&controller);
	}
	wb_robot_cleanup();

	return 0;
}


static double RunAndEvaluateForOneTimeStep(candidate* evaluee) {
	double fitness = 0;
	// read
	  wb_differential_wheels_enable_encoders(TIME_STEP);
	  wb_differential_wheels_set_encoders(0,0);
	// read sensors
	  int i = 0;
	  for(i=0;i<NB_DIST_SENS;i++){
		ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
	  }
	// calculate neural net
	  double left=0.0;
	  double right=0.0;
	  for(i=0;i<NB_DIST_SENS;i++){
		  left += mindouble((ps_value[i]-(SENSOR_MAX/2))/(SENSOR_MAX/2), 1.0) * evaluee->weights[i];
		  right += mindouble((ps_value[i]-(SENSOR_MAX/2))/(SENSOR_MAX/2), 1.0) * evaluee->weights[i+(NMBRWEIGHTS/2)];
	  }
	  left += evaluee->weights[(NMBRWEIGHTS/2)-1];
	  right +=  evaluee->weights[(NMBRWEIGHTS)-1];
	  left = tanh(left);
	  right = tanh(right);
	  if (left > 1) {left=1;}
	  if (left <-1) {left=-1;}
	  if (right > 1) {right=1;}
	  if (right <-1) {right=-1;}
	  motorspeed[LEFT]=left*MAXSPEED;
	  motorspeed[RIGHT]=right*MAXSPEED;
	// set motor speed
	wb_differential_wheels_set_speed(motorspeed[LEFT],motorspeed[RIGHT]);
	// wait till end of step
	wb_robot_step(TIME_STEP);
  	// determine fitness
  	double sensorpenalty=0;
          for(i=0;i<NB_DIST_SENS;i++){
               if (sensorpenalty< (double)ps_value[i]/(double)SENSOR_MAX) {
		  sensorpenalty= (double)ps_value[i]/((double)SENSOR_MAX);
                 }
	  }
	  if (sensorpenalty>1) {sensorpenalty=1;}
	  double speedpenalty = 0.0;// fabs(turn);
	  if (motorspeed[LEFT]>motorspeed[RIGHT]) {
  	     speedpenalty = (double)(motorspeed[LEFT]-motorspeed[RIGHT])/(double)(MAXSPEED);
	  } else {
  	     speedpenalty = (double)(motorspeed[RIGHT]-motorspeed[LEFT])/(double)(MAXSPEED);
	  }
	  if (speedpenalty>1) {speedpenalty=1;}
	  //printf("Sensorpenalty: %f \n",sensorpenalty);
	  //printf("fitness: %f  %f \n",wb_differential_wheels_get_left_encoder(), wb_differential_wheels_get_right_encoder());
      fitness = (double)(wb_differential_wheels_get_left_encoder()+wb_differential_wheels_get_right_encoder())*((double)1-speedpenalty)*((double)1-sensorpenalty);
	return fitness;
}

static double gaussrand() {
	double V1, V2, S;
	int phase = 0;
	double X;

	if (phase == 0) {
		do {
			double U1 = (double) rand() / (double)RAND_MAX;
			double U2 = (double) rand() / (double)RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	} else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	return X;
}

static double mindouble(double a, double b) {
    if (a<b) {
       return a;
    } else {
       return b;
    }
}


