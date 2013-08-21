// Included libraries
#include "muplusone.h" //obtain main library of webots

int motorspeed[2] = { 0, 0 };
WbDeviceTag ps[NB_DIST_SENS];	/* proximity sensors */
int ps_value[NB_DIST_SENS]={0,0,0,0,0,0,0,0};

double pc = 0.0;				//Crossover rate pc   0-1
int tau = 175;				//Evaluation period tau, in steps
double rho = 0.01;				//Re-evaluation rate ro  0-1
double alpha = 0.9;			//Significance level for racing alpha
double beta = 1.9;				//Strictness level for racing beta
double sigmainitial = 0.5;		// initital sigma

candidate population[MU];

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
	/*time_t now;

    struct tm *today;  
    char date[23];
	time(&now);  
    today = localtime(&now);
	strftime(date, 23, "exp%Y%m%d.%H%M%S.txt", today);*/
	srand ( time(NULL) );
	wb_robot_init();
	init();
	wb_robot_step(TIME_STEP);
	wb_robot_step(TIME_STEP);

	int i = 0;
	//printf("Creating random genomes\n");
	for (i = 0; i < MU; i++) { // Initialisation
		int j = 0;
		//printf("Genome nmbr: %d\n", i);
		double *tmp = CreateRandomGenome();
		//printf("weights: ");
		for (j = 0; j < NMBRWEIGHTS; j++) {
			population[i].weights[j] = tmp[j];
			//printf("  %f ", population[i].weights[j]);
		}
		//free(tmp);
		population[i].sigma = sigmainitial;
		//printf("\nEvaluate candidate\n");
		population[i].fitness = RunAndEvaluate(&population[i]);
		//printf("Fitness: %f\n", population[i].fitness);
	}
	//printf("Sorting initial population\n");
	SortPopulation(population);

	/* main loop */
	while (wb_robot_step(TIME_STEP) != -1) {

		if (((double)rand()/(double)RAND_MAX) < rho) { // Don?t create offspring, but re-evaluate
			//printf("reevaluate\n");
			int evaluatee = BinaryTournament(MU + 1); // mu+1 is used to select from all
			population[evaluatee].fitness = (population[evaluatee].fitness
					+ RunAndEvaluate(&population[evaluatee])) / 2; // Combine re-evaluation results
		} else { // Create offspring and evaluate that as challenger
			int challengern = BinaryTournament(MU + 1);
			candidate challenger = population[challengern];
			if (((double)rand()/(double)RAND_MAX) < pc) {
				candidate ParentB = population[BinaryTournament(challengern)];
				AveragingCrossover(&challenger, &ParentB);
			}

			Mutate(&challenger); // Also updates ? s depending on adaptation scheme
			double a = population[0].fitness;
			double b = population[MU - 1].fitness;

			int n = 0;
			challenger.fitness = 0;
			for (n = 0; n < tau; n++) { // Racing: monitor evaluation, abort if challenger fails
				challenger.fitness += RunAndEvaluateForOneTimeStep(&challenger);
				if (alpha > 0) {
				   double ksi = sqrt(
						((pow(a - b, (double) 2) * log(2 / alpha)) / (beta * (n+1))));
				   if (challenger.fitness
						< (population[MU - 1].fitness - (2 * ksi))) {
					//printf("race lost because f: %f  ksi: %f pop:%f",challenger.fitness ,ksi , (population[MU - 1].fitness - (2 * ksi)));
					break;
				   }
				}
			}
			if (challenger.fitness > population[MU - 1].fitness) { // Replace last (i.e. worst) individual
				int j = 0;
				for (j = 0; j < NMBRWEIGHTS; j++) {
					population[MU - 1].weights[j] = challenger.weights[j];
				}
				population[MU - 1].sigma = challenger.sigma;
				population[MU - 1].fitness = challenger.fitness;
			}
		}
		SortPopulation(population);
		
		double averagefitness=0.0;
	           for (i = 0; i < MU; i++) { // Initialisation
		averagefitness += population[i].fitness;
				//printf("Number %d fitness: %f \n",i, population[i].fitness );

    	           }
    	           averagefitness= averagefitness/MU;
		//printf("Best fitness: %f  Average fitness: %f\n",population[0].fitness, averagefitness);
	}
	// write all best solutions to file
	/*FILE *fp;
    fp=fopen(date, "w+");
	for (i = 0; i < MU; i++) {
		fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f \n",population[i].weights[0],population[i].weights[1],population[i].weights[2],population[i].weights[3],population[i].weights[4],population[i].weights[5],population[i].weights[6],population[i].weights[7],population[i].weights[8],population[i].weights[9],population[i].weights[10],population[i].weights[11],population[i].weights[12],population[i].weights[13],population[i].weights[14],population[i].weights[15],population[i].weights[16],population[i].weights[17],population[i].fitness,population[i].sigma);
	}
	fclose(fp);*/
	wb_robot_cleanup();

	return 0;
}

static double* CreateRandomGenome() {
	//double *weights = malloc(NMBRWEIGHTS);
	static double weights[NMBRWEIGHTS];
	int i = 0;
	for (i = 0; i < NMBRWEIGHTS; i++) {
		weights[i] = ((2*(double) rand()) / (double) RAND_MAX)-1;
	}
	return weights;
}

static void AveragingCrossover(candidate* parentA, candidate* parentB) {
	int i = 0;
	for (i = 0; i < NMBRWEIGHTS; i++) {
		parentA->weights[i] = (parentA->weights[i]+parentB->weights[i]) / 2;
	}
	parentA->sigma = (parentA->sigma+parentB->sigma) / 2;
}

static int BinaryTournament(int c) {
	int x = 0;
	if (c < MU) {
		x++;
	}

	int a = rand() % (MU - x);
	x++;
	int b = rand() % (MU - x);
	if (b >= a) {
		b++;
	}
	if (b >= c) {
		b++;
	}
	if (population[a].fitness > population[b].fitness) {
		return a;
	} else {
		return b;
	}
}

static void SortPopulation() {
	/* size_t structs_len = sizeof(population) / sizeof(struct candidate);
	 qsort(population, structs_len, sizeof(struct candidate), struct_cmp_by_fitness);
*/
    // manual sorting for microprocessor
    struct candidate temp;
    int i = 0;
    int j = 0;
    for(i = 0; i < MU-1; i++)  {
       for(j = i + 1; j < MU; j++)  {
          if(population[i].fitness < population[j].fitness)  {
             temp = population[i];
            population[i] = population[j];
            population[j] = temp;
          }
       }
    }
}

static void Mutate(candidate* solution) {
            solution->sigma = solution->sigma * pow(M_E, gaussrand()); //gaussrand() * solution->sigma;

	int i = 0;
	for (i = 0; i < NMBRWEIGHTS; i++) {
			solution->weights[i] = solution->weights[i]
					+ gaussrand() * solution->sigma;
	}

	
}

static double RunAndEvaluate(candidate* evaluee) {
	int i = 0;
	double fitness = 0;
	while (i<tau) {
		fitness += RunAndEvaluateForOneTimeStep(evaluee);
		i++;
	}

	return fitness;
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

/* qsort struct comparision function (price float field) */
static int struct_cmp_by_fitness(const void *a, const void *b)
{
    struct candidate *ia = (struct candidate *)a;
    struct candidate *ib = (struct candidate *)b;
    return (int)(100.f*ib->fitness - 100.f*ia->fitness);
	/* float comparison: returns negative if b > a
	and positive if a > b. We multiplied result by 100.0
	to preserve decimal fraction */

}

