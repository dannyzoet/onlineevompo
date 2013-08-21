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
	srand ( time(NULL) );
	wb_robot_init();
	init();
	wb_robot_step(TIME_STEP);
	wb_robot_step(TIME_STEP);
	loadpopulation();

	int i = 0;
	//printf("Creating random genomes\n");
	for (i = 0; i < MU; i++) { // Initialisation
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
	wb_robot_cleanup();

	return 0;
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

static void loadpopulation() {

population[0].weights[0]=-8.467157;
population[0].weights[1]=-2.981824;
population[0].weights[2]=-4.295264;
population[0].weights[3]=-0.402230;
population[0].weights[4]=5.548127;
population[0].weights[5]=-1.816437;
population[0].weights[6]=-4.418658;
population[0].weights[7]=-2.869396;
population[0].weights[8]=2.162476;
population[0].weights[9]=0.639178;
population[0].weights[10]=4.183457;
population[0].weights[11]=0.690546;
population[0].weights[12]=-11.319569;
population[0].weights[13]=-13.953774;
population[0].weights[14]=5.357971;
population[0].weights[15]=-1.209830;
population[0].weights[16]=-6.819806;
population[0].weights[17]=14.558633;
population[0].fitness=8244.284164;
population[0].sigma=6.303305;
population[1].weights[0]=-5.422070;
population[1].weights[1]=-8.175634;
population[1].weights[2]=-7.887718;
population[1].weights[3]=6.954063;
population[1].weights[4]=1.041809;
population[1].weights[5]=8.366294;
population[1].weights[6]=-8.198469;
population[1].weights[7]=-2.306919;
population[1].weights[8]=4.174816;
population[1].weights[9]=-2.864046;
population[1].weights[10]=1.129576;
population[1].weights[11]=-0.965890;
population[1].weights[12]=-12.771284;
population[1].weights[13]=-6.446743;
population[1].weights[14]=2.069017;
population[1].weights[15]=-9.745623;
population[1].weights[16]=-0.669043;
population[1].weights[17]=18.660928;
population[1].fitness=8243.165839;
population[1].sigma=3.906030;
population[2].weights[0]=-11.098666;
population[2].weights[1]=-2.863019;
population[2].weights[2]=-3.721009;
population[2].weights[3]=0.079610;
population[2].weights[4]=5.145506;
population[2].weights[5]=-0.050983;
population[2].weights[6]=-3.445042;
population[2].weights[7]=-5.261978;
population[2].weights[8]=1.903690;
population[2].weights[9]=-0.522911;
population[2].weights[10]=3.679870;
population[2].weights[11]=0.911180;
population[2].weights[12]=-9.320191;
population[2].weights[13]=-13.437421;
population[2].weights[14]=5.026683;
population[2].weights[15]=-2.626939;
population[2].weights[16]=-6.979174;
population[2].weights[17]=14.399922;
population[2].fitness=7980.315224;
population[2].sigma=1.351730;
population[3].weights[0]=-9.159371;
population[3].weights[1]=-5.400712;
population[3].weights[2]=-5.561907;
population[3].weights[3]=4.537699;
population[3].weights[4]=4.849437;
population[3].weights[5]=1.385520;
population[3].weights[6]=-3.170868;
population[3].weights[7]=-3.843767;
population[3].weights[8]=5.253052;
population[3].weights[9]=-2.030817;
population[3].weights[10]=1.184416;
population[3].weights[11]=1.432320;
population[3].weights[12]=-4.639222;
population[3].weights[13]=-12.697324;
population[3].weights[14]=4.024707;
population[3].weights[15]=-10.543948;
population[3].weights[16]=-7.074995;
population[3].weights[17]=16.071884;
population[3].fitness=6863.641563;
population[3].sigma=1.179197;
population[4].weights[0]=-9.501531;
population[4].weights[1]=-6.921253;
population[4].weights[2]=-5.508032;
population[4].weights[3]=3.604767;
population[4].weights[4]=4.196343;
population[4].weights[5]=1.675412;
population[4].weights[6]=-2.892638;
population[4].weights[7]=-4.503502;
population[4].weights[8]=3.198184;
population[4].weights[9]=-2.886918;
population[4].weights[10]=-0.505364;
population[4].weights[11]=0.572913;
population[4].weights[12]=-4.692287;
population[4].weights[13]=-11.406135;
population[4].weights[14]=2.990480;
population[4].weights[15]=-10.067048;
population[4].weights[16]=-7.173916;
population[4].weights[17]=17.086699;
population[4].fitness=6168.152792;
population[4].sigma=3.092537;
population[5].weights[0]=0.751744;
population[5].weights[1]=0.930029;
population[5].weights[2]=-1.389806;
population[5].weights[3]=0.442098;
population[5].weights[4]=0.536753;
population[5].weights[5]=-0.408364;
population[5].weights[6]=-0.840242;
population[5].weights[7]=1.523349;
population[5].weights[8]=2.984605;
population[5].weights[9]=-0.404014;
population[5].weights[10]=0.188533;
population[5].weights[11]=-0.555146;
population[5].weights[12]=0.261955;
population[5].weights[13]=-0.619708;
population[5].weights[14]=-1.371723;
population[5].weights[15]=-3.478219;
population[5].weights[16]=-0.069950;
population[5].weights[17]=3.229576;
population[5].fitness=4248.458945;
population[5].sigma=0.143998;
population[6].weights[0]=-3.514251;
population[6].weights[1]=-1.041805;
population[6].weights[2]=-0.886059;
population[6].weights[3]=-0.346648;
population[6].weights[4]=3.366557;
population[6].weights[5]=-0.344981;
population[6].weights[6]=-2.451442;
population[6].weights[7]=-3.139197;
population[6].weights[8]=3.414262;
population[6].weights[9]=0.623411;
population[6].weights[10]=2.329264;
population[6].weights[11]=-1.091781;
population[6].weights[12]=-4.141920;
population[6].weights[13]=-6.061550;
population[6].weights[14]=-3.640097;
population[6].weights[15]=-5.726233;
population[6].weights[16]=2.412568;
population[6].weights[17]=2.664484;
population[6].fitness=3972.633053;
population[6].sigma=2.034365;
population[7].weights[0]=-0.277968;
population[7].weights[1]=0.086295;
population[7].weights[2]=-0.514817;
population[7].weights[3]=-0.378298;
population[7].weights[4]=0.634141;
population[7].weights[5]=-0.247609;
population[7].weights[6]=1.395689;
population[7].weights[7]=-0.399682;
population[7].weights[8]=0.768557;
population[7].weights[9]=-1.198304;
population[7].weights[10]=-0.469692;
population[7].weights[11]=-0.236063;
population[7].weights[12]=-1.165722;
population[7].weights[13]=-0.795538;
population[7].weights[14]=-1.030701;
population[7].weights[15]=-0.132963;
population[7].weights[16]=0.393671;
population[7].weights[17]=0.891473;
population[7].fitness=3292.735994;
population[7].sigma=0.093943;
population[8].weights[0]=0.744671;
population[8].weights[1]=0.958848;
population[8].weights[2]=-1.460916;
population[8].weights[3]=0.497646;
population[8].weights[4]=0.619727;
population[8].weights[5]=-0.483121;
population[8].weights[6]=-0.795025;
population[8].weights[7]=1.588990;
population[8].weights[8]=2.969185;
population[8].weights[9]=-0.400210;
population[8].weights[10]=0.175757;
population[8].weights[11]=-0.487951;
population[8].weights[12]=0.314174;
population[8].weights[13]=-0.603617;
population[8].weights[14]=-1.383103;
population[8].weights[15]=-3.496593;
population[8].weights[16]=-0.097529;
population[8].weights[17]=3.229365;
population[8].fitness=3238.159944;
population[8].sigma=0.020913;
population[9].weights[0]=0.731155;
population[9].weights[1]=0.985044;
population[9].weights[2]=-1.465163;
population[9].weights[3]=0.504645;
population[9].weights[4]=0.624686;
population[9].weights[5]=-0.478682;
population[9].weights[6]=-0.799439;
population[9].weights[7]=1.584859;
population[9].weights[8]=2.973530;
population[9].weights[9]=-0.399580;
population[9].weights[10]=0.157375;
population[9].weights[11]=-0.455096;
population[9].weights[12]=0.300012;
population[9].weights[13]=-0.643403;
population[9].weights[14]=-1.396374;
population[9].weights[15]=-3.490562;
population[9].weights[16]=-0.117615;
population[9].weights[17]=3.235586;
population[9].fitness=2030.689663;
population[9].sigma=0.017883;

return;
}

