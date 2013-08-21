#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define TIME_STEP 100

int main() {
  wb_robot_init();

  // do this once only
  WbNodeRef robot_node1 = wb_supervisor_node_get_from_def("epuck1");
  WbFieldRef trans_field1 = wb_supervisor_node_get_field(robot_node1, "translation");
  
      time_t now;

    struct tm *today;  
    char date[23];

    //get current date  
    time(&now);  
    today = localtime(&now);

    //print it in DD.MM.YY format.
    strftime(date, 23, "sim%Y%m%d.%H%M%S.txt", today);
    
  FILE *fp;
  fp=fopen(date, "w");
  double time = 0.0;
   for (time = 0.0; time < 30.0; time += TIME_STEP / 1000.0) {
    // this is done repeatedly
    const double *trans1 = wb_supervisor_field_get_sf_vec3f(trans_field1);
    fprintf(fp, "%g,%g",trans1[0], trans1[2]);
    fprintf(fp, "\n");
    wb_robot_step(TIME_STEP);
  }
  wb_supervisor_simulation_quit(EXIT_SUCCESS);
fclose(fp);
wb_robot_cleanup();
  return 0;
}