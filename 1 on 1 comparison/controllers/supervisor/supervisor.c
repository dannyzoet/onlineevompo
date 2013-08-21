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
  /*WbNodeRef robot_node2 = wb_supervisor_node_get_from_def("epuck2");
  WbFieldRef trans_field2 = wb_supervisor_node_get_field(robot_node2, "translation");
  WbNodeRef robot_node3 = wb_supervisor_node_get_from_def("epuck3");
  WbFieldRef trans_field3 = wb_supervisor_node_get_field(robot_node3, "translation");
  WbNodeRef robot_node4 = wb_supervisor_node_get_from_def("epuck4");
  WbFieldRef trans_field4 = wb_supervisor_node_get_field(robot_node4, "translation");
  WbNodeRef robot_node5 = wb_supervisor_node_get_from_def("epuck5");
  WbFieldRef trans_field5 = wb_supervisor_node_get_field(robot_node5, "translation");
  WbNodeRef robot_node6 = wb_supervisor_node_get_from_def("epuck6");
  WbFieldRef trans_field6 = wb_supervisor_node_get_field(robot_node6, "translation");
  WbNodeRef robot_node7 = wb_supervisor_node_get_from_def("epuck7");
  WbFieldRef trans_field7 = wb_supervisor_node_get_field(robot_node7, "translation");
  */
  
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
   for (time = 0.0; time < 3600.0; time += TIME_STEP / 1000.0) {
    // this is done repeatedly
    const double *trans1 = wb_supervisor_field_get_sf_vec3f(trans_field1);
    fprintf(fp, "%g,%g",trans1[0], trans1[2]);
    /*const double *trans2 = wb_supervisor_field_get_sf_vec3f(trans_field2);
    fprintf(fp, ",%g,%g",trans2[0], trans2[2]);
    const double *trans3 = wb_supervisor_field_get_sf_vec3f(trans_field3);
    fprintf(fp, ",%g,%g",trans3[0], trans3[2]);
    const double *trans4 = wb_supervisor_field_get_sf_vec3f(trans_field4);
    fprintf(fp, ",%g,%g",trans4[0], trans4[2]);
    const double *trans5 = wb_supervisor_field_get_sf_vec3f(trans_field5);
    fprintf(fp, ",%g,%g",trans5[0], trans5[2]);
    const double *trans6 = wb_supervisor_field_get_sf_vec3f(trans_field6);
    fprintf(fp, ",%g,%g",trans6[0], trans6[2]);
    const double *trans7 = wb_supervisor_field_get_sf_vec3f(trans_field7);
    fprintf(fp, ",%g,%g",trans7[0], trans7[2]);*/
    fprintf(fp, "\n");
    wb_robot_step(TIME_STEP);
  }
  wb_supervisor_simulation_quit(EXIT_SUCCESS);
fclose(fp);
wb_robot_cleanup();
wb_robot_step(TIME_STEP);
wb_robot_step(TIME_STEP);
wb_robot_step(TIME_STEP);
  return 0;
}