#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

PID::PID(){

    /* TO DO
     *
     * find appropriate values for PID contorl, and initialize them.
     *
    */

	Kp=3.0;
	Ki=0.00;
	Kd=2.5;
//	Ki=0;	Kd=0;
	error = 0.0;
	error_sum = 0.0;
	error_diff = 0.0;
	pre_x=0.0;
	pre_y=0.0;	

}

void PID::reset() {
    error = 0;
    error_sum = 0;
    error_diff = 0;
}

float PID::get_control(point car_pose, point goal_pose){

    /* TO DO
     *
     * implement pid algorithm
     *
    */
	float ctrl;
/*    float cartogoal_size=sqrt(pow(goal_pose.x-car_pose.x,2)+pow(goal_pose.y-car_pose.y,2));
    float heading_size=sqrt(pow(car_pose.x-pre_x,2)+pow(car_pose.y-pre_y,2));
    float cartogoal_x=(goal_pose.x-car_pose.x)/cartogoal_size;
    float cartogoal_y=(goal_pose.y-car_pose.y)/cartogoal_size;
    float heading_x=(car_pose.x-pre_x)/heading_size;
    float heading_y=(car_pose.y-pre_y)/heading_size;
  */  

	float angle_from_car_to_goal=atan2(goal_pose.y-car_pose.y,goal_pose.x-car_pose.x);
	float heading_angle = car_pose.th;//atan2(car_pose.y-pre_y,car_pose.x-pre_x);
	float pre_error=error;
	error = angle_from_car_to_goal - heading_angle;
		if(error<= -M_PI) error += 2*M_PI;
	else if(error>=M_PI) error -= 2*M_PI;


	if(sqrt(pow(car_pose.x-goal_pose.x,2)+pow(car_pose.y-goal_pose.y,2))<0.2) {
        //	reset();
	}



	error_sum += pre_error;
	error_diff =  pre_error - error;
//	printf("P I D %.2f %.2f %.2f \n",Kp*error, Ki*error_sum, Kd*error_diff);

	ctrl = Kp* error + Ki*error_sum + Kd*error_diff;  
	if(ctrl>=4.0) ctrl=4.0;
	else if(ctrl<=-4.0) ctrl=-4.0;

	pre_x=car_pose.x;
	pre_y=car_pose.y;
//	printf(" ctrl %.2f\n\n",ctrl);
	return ctrl;


}
