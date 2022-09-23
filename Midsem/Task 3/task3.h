/*
 * impl_tsk.h
 *
 *  Created on: Sep 18, 2022
 *      Author: SysIDEA_OSACE
 */

#ifndef INC_IMPL_TSK_H_
#define INC_IMPL_TSK_H_


void motor_trqctrl(float *des_curr_m1,float *des_curr_m2){

/**************code here**********************/
float x_0=0.1;
float y_0=0.1;
float k=70;

float F_x=-k*(0.1*cos(th1)+0.1*cos(th1+th2)-x_0);
float F_y=-k*(0.1*sin(th1)+0.1*sin(th1+th2)-y_0);
float tau_1=-F_x*(0.1*sin(th1)+0.1*sin(th1+th2))+F_y*(0.1*cos(th1)+0.1*cos(th1+th2));
float tau_2=-F_x*0.1*sin(th1+th2)+F_y*0.1*cos(th1+th2);



/**************code here**********************/

*des_curr_m1=tau_1/0.1;//write ur calculated current variable here for motor1
*des_curr_m2=tau_2/0.1;//write ur calculated current variable here for motor2
}

void traj_ctrl(float *des_theta1,float *des_theta2,float *des_thetad1,float *des_thetad2){
/**************code here**********************/



/**************code here**********************/

		*des_theta1=0;
		*des_theta2=1.57079633;
//		*des_thetad1=vel_track_d2[arr_indx];
//		*des_thetad2=vel_track_d2[arr_indx];
//		if(arr_indx>=646){
//			*des_theta1=thd1_trac[646];
//			*des_theta2=thd2_trac[646];
//		}
}

#endif /* INC_IMPL_TSK_H_ */


//////////////////////////////////////////////////////////////////////
//***********variable===>description and use********************//////
//raw current:
//current===>motor1
//current2==>motor2
//
//filtered current:
//estimate===>motor1
//estimate2==>motor2
//
//angle in radians
//th1===>theta1
//th2==>theta2
//
//estimate3===>theat1_dot
//estimate4===>theat2_dot
//
//arr_indx===>index to manipulate array for trajectory
//
//spth1===>initial position of manipulator theta1
//spth2===>initial position of manipulator theta2

//time_vectt==>time
//////////////////////////////////////////////////////////////////////



