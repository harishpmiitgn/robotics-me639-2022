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



/**************code here**********************/

*des_curr_m1=2.34;//write ur calculated current variable here for motor1
*des_curr_m2=0.00234;//write ur calculated current variable here for motor2
}

void traj_ctrl(float *des_theta1,float *des_theta2,float *des_thetad1,float *des_thetad2){
/**************code here**********************/



/**************code here**********************/

		*des_theta1=thd1_trac[arr_indx];
		*des_theta2=thd2_trac[arr_indx];
//		*des_thetad1=vel_track_d2[arr_indx];
//		*des_thetad2=vel_track_d2[arr_indx];
		if(arr_indx>=646){
			*des_theta1=thd1_trac[646];
			*des_theta2=thd2_trac[646];
		}
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



