/*
 * p_control_path_planner.h
 *
 *  Created on: Sep 9, 2015
 *      Author: vgrabe
 */

#ifndef P_CONTROL_PATH_PLANNER_H_
#define P_CONTROL_PATH_PLANNER_H_


bool computeNewAngleSpeed(float XRob_w, float YRob_w, float ThRob_w, float Xway_w, float Yway_w, float Thway_w, float& Vel_r, float& Th_steer_r);


#endif /* P_CONTROL_PATH_PLANNER_H_ */
