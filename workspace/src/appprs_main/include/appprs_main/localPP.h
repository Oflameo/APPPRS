/*
 * localPP.h
 *
 *  Created on: Sep 9, 2015
 *      Author: vgrabe
 */

#ifndef LOCALPP_H_
#define LOCALPP_H_

#include <geometry_msgs/PoseStamped.h>

std::vector<geometry_msgs::PoseStamped > getPath(float xa, float ya, float Tha, float xb, float yb, float Thb);




#endif /* LOCALPP_H_ */
