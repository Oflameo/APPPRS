/*
 * goal_updater_node.cpp
 *
 *  Created on: Aug 19, 2015
 *      Author: vgrabe
 */

#include "appprs_main/GoalPositionUpdater.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_updater_node");

  GoalPositionUpdater *gpu = new GoalPositionUpdater();
  ros::spin();
  delete gpu; gpu = NULL;

}

