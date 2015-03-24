/*
 * TrajectoryFollowing.h
 *
 *  This class is used to guide the robot throug a series of waypoints.
 *
 *  Created on: 24/03/2014
 *      Author: dfornas
 */

#ifndef TRAJECTORYFOLLOWING_H_
#define TRAJECTORYFOLLOWING_H_

#include <pm_perception/border_detection.h>
#include <mar_robot_arm5e/ARM5Arm.h>
//#include <tf/transform_datatypes.h>


/** Description
 */
class TrajectoryFollowing {

  nav_msgs::Path & path_;
  ARM5Arm * robot;

public:


  /** Constructor.
   * @param xx
   * */
  TrajectoryFollowing(nav_msgs::Path & path) : path_(path){}

  void moveToNextWaypoint();
  void getCurrentWaypointPose();

  ~TrajectoryFollowing() {}

};
#endif /* TRAJECTORYFOLLOWING_H_ */
