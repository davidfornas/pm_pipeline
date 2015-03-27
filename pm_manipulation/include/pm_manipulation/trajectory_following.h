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
  boost::shared_ptr<ARM5Arm> robot_;

  int path_index_;
public:


  /** Constructor.
   * @param xx
   * */
  TrajectoryFollowing(nav_msgs::Path path, ros::NodeHandle & nh, std::string joint_state, std::string joint_state_command) : path_(path), path_index_(-1){

    robot_ = boost::shared_ptr<ARM5Arm>(new ARM5Arm(nh, joint_state, joint_state_command));
  }

  void moveToNextWaypoint();
  void getCurrentWaypointPose();

  ~TrajectoryFollowing() {}

};
#endif /* TRAJECTORYFOLLOWING_H_ */
