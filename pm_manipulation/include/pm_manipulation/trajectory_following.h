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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>


/** Description
 */
class TrajectoryFollowing {

  nav_msgs::Path path_;
  boost::shared_ptr<ARM5Arm> robot_;

  vpColVector current_config_;
  int path_index_;


  boost::shared_ptr<tf::TransformBroadcaster> broadcaster_;
  boost::shared_ptr<tf::TransformListener> listener_;

  ros::Publisher js_pub_;
  ros::Publisher position_pub_;

public:

  /** Constructor.
   * @param xx
   * */
  TrajectoryFollowing(nav_msgs::Path path, ros::NodeHandle & nh, std::string joint_state, std::string joint_state_command) : path_(path), path_index_(-1){
    robot_ = boost::shared_ptr<ARM5Arm>(new ARM5Arm(nh, joint_state, joint_state_command));
    broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
    listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
    js_pub_ = nh.advertise<sensor_msgs::JointState>("/uwsim/joint_state_command", 1);
    position_pub_ = nh.advertise<nav_msgs::Odometry>("/dataNavigator",1);
  }

  void moveToNextWaypoint();
  vpColVector getCurrentWaypointPose(){ return current_config_; }

  ~TrajectoryFollowing() {}

};
#endif /* TRAJECTORYFOLLOWING_H_ */
