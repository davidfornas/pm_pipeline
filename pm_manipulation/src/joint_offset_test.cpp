/**
 * This program launches the joint_offset. Now only Elbow is statically corrected.
 *  Created on: 27/02/2017
 *
 */
#include <ros/ros.h>
#include <pm_manipulation/joint_offset.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <visp/vpColVector.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_offset_launcher");
  ros::NodeHandle nh;

  std::string joint_state_command("/uwsim/joint_state_command");
  nh.getParam("joint_state_command", joint_state_command);

  std::string joint_state("/uwsim/joint_state");
  nh.getParam("joint_state", joint_state);

  std::string joint_state_fixed("/uwsim/joint_state_fixed");
  nh.getParam("joint_state_fixed", joint_state_fixed);

  double elbow_offset = 0.3;
  JointOffset* joint_offset = new JointOffset(nh, joint_state, joint_state_command, joint_state_fixed, elbow_offset);

  /** TEST JOINT OFFSET
  JointOffset* joint_offset = new JointOffset(nh, joint_state, joint_state_command, joint_state_fixed);
  vpColVector initJoints;
  //joint_offset->setbMcWithMarker(initJoints);
  joint_offset->setbMcFromTf();
  */

  ros::spin();
  return (0);
}
