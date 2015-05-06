/*
 * trajectory_following.cpp
 *
 *  Created on: 24/03/2014
 *      Author: dfornas
 */

#include <pm_manipulation/trajectory_following.h>

/* NEEDED TFs. THese TF should be publish to simulate the
 * real arm TF tree  @ TODO Move to roslaunch */

//rosrun tf static_transform_publisher 0 0 1.08 0 0 1 0 new_base_link kinematic_base 200
//rosrun tf static_transform_publisher 0 0 1.08 0 0 1 0 base_link kinematic_base 200
//rosrun tf static_transform_publisher 0.4 -0.06 1.05 -0.176 -0.174 0.685 0.685 base_link stereo_down 100
//rosrun tf static_transform_publisher 1.184 0 1.825 0 0 0 world base_link 100
void TrajectoryFollowing::moveToNextWaypoint()
{

  path_index_ = (path_index_ + 1) % path_.poses.size();
  //Obtain next point
  vpTranslationVector t(path_.poses[path_index_].pose.position.x, path_.poses[path_index_].pose.position.y,
                        path_.poses[path_index_].pose.position.z);
  vpQuaternionVector quat(path_.poses[path_index_].pose.orientation.x, path_.poses[path_index_].pose.orientation.y,
                          path_.poses[path_index_].pose.orientation.z, path_.poses[path_index_].pose.orientation.w);

  //Target frame, assuming w.r.t. vehicle_base.
  vpHomogeneousMatrix target(t, quat);

  //Arm vehicle kinematics
  vpColVector joints(8);
  joints = robot_->vehicleArmIK(target);

  //std::cout << "Reachable position vehicle config: " << std::endl << joints << std::endl;
  ROS_INFO_STREAM("Desired end effector position" << std::endl << target);
  tf::StampedTransform debug(VispTools::tfTransFromVispHomog(target), ros::Time::now(), "/base_link", "/target"); //kinematic_base <=> vehicle
  broadcaster_->sendTransform(debug);

  //Copy results
  current_config_ = joints;

  //Forward arm kinematics.
  vpColVector arm_joints(5);
  arm_joints[0] = joints[4];
  arm_joints[1] = joints[5];
  arm_joints[2] = joints[6];
  arm_joints[3] = joints[7];
  arm_joints[4] = 0.5; /// @TODO from gripper aperture parameter
  vpHomogeneousMatrix target_fk = robot_->directKinematics(arm_joints);
  ROS_INFO_STREAM("Resulting configuration" << std::endl << joints);

  //Publish FK of found joint config
  tf::StampedTransform fk(VispTools::tfTransFromVispHomog(target_fk), ros::Time::now(), "/kinematic_base",
                          "/reachable_cMg"); //kinematic_base <=> vehicle

  vpHomogeneousMatrix x(joints[0], 0, 0, 0, 0, 0);
  vpHomogeneousMatrix y(0, joints[1], 0, 0, 0, 0);
  vpHomogeneousMatrix z(0, 0, joints[2], 0, 0, 0);
  vpHomogeneousMatrix yaw(0, 0, 0, 0, 0, joints[3]);
  vpHomogeneousMatrix pos = x * y * z * yaw;
  tf::StampedTransform v(VispTools::tfTransFromVispHomog(pos), ros::Time::now(), "/base_link", "/new_base_link"); //kinematic_base <=> vehicle

  broadcaster_->sendTransform(fk);
  broadcaster_->sendTransform(v);

  //Next point service warning for benchmark platform

  //Send orders to vehicle/arm.
  sensor_msgs::JointState js;
  js.name.push_back(std::string("Slew"));
  js.position.push_back(joints[4]);
  js.name.push_back(std::string("Shoulder"));
  js.position.push_back(joints[5]);
  js.name.push_back(std::string("Elbow"));
  js.position.push_back(joints[6]);
  js.name.push_back(std::string("JawRotate"));
  js.position.push_back(joints[7]);
  js.name.push_back(std::string("JawOpening"));
  js.position.push_back(0.5);
  js_pub_.publish(js);

  vpHomogeneousMatrix yaw2(0, 0, 0, 0, 0, joints[3]);
  vpQuaternionVector q;
  yaw2.extract(q);

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = 1.184 + joints[0];
  odom.pose.pose.position.y = 0 + joints[1];
  odom.pose.pose.position.z = 1.825 + joints[2];
  odom.pose.pose.orientation.x = q.x(); //rot.x();//YAW FROM joints[3]
  odom.pose.pose.orientation.y = q.y(); //rot.y();
  odom.pose.pose.orientation.z = q.z(); //rot.z();
  odom.pose.pose.orientation.w = q.w(); //rot.w();
  position_pub_.publish(odom);

  ros::spinOnce();
}

