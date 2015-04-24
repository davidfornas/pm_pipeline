/** 
 * This program test the pose estimation
 *  Created on: 16/04/2015
 *      Author: dfornas
 */
#include <ros/ros.h>

#include <pm_perception/pose_estimation.h>
#include <pm_tools/visp_tools.h>

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimation_example");
  ros::NodeHandle nh;

  PoseEstimation pose_est(&nh, "stereo/left/image_raw", "stereo/left/camera_info");
  vpHomogeneousMatrix left(pose_est.process());

  PoseEstimation pose_est_r(&nh, "stereo/right/image_raw", "stereo/right/camera_info");
  vpHomogeneousMatrix right(pose_est_r.process());
  vpHomogeneousMatrix left_to_right(0.06,0,0,0,0,0);

  VispToTF v;

  v.addTransform(left, "stereo", "left_pose", "0");
  v.addTransform(right, "stereo_right", "right_pose", "1");
  v.addTransform(left_to_right, "stereo", "stereo_right", "2");

  ros::Rate r(10);
  while(ros::ok()){
    v.publish();
    r.sleep();
  }



  return (0);
}


