/** 
 * This program test the pose estimation
 *  Created on: 16/04/2015
 *      Author: dfornas
 */
#include <ros/ros.h>

#include <pm_perception/pose_estimation.h>
#include <pm_tools/visp_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimation_example");
  ros::NodeHandle nh;


  int counter = 0, left_weight=0, right_weight=0;
  vpHomogeneousMatrix left_avg, right_avg;
  vpHomogeneousMatrix left, right;
  while (ros::ok() && counter < 6)
  {
    if (counter % 2 == 0)
    {
      ROS_INFO("Left");
      PoseEstimation pose_est(&nh, "stereo/left/image_rect", "stereo/left/camera_info");
      left = pose_est.process();
      //v.removeTransform("a");
      //v.addTransform(left, "stereo", "left_pose", "a");

      //Average...
      if (left_weight==0) left_avg=left;
      else left_avg=VispTools::weightedAverage(left_avg, left_weight, left);

      left_weight++;
      //v.resetTransform(left_avg, "3");
    }

    if (counter % 2 == 1)
    {
      ROS_INFO("Right");
      PoseEstimation pose_est_r(&nh, "stereo/right/image_rect", "stereo/right/camera_info");
      right = pose_est_r.process();
      //v.resetTransform(right, "1");

      //Average...
      if (right_weight==0) right_avg=right;
      else right_avg=VispTools::weightedAverage(right_avg, left_weight, right);
      right_weight++;
      //v.resetTransform(right_avg, "4");
    }
    //v.publish();
    counter++;
  }

  VispToTF v;
  v.addTransform(left, "stereo", "left_pose", "a");
  v.addTransform(right, "stereo_right", "right_pose", "1");
  v.addTransform(vpHomogeneousMatrix(0.09, 0, 0, 0, 0, 0), "stereo", "stereo_right", "2");
  v.addTransform(left_avg, "stereo", "left_average", "3");
  v.addTransform(right_avg, "stereo_right", "right_average", "4");

  ros::Rate r(10);
  while(ros::ok()){
    v.publish();
    r.sleep();
  }

  return (0);
}

