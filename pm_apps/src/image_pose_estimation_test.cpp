/** 
 * This program test the pose estimation using a rectified camera and the pose
 * estimation tool, then publish results in the TF tree.
 *  Created on: 16/04/2015
 *      Author: dfornas
 */

#include <ros/ros.h>
#include <pm_perception/image_pose_estimation.h>
#include <pm_tools/tf_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimation_example");
  ros::NodeHandle nh;

  //Average pose from various left and right images.
  int counter = 0, left_weight = 0, right_weight = 0;
  int num_images = 3; // @ TODO Get it from ROS param or cmd line.
  vpHomogeneousMatrix left_avg, right_avg;
  vpHomogeneousMatrix left, right;

  while (ros::ok() && counter < 2 * num_images)
  {
    if (counter % 2 == 0)
    {
      ROS_DEBUG("Estimating from left camera");
      ImagePoseEstimation pose_est(&nh, "stereo/left/image_rect", "stereo/left/camera_info");
      left = pose_est.process();

      if (left_weight == 0)
        left_avg = left;
      else
        left_avg = VispTools::weightedAverage(left_avg, left_weight, left);
      left_weight++;
    }

    if (counter % 2 == 1)
    {
      ROS_DEBUG("Estimating from right camera");
      ImagePoseEstimation pose_est_r(&nh, "stereo/right/image_rect", "stereo/right/camera_info");
      right = pose_est_r.process();

      if (right_weight == 0)
        right_avg = right;
      else
        right_avg = VispTools::weightedAverage(right_avg, left_weight, right);
      right_weight++;
    }
    counter++;
  }

  //Publish results
  // @ TODO Use parameter server
  // @ TODO Publish TF while averaging, not only final result.
  FrameToTF v;
  v.addTransform(left, "stereo", "left_pose", "0");
  v.addTransform(right, "stereo_right", "right_pose", "1");
  v.addTransform(vpHomogeneousMatrix(0.09, 0, 0, 0, 0, 0), "stereo", "stereo_right", "2");
  v.addTransform(left_avg, "stereo", "left_average", "3");
  v.addTransform(right_avg, "stereo_right", "right_average", "4");

  ros::Rate r(5);
  while (ros::ok())
  {
    v.publish();
    r.sleep();
  }
  return (0);
}

