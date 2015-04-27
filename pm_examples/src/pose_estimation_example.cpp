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

  VispToTF v;

  v.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "stereo", "left_pose", "0");
  v.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "stereo_right", "right_pose", "1");
  v.addTransform(vpHomogeneousMatrix(0.06, 0, 0, 0, 0, 0), "stereo", "stereo_right", "2");
  v.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "stereo", "left_average", "3");
  v.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "stereo_right", "right_average", "4");

  int counter = 0, left_weight=0, right_weight=0;
  vpHomogeneousMatrix left_avg, right_avg;
  while (ros::ok())
  {
    if (counter % 2 == 0)
    {
      PoseEstimation pose_est(&nh, "stereo/left/image_rect", "stereo/left/camera_info");
      vpHomogeneousMatrix left(pose_est.process());
      v.resetTransform(left, "0");

      //Average...
      if (left_weight==0) left_avg=left;
      else left_avg=VispTools::weightedAverage(left_avg, left_weight, left);

      left_weight++;
      v.resetTransform(left, "3");
    }

    if (counter % 2 == 1)
    {
      PoseEstimation pose_est_r(&nh, "stereo/right/image_rect", "stereo/right/camera_info");
      vpHomogeneousMatrix right(pose_est_r.process());
      v.resetTransform(right, "1");

      //Average...
      if (right_weight==0) right_avg=right;
      else right_avg=VispTools::weightedAverage(right_avg, left_weight, right);

      right_weight++;
      v.resetTransform(right, "4");
    }



    v.publish();
    counter++;
  }

  return (0);
}

