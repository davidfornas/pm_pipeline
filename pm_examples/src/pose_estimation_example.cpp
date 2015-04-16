/** 
 * This program test the pose estimation
 *  Created on: 16/04/2015
 *      Author: dfornas
 */
#include <ros/ros.h>

#include <pm_perception/pose_estimation.h>

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimation_example");
  ros::NodeHandle nh;
  PoseEstimation pose_est;
  pose_est.process();

  // -----Main loop-----
  while (1)
  {
    //Show results
  }
  return (0);
}
