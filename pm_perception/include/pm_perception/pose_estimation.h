/*
 * pose_estimation.h
 *
 *  Created on: 16/04/2014
 *      Author: dfornas
 */

#ifndef POSEESTIMATION_H_
#define POSEESTIMATION_H_

#include<ros/ros.h>
#include <pm_tools/virtual_image.h>

#include <visp/vpHomogeneousMatrix.h>

/** Description
 */
class PoseEstimation
{

  ros::NodeHandle * nh_;
  std::string image_topic_, image_info_topic_;
  vpImagePoint clicks_[4];


public:
  /** Constructor.
   * @param
   * */
  PoseEstimation(ros::NodeHandle * nh) :
      nh_(nh)
  {
    image_topic_ = "stereo/left/image_raw";
    image_info_topic_ = "stereo/left/camera_info";
  }

  PoseEstimation(ros::NodeHandle * nh, std::string im_topic, std::string im_info_topic) :
      nh_(nh), image_topic_(im_topic), image_info_topic_(im_info_topic){}

  vpHomogeneousMatrix process();

  ~PoseEstimation()
  {
  }

private:

  void displayClicks( vpImage<vpRGBa> & Ic, int num_clicks );
  void display5Clicks( vpImage<vpRGBa> & Ic, int num_clicks );
  void display6Clicks( vpImage<vpRGBa> & Ic, int num_clicks );


};

#endif /* POSEESTIMATION_H_ */
