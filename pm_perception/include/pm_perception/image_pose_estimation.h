/*
 * pose_estimation.h Box top face pose estimation from user input clicks. The pose
 * is obtained using 4-6 user clicks.
 *
 *  Created on: 16/04/2014
 *      Author: dfornas
 */

#ifndef IMAGEPOSEESTIMATION_H_
#define IMAGEPOSEESTIMATION_H_

#include<ros/ros.h>
#include <pm_tools/virtual_image.h>
#include <visp/vpHomogeneousMatrix.h>

/** Estimates the pose of a box of known dimensions using user clicks. */
class ImagePoseEstimation
{

  ros::NodeHandle * nh_;
  std::string image_topic_, image_info_topic_;
  vpImagePoint clicks_[4];

public:
  /** Constructor.
   * @nh Node handler
   * @ TODO Add box dimensions and number of points as parameters
   * */
  ImagePoseEstimation(ros::NodeHandle * nh) :
      nh_(nh)
  {
    //Defaults @ TODO unify contructors
    image_topic_ = "stereo/left/image_raw";
    image_info_topic_ = "stereo/left/camera_info";
  }

  ImagePoseEstimation(ros::NodeHandle * nh, std::string im_topic, std::string im_info_topic) :
      nh_(nh), image_topic_(im_topic), image_info_topic_(im_info_topic)
  {
  }

  /** Get image and user input to compute the box pose **/
  vpHomogeneousMatrix process();

  ~ImagePoseEstimation()
  {
  }

private:

  /** Display user clicks in vpDisplay, different versions based on the number of points **/
  void displayClicks(vpImage<vpRGBa> & Ic, int num_clicks);
  void display5Clicks(vpImage<vpRGBa> & Ic, int num_clicks);
  void display6Clicks(vpImage<vpRGBa> & Ic, int num_clicks);
};

#endif /* IMAGEPOSEESTIMATION_H_ */
