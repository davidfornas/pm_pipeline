/*
 * pose_estimation.h
 *
 *  Created on: 16/04/2014
 *      Author: dfornas
 */

#ifndef POSEESTIMATION_H_
#define POSEESTIMATION_H_

#include<ros/ros.h>

/** Description
 */
class PoseEstimation {

  ros::NodeHandle * nh_;
  std::string image_topic_, image_info_topic_;


    public:
	/** Constructor.
	 * @param
	 * */
        PoseEstimation(ros::NodeHandle * nh): nh_(nh){
          image_topic_="stereo/left/image_raw";
          image_info_topic_="stereo/left/camera_info";
	}

	void process();

	~PoseEstimation() {}
};

#endif /* POSEESTIMATION_H_ */
