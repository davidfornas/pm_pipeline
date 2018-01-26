#ifndef JOINTOFFSET_H
#define JOINTOFFSET_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
#include "mar_robot_arm5e/ARM5Arm.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>

class JointOffset{

	ros::NodeHandle nh_;
	ARM5Arm *robot;
	vpColVector offset_;
	vpHomogeneousMatrix bMc;
	tf::StampedTransform cMm_tf;
	bool cMm_found;
	bool bMc_init;
	ros::Subscriber joint_state_sub;
	ros::Subscriber marker_sub;
	ros::Publisher joint_state_pub;
	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;

	void readJointsCallback(const sensor_msgs::JointState::ConstPtr& m);
	void markerCallback(const geometry_msgs::PoseStamped::ConstPtr& m);
	vpHomogeneousMatrix markerToEndEffector(tf::Transform cMm_tf);

public:
	int reset_bMc(vpColVector initial_posture);
	JointOffset(ros::NodeHandle& nh, std::string topic_joint_state, std::string topic_command_joint, std::string topic_joint_state_fixed);
	int get_bMc(vpHomogeneousMatrix &bMc_ret){
		if(bMc_init){
			bMc_ret=bMc;
			return 0;
		}
		return -1;
	}

};



#endif
