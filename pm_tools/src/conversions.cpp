/*
 * conversions
 * 			 
 *  Created on: 13/01/2016
 *      Author: dfornas
 */

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <cstdlib>
#include <iostream>


void imu_callback(const sensor_msgs::Imu& msg) {
	tf::Quaternion q( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w );
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	std::cout << "R,P,Y: " << roll << ", " << pitch << ", " << yaw << std::endl;
}

int main(int argc, char * argv[]){
	ros::init(argc, argv, "Conversion");
	ros::NodeHandle nh;
	std::string imu_topic("imu/data");
        ros::Subscriber imu_sub=nh.subscribe(imu_topic, 1, imu_callback);

	ros::spin();

	return 0;
}

