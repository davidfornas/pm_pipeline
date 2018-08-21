/*
 * timing: Classes used to measure execution times and more.
 *  Created on: 27/01/2016
 *      Author: dfornas
 */
#ifndef TIMING_H_
#define TIMING_H_

//Time measures with C library, should use another library such as ROS
#include <ctime>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>

class ProgramTimer
{
	clock_t startTime_, lastLapTime_;

public:

  ProgramTimer();

	void resetTimer();

	double getLapTimeWithDebug();
	double getLapTime();
	std_msgs::Float32 getLapTimeMsg();

	double getTotalTimeWithDebug();
	double getTotalTime();
	std_msgs::Float32 getTotalTimeMsg();

	static std_msgs::Float32 toFloat32Msgs(const float msg);

};

class SystemTimer
{
  ros::Time startTime_, lastLapTime_;

public:

  SystemTimer();

  void resetTimer();

  double getLapTimeWithDebug();
  double getLapTime();
  std_msgs::Float32 getLapTimeMsg();

  double getTotalTimeWithDebug();
  double getTotalTime();
  std_msgs::Float32 getTotalTimeMsg();

  static std_msgs::Float32 toFloat32Msgs(const float msg);

};

#endif
