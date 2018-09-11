/*
 * timing: Classes used to average topic readings
 *  Created on: 09/08/2018
 *      Author: dfornas
 */
#ifndef AVERAGE_H_
#define AVERAGE_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>

class AverageFloat32
{
  ros::NodeHandle & nh_;
  ros::Publisher averagePublisher, stdDevPublisher;
  ros::Subscriber averageSubscriber;

  std_msgs::Float32 average_, stdDev_, averageOfSqares_;
  int count_;
  
public:

  AverageFloat32(ros::NodeHandle & nh, const char * in, const char * mean, const char * stdDev);
  void msgCallback(const std_msgs::Float32ConstPtr& m);
  
};

class AverageFloat32MultiArray
{
  ros::NodeHandle & nh_;
  ros::Publisher averagePublisher, stdDevPublisher;
  ros::Subscriber averageSubscriber;

  std_msgs::Float32MultiArray averages_, stdDevs_, averageOfSqares_;
  int count_;

public:

  AverageFloat32MultiArray(ros::NodeHandle & nh, const char * in, const char * mean, const char * stdDev);
  void msgCallback(const std_msgs::Float32MultiArrayConstPtr& m);

};

class AveragePose
{
  ros::NodeHandle & nh_;
  ros::Publisher averagePublisher, stdDevPublisher;
  ros::Subscriber averageSubscriber;

  geometry_msgs::Pose averagePose_;
  std::vector<std_msgs::Float64MultiArray> poses_;
  std::vector<double> stdDevs_;
  int count_;

  void computeAverage();

public:

  AveragePose(ros::NodeHandle & nh, const char * in, const char * mean, const char * stdDev);
  void msgCallback(const geometry_msgs::PoseConstPtr& m);

};

#endif
