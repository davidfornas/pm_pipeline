/*
 * Logger
 *  Created on: 17/05/2018
 *      Author: dfornas
 */
#ifndef LOGGER_H_
#define LOGGER_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>

class SimplePoseLogger{

public:

  std::ofstream myfile;

  SimplePoseLogger(std::string file_name, bool default_header = true );
  void setPoseHeader();
  void storePose( geometry_msgs::PoseStamped msg );

  ~SimplePoseLogger();

};

class AllDataLogger{

  std::ofstream file_;
  std::string objectId_;
  geometry_msgs::Pose pose_;
  std::vector<float> modelParameters_;
  float cloudSize_, symmetry_, estimation_, background_, filter_, process_, load_;

  ros::Subscriber cloudSizeSubscriber_;
  ros::Subscriber symmetrySubscriber_;
  ros::Subscriber estimationSubscriber_;
  ros::Subscriber backgroundSubscriber_;
  ros::Subscriber filterSubscriber_;
  ros::Subscriber loadSubscriber_;
  ros::Subscriber processSubscriber_;

  ros::Subscriber modelParametersSubscriber_;
  ros::Subscriber poseSubscriber_;

  int aliveSubscribers_;


public:
  AllDataLogger(std::string file_name, std::string objectId, ros::NodeHandle & nh, bool appendMode = true, bool getAverages = true);

  int getAliveSubscribers();

  void writeSQHeader();
  void writePCAHeader();
  void writeRANSACCylinderHeader();
  void writeRANSACBoxHeader();
  void writeRANSACSphereHeader();
  void writeRow();

  void modelParametersCallback(const std_msgs::Float32MultiArrayConstPtr& m);

  void cloudSizeCallback(const std_msgs::Float32ConstPtr& m);
  void symmetryCallback(const std_msgs::Float32ConstPtr& m);
  void backgroundCallback(const std_msgs::Float32ConstPtr& m);
  void estimationCallback(const std_msgs::Float32ConstPtr& m);
  void filterCallback(const std_msgs::Float32ConstPtr& m);
  void loadCallback(const std_msgs::Float32ConstPtr& m);
  void processCallback(const std_msgs::Float32ConstPtr& m);

  void poseCallback(const geometry_msgs::PoseConstPtr& m);

  ~AllDataLogger();

};

#endif
