/*
 * Logger
 *  Created on: 17/05/2018
 *      Author: dfornas
 */
#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pm_tools/timing.h>

class SimplePoseLogger{

public:

  std::ofstream myfile;

  SimplePoseLogger(std::string file_name, bool default_header = true );
  void setPoseHeader();
  void storePose( geometry_msgs::PoseStamped msg );

  ~SimplePoseLogger();

};

class AllDataSingleLogger{

  ros::NodeHandle & nh;

  std::ofstream file_;
  std::string objectId_;
  geometry_msgs::Pose pose_;
  std::vector<float> modelParameters_;
  std::vector<float> poseStdDev_;
  float originalCloudSize_, filteredCloudSize_, noBackgroundCloudSize_, cloudSize_, symmetry_, estimation_, background_, filter_, process_, load_;

  ros::Subscriber originalCloudSizeSubscriber_;
  ros::Subscriber filteredCloudSizeSubscriber_;
  ros::Subscriber noBackgroundcloudSizeSubscriber_;
  ros::Subscriber cloudSizeSubscriber_;
  ros::Subscriber symmetrySubscriber_;
  ros::Subscriber estimationSubscriber_;
  ros::Subscriber backgroundSubscriber_;
  ros::Subscriber filterSubscriber_;
  ros::Subscriber loadSubscriber_;
  ros::Subscriber processSubscriber_;
  ros::Subscriber modelParametersSubscriber_;
  ros::Subscriber poseSubscriber_;

  bool alive_originalCloudSizeSubscriber_;
  bool alive_filteredCloudSizeSubscriber_;
  bool alive_noBackgroundcloudSizeSubscriber_;
  bool alive_cloudSizeSubscriber_;
  bool alive_symmetrySubscriber_;
  bool alive_estimationSubscriber_;
  bool alive_backgroundSubscriber_;
  bool alive_filterSubscriber_;
  bool alive_loadSubscriber_;
  bool alive_processSubscriber_;
  bool alive_modelParametersSubscriber_;
  bool alive_poseSubscriber_;

  bool areAllSubscribersAlive();
  void subscribeToAllAvg();
  void subscribeToAllStdDev();

  void modelParametersCallback(const std_msgs::Float32MultiArrayConstPtr& m);
  void originalCloudSizeCallback(const std_msgs::Float32ConstPtr& m);
  void filteredCloudSizeCallback(const std_msgs::Float32ConstPtr& m);
  void noBackgroundCloudSizeCallback(const std_msgs::Float32ConstPtr& m);
  void cloudSizeCallback(const std_msgs::Float32ConstPtr& m);
  void symmetryCallback(const std_msgs::Float32ConstPtr& m);
  void backgroundCallback(const std_msgs::Float32ConstPtr& m);
  void estimationCallback(const std_msgs::Float32ConstPtr& m);
  void filterCallback(const std_msgs::Float32ConstPtr& m);
  void loadCallback(const std_msgs::Float32ConstPtr& m);
  void processCallback(const std_msgs::Float32ConstPtr& m);
  void poseCallback(const geometry_msgs::PoseConstPtr& m);
  void poseStdDevCallback(const std_msgs::Float32MultiArrayConstPtr& m);

public:
  AllDataSingleLogger(std::string file_name, std::string objectId, ros::NodeHandle & nh, bool appendMode = true, bool stdDevInsteadOfAvg = false);

  void writeSQHeader();
  void writePCAHeader();
  void writeRANSACCylinderHeader();
  void writeRANSACBoxHeader();
  void writeRANSACSphereHeader();
  void writeRow();
  void writeRowStdDev();

  ~AllDataSingleLogger();

};

class AllDataMultiLogger{

  ros::NodeHandle & nh;

  std::ofstream file_;
  std::string objectId_;
  geometry_msgs::Pose pose_;
  std::vector<float> modelParameters_;
  float originalCloudSize_, filteredCloudSize_, noBackgroundCloudSize_, cloudSize_, symmetry_, estimation_, background_, filter_, process_, load_;

  bool timerStarted_;
  SystemTimer timer;

  ros::Subscriber originalCloudSizeSubscriber_;
  ros::Subscriber filteredCloudSizeSubscriber_;
  ros::Subscriber noBackgroundcloudSizeSubscriber_;
  ros::Subscriber cloudSizeSubscriber_;
  ros::Subscriber symmetrySubscriber_;
  ros::Subscriber estimationSubscriber_;
  ros::Subscriber backgroundSubscriber_;
  ros::Subscriber filterSubscriber_;
  ros::Subscriber loadSubscriber_;
  ros::Subscriber processSubscriber_;
  ros::Subscriber modelParametersSubscriber_;
  ros::Subscriber poseSubscriber_;

  bool alive_originalCloudSizeSubscriber_;
  bool alive_filteredCloudSizeSubscriber_;
  bool alive_noBackgroundCloudSizeSubscriber_;
  bool alive_cloudSizeSubscriber_;
  bool alive_symmetrySubscriber_;
  bool alive_estimationSubscriber_;
  bool alive_backgroundSubscriber_;
  bool alive_filterSubscriber_;
  bool alive_loadSubscriber_;
  bool alive_processSubscriber_;
  bool alive_modelParametersSubscriber_;
  bool alive_poseSubscriber_;

  bool areAllSubscribersAlive();
  void subscribeToAll();

  void writeRANSACCylinderHeader(); //mode 1
  void writeRANSACBoxHeader(); //mode 2
  void writeRANSACSphereHeader(); //mode 3
  void writeSQHeader(); //mode 4
  void writePCAHeader(); //mode 5
  void writeRow();

  void modelParametersCallback(const std_msgs::Float32MultiArrayConstPtr& m);
  void originalCloudSizeCallback(const std_msgs::Float32ConstPtr& m);
  void filteredCloudSizeCallback(const std_msgs::Float32ConstPtr& m);
  void noBackgroundCloudSizeCallback(const std_msgs::Float32ConstPtr& m);
  void cloudSizeCallback(const std_msgs::Float32ConstPtr& m);
  void symmetryCallback(const std_msgs::Float32ConstPtr& m);
  void backgroundCallback(const std_msgs::Float32ConstPtr& m);
  void estimationCallback(const std_msgs::Float32ConstPtr& m);
  void filterCallback(const std_msgs::Float32ConstPtr& m);
  void loadCallback(const std_msgs::Float32ConstPtr& m);
  void processCallback(const std_msgs::Float32ConstPtr& m);
  void poseCallback(const geometry_msgs::PoseConstPtr& m);

public:
  AllDataMultiLogger(std::string file_name, std::string objectId, ros::NodeHandle & nh, int mode = 1);
  ~AllDataMultiLogger();

};

#endif
