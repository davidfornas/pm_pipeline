/*
 * Logger
 *  Created on: 17/05/2018
 *      Author: dfornas
 */
#ifndef LOGGER_H_
#define LOGGER_H_


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>


class Logger{

  // Se tto use time stamp as first row or time difference.
  bool use_stamp, stamp_init;


public:

  std::ofstream myfile;

  Logger(std::string file_name, bool default_header = true ){

    myfile.open ( (file_name + std::string(".csv")).c_str() );
    if(default_header)
      setPoseHeader();
  }

  // Store Pose header
  void setPoseHeader(){
    myfile << "Stamp,PoseX,PoseY,PoseZ\n";
  }

  void storePose( geometry_msgs::PoseStamped msg ){
    myfile << msg.header.stamp << ",";
    myfile << msg.pose.position.x << ",";
    myfile << msg.pose.position.y << ",";
    myfile << msg.pose.position.z << "\n";
  }

  ~Logger(){
    myfile.close();
  }

};

#endif
