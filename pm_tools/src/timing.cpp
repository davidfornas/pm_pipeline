/*
 * timing
 * 			 
 *  Created on: 27/01/2016
 *      Author: dfornas
 */
#include <pm_tools/timing.h>

ProgramTimer::ProgramTimer()
{
  startTime_ = clock();
  lastLapTime_ = clock();
}

double ProgramTimer::getLapTimeWithDebug(){
  clock_t end = clock();
  double elapsed =  double(end - lastLapTime_) / CLOCKS_PER_SEC;
  ROS_DEBUG_STREAM("Elapsed time: " << elapsed << "Total time: " << double(end - lastLapTime_) / CLOCKS_PER_SEC);
  lastLapTime_ = clock();
  return elapsed;
}

double ProgramTimer::getLapTime(){
  clock_t end = clock();
  double elapsed =  double(end - lastLapTime_) / CLOCKS_PER_SEC;
  lastLapTime_ = clock();
  return elapsed;
}

std_msgs::Float32 ProgramTimer::getLapTimeMsg(){
  clock_t end = clock();
  double elapsed =  double(end - lastLapTime_) / CLOCKS_PER_SEC;
  lastLapTime_ = clock();
  return toFloat32Msgs(elapsed);
}

double ProgramTimer::getTotalTimeWithDebug(){
  clock_t end = clock();
  double elapsed = double(end - startTime_) / CLOCKS_PER_SEC;
  ROS_DEBUG_STREAM("Total time: " << elapsed);
  return elapsed;
}

double ProgramTimer::getTotalTime(){
  clock_t end = clock();
  double elapsed = double(end - startTime_) / CLOCKS_PER_SEC;
  return elapsed;
}

std_msgs::Float32 ProgramTimer::getTotalTimeMsg(){
  clock_t end = clock();
  double elapsed = double(end - startTime_) / CLOCKS_PER_SEC;
  ROS_DEBUG_STREAM("Total time: " << elapsed);
  return toFloat32Msgs(elapsed);
}

std_msgs::Float32 ProgramTimer::toFloat32Msgs(float msg){
  std_msgs::Float32 rosMsg;
  rosMsg.data = msg;
  return rosMsg;
}

void ProgramTimer::resetTimer(){
  startTime_ = clock();
  lastLapTime_ = clock();
}

SystemTimer::SystemTimer()
{
  startTime_ = ros::Time::now();
  lastLapTime_ = ros::Time::now();
}

double SystemTimer::getLapTimeWithDebug(){
  ros::Time end = ros::Time::now();
  double elapsed =  (end - lastLapTime_).toSec();
  ROS_DEBUG_STREAM("Elapsed time: " << elapsed << "Total time: " << (end - lastLapTime_).toSec());
  lastLapTime_ = ros::Time::now();
  return elapsed;
}

double SystemTimer::getLapTime(){
  ros::Time end = ros::Time::now();
  double elapsed =  (end - lastLapTime_).toSec();
  lastLapTime_ = ros::Time::now();
  return elapsed;
}

std_msgs::Float32 SystemTimer::getLapTimeMsg(){
  ros::Time end = ros::Time::now();
  double elapsed =  (end - lastLapTime_).toSec();
  lastLapTime_ = ros::Time::now();
  return toFloat32Msgs(elapsed);
}

double SystemTimer::getTotalTimeWithDebug(){
  ros::Time end = ros::Time::now();
  double elapsed = (end - startTime_).toSec();
  ROS_DEBUG_STREAM("Total time: " << elapsed);
  return elapsed;
}

double SystemTimer::getTotalTime(){
  ros::Time end = ros::Time::now();
  double elapsed = (end - startTime_).toSec();
  return elapsed;
}

std_msgs::Float32 SystemTimer::getTotalTimeMsg(){
  ros::Time end = ros::Time::now();
  double elapsed = (end - startTime_).toSec();
  ROS_DEBUG_STREAM("Total time: " << elapsed);
  return toFloat32Msgs(elapsed);
}

std_msgs::Float32 SystemTimer::toFloat32Msgs(float msg){
  std_msgs::Float32 rosMsg;
  rosMsg.data = msg;
  return rosMsg;
}

void SystemTimer::resetTimer(){
  startTime_ = ros::Time::now();
  lastLapTime_ = ros::Time::now();
}
