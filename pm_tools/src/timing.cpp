/*
 * timing
 * 			 
 *  Created on: 27/01/2016
 *      Author: dfornas
 */
#include <pm_tools/timing.h>

Timing::Timing()
{
  startTime_ = clock();
  lastLapTime_ = clock();
}

double Timing::getLapTimeWithDebug(){
  clock_t end = clock();
  double elapsed =  double(end - lastLapTime_) / CLOCKS_PER_SEC;
  ROS_DEBUG_STREAM("Elapsed time: " << elapsed << "Total time: " << double(end - lastLapTime_) / CLOCKS_PER_SEC);
  lastLapTime_ = clock();
  return elapsed;
}

double Timing::getLapTime(){
  clock_t end = clock();
  double elapsed =  double(end - lastLapTime_) / CLOCKS_PER_SEC;
  lastLapTime_ = clock();
  return elapsed;
}

std_msgs::Float32 Timing::getLapTimeMsg(){
  clock_t end = clock();
  double elapsed =  double(end - lastLapTime_) / CLOCKS_PER_SEC;
  lastLapTime_ = clock();
  return toFloat32Msgs(elapsed);
}

double Timing::getTotalTimeWithDebug(){
  clock_t end = clock();
  double elapsed = double(end - startTime_) / CLOCKS_PER_SEC;
  ROS_DEBUG_STREAM("Total time: " << elapsed);
  return elapsed;
}

double Timing::getTotalTime(){
  clock_t end = clock();
  double elapsed = double(end - startTime_) / CLOCKS_PER_SEC;
  ROS_DEBUG_STREAM("Total time: " << elapsed);
  return elapsed;
}

std_msgs::Float32 Timing::getTotalTimeMsg(){
  clock_t end = clock();
  double elapsed = double(end - startTime_) / CLOCKS_PER_SEC;
  ROS_DEBUG_STREAM("Total time: " << elapsed);
  return toFloat32Msgs(elapsed);
}

std_msgs::Float32 Timing::toFloat32Msgs(float msg){
  std_msgs::Float32 rosMsg;
  rosMsg.data = msg;
  return rosMsg;
}

void Timing::resetTimer(){
  startTime_ = clock();
  lastLapTime_ = clock();
}
