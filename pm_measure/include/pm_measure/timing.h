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

class Timing
{
	clock_t start_time_, last_lap_time_;
public:

  Timing()
  {
	  start_time_ = clock();
	  last_lap_time_ = clock();
  }

  /** Display current time and total time. **/
  bool lap(){
	  clock_t end = clock();
	  ROS_DEBUG_STREAM("Elapsed time: " << double(end - last_lap_time_) / CLOCKS_PER_SEC << "Total time: " << double(end - start_time_) / CLOCKS_PER_SEC);
	  last_lap_time_ = clock();
  }

  /** Display total time and restart. **/
  bool total(){
	  clock_t end = clock();
	  ROS_DEBUG_STREAM("Total time: " << double(end - start_time_) / CLOCKS_PER_SEC);
	  last_lap_time_ = clock();
	  start_time_ = clock();
  }

  /** Remove the plane from the cloud, easy to use method.  */
  static void now();//pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out, int iteration=100, double threshold=0.06);

};

#endif
