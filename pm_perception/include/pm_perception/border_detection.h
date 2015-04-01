/*
 * border_detection.h
 *
 *  Created on: 18/03/2014
 *      Author: dfornas
 */

#ifndef BORDERDETECTION_H_
#define BORDERDETECTION_H_

#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <nav_msgs/Path.h>

#include <pm_tools/visp_tools.h>

//VISP
#include <visp/vpHomogeneousMatrix.h>

typedef pcl::PointXYZRGB PointT;

/** Border detection base class
 */
class BorderDetection {

protected:

  VispToTF vispToTF;
  pcl::PointCloud<PointT>::Ptr cloud_, border_cloud_;
  nav_msgs::Path path_;

public:

  void publishTF(){ vispToTF.publish(); }

  BorderDetection(pcl::PointCloud<PointT>::Ptr cloud) : cloud_(cloud) {}

  virtual void process(){};

  virtual void generatePath(){};
  void publishPath(ros::NodeHandle & n){
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path", 10);
    path_pub.publish(path_);
  }
  void transformPathFrame(std::string new_frame);
  nav_msgs::Path getPath(){ return path_;}
  void savePathToFile();

  void getTrajectory(pcl::PointCloud<PointT>::Ptr & trajectory){ trajectory=border_cloud_; }
  virtual ~BorderDetection() {}

};
typedef boost::shared_ptr<BorderDetection> BorderDetectionPtr;

class RangeImageBorderDetection : public BorderDetection {
  pcl::PointCloud<pcl::PointWithRange>::Ptr border_cloud_with_ranges_;
public:

  RangeImageBorderDetection(pcl::PointCloud<PointT>::Ptr cloud) : BorderDetection(cloud) {}
  void process();
  void generatePath();
  void getTrajectoryWithRange(pcl::PointCloud<pcl::PointWithRange>::Ptr & trajectory){ trajectory=border_cloud_with_ranges_; }
  ~RangeImageBorderDetection(){}

};

class ConcaveHullBorderDetection : public BorderDetection {

  vpColVector plane_normal_;

public:

  ConcaveHullBorderDetection(pcl::PointCloud<PointT>::Ptr cloud) : BorderDetection(cloud) {}
  void process();
  void generatePath();
  ~ConcaveHullBorderDetection(){}

};


#endif /* BORDERDETECTION_H_ */
