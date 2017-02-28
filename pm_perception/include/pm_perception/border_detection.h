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

#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>
#include <pm_tools/tf_tools.h>

#include <visp/vpHomogeneousMatrix.h>

typedef pcl::PointXYZRGB PointT;

/** Border detection base class */
class BorderDetection
{

protected:

  FrameToTF vispToTF;
  pcl::PointCloud<PointT>::Ptr cloud_, border_cloud_;
  nav_msgs::Path path_;
  double subsampling_distance_;

public:

  void publishTF()
  {
    vispToTF.publish();
  }

  /** Constructor, subsampling distance 0 means not path subsampling */
  BorderDetection(pcl::PointCloud<PointT>::Ptr cloud) :
      cloud_(cloud), subsampling_distance_(0)
  {
  }

  /** Obtain points */
  virtual void process()
  {
  }
  ;

  /** Generate path from points */
  virtual void generatePath()
  {
  }
  ;

  /** Publish path @ TODO Topic name as parameter */
  void publishPath(ros::NodeHandle & n)
  {
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path", 10);
    path_pub.publish(path_);
  }

  /** Transform path frame_id using TF tree */
  void transformPathFrame(std::string new_frame);

  /** Path getter */
  nav_msgs::Path getPath()
  {
    return path_;
  }

  /** Store path as text in RPY and quaternion form */
  void savePathToFile();

  /** Store trajectory in initialized cloud */
  void getTrajectory(pcl::PointCloud<PointT>::Ptr & trajectory)
  {
    trajectory = border_cloud_;
  }

  /* Set sumsampling distance between path points. Distance:0 means subsampling disabled */
  void setSubsamplingDistance(double distance)
  {
    subsampling_distance_ = distance;
  }
  virtual ~BorderDetection()
  {
  }

};
typedef boost::shared_ptr<BorderDetection> BorderDetectionPtr;

/** Border detection with range image. The point cloud is converted in a range
 * image and then border detection is computed taking into account range jumps. */
class RangeImageBorderDetection : public BorderDetection
{

  pcl::PointCloud<pcl::PointWithRange>::Ptr border_cloud_with_ranges_;

public:

  RangeImageBorderDetection(pcl::PointCloud<PointT>::Ptr cloud) :
      BorderDetection(cloud)
  {
  }
  void process();
  void generatePath();
  void getTrajectoryWithRange(pcl::PointCloud<pcl::PointWithRange>::Ptr & trajectory)
  {
    trajectory = border_cloud_with_ranges_;
  }
  ~RangeImageBorderDetection()
  {
  }

};

/** Border detection using concave hull. The point cloud is segmented using RANSAC to extract
 * a plane and a cylinder. Then the concave hull of the segmented object is used to generate
 * the path. @ TODO Implement concave/convex hull switching. */
class ConcaveHullBorderDetection : public BorderDetection
{
  vpColVector plane_normal_;

public:

  ConcaveHullBorderDetection(pcl::PointCloud<PointT>::Ptr cloud) :
      BorderDetection(cloud)
  {
  }
  void process();
  void generatePath();
  ~ConcaveHullBorderDetection()
  {
  }

};

#endif /* BORDERDETECTION_H_ */
