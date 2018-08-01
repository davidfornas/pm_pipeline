/*
 * Pose Estimation classes
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */
#ifndef PCA_POSE_ESTIMATION_H
#define PCA_POSE_ESTIMATION_H

#include <pm_tools/tf_tools.h>
#include <pm_tools/marker_tools.h>
#include <pm_tools/pcl_segmentation.h>
#include <pm_tools/pcl_clustering.h>
#include <pm_perception/background_removal.h>

#include <pm_superquadrics/fit_superquadric_ceres.h>

#include <pcl/io/pcd_io.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpPoint.h>
#include <visp/vpHomogeneousMatrix.h>

#include <tf/transform_datatypes.h>
#include <list>

#include <pm_perception/pose_estimation.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

/** Pose Estimation using PCA */
class PCAPoseEstimation : public PoseEstimation{

  int cluster_index_, cluster_thereshold_;
  CloudClustering<PointT> * cloud_clustering_;

  float width_, height_, depth_;

public:

  PCAPoseEstimation(ros::NodeHandle & nh, CloudPtr source, int cluster_thereshold = 400) : PoseEstimation(nh, source){
    cluster_thereshold_ = cluster_thereshold;
  }

  void publishResults();
  bool initialize(){ return process(); }

  // Use process multiple times with new clouds.
  bool process();

  //Use process next to process other cluster without repeating clustering.
  bool processNext();
};

#endif