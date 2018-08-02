/*
 * Pose Estimation classes
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */
#ifndef SQ_POSE_ESTIMATION_H
#define SQ_POSE_ESTIMATION_H

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

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

/** Pose Estimation using SQ */
class SQPoseEstimation : public PoseEstimation{

  int cluster_index_, cluster_thereshold_;
  CloudClustering<PointT> * cloud_clustering_;

  CloudPtr sq_cloud_;

  bool use_ceres_, use_region_growing_;
  double region_growing_norm_th_, region_growing_curv_th_;
  double euclidian_tolerance_;

  sq::SuperquadricParameters<double> sq_params_;
  pcl::PolygonMesh sq_mesh_;

public:

  SQPoseEstimation(ros::NodeHandle & nh, CloudPtr source, int cluster_thereshold = 400, double euclidian_tolerance = 0.02 )
          : PoseEstimation(nh, source), cluster_thereshold_(cluster_thereshold), euclidian_tolerance_(euclidian_tolerance){
    use_ceres_ = true;
    use_region_growing_ = false;
  }

  bool initialize(){ return process(); }

  // Use process multiple times with new clouds.
  bool process();

  //Use process next to process other cluster without repeating clustering.
  bool processNext();

  /** Get the object cloud extracted */
  CloudPtr getSQCloud() {return sq_cloud_;}

  /** Set Fitting using LM instead of Ceres */
  void setLMFitting(){ use_ceres_ = false; }

  /** Set region growing and params */
  void setRegionGrowingClustering( double region_growing_norm_th, double region_growing_curv_th ){
    use_region_growing_ = true;
    region_growing_norm_th_ = region_growing_norm_th;
    region_growing_curv_th_ = region_growing_curv_th;
  }

  void display( int ms = 500 );
  void publishResults();

};

#endif // SQ_POSE_ESTIMATION_H
