/*
 * Pose Estimation classes
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */
#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

class PoseEstimation{

protected:

  CloudPtr cloud_, object_cloud_;
  std::string camera_frame_name, topic_name;
  FrameToTF vispToTF;
  bool debug_;
  bool symmetry_search_, planar_symmetry_, fixed_half_height_;
  double angle_limit_, angle_step_, distance_ratio_step_;

  ros::NodeHandle & nh_;
  ros::Publisher objectPosePublisher;
  ros::Publisher objectParameterPublisher;
  ros::Publisher objectCloudPublisher;
  ros::Publisher objectCloudSizePublisher;

  ros::Publisher estimationStatsPublisher;
  ros::Publisher symmetryStatsPublisher;

public:

  // Object frame with respect to the camera
  vpHomogeneousMatrix  cMo;
  BackgroundRemoval * bg_remove;

  PoseEstimation(ros::NodeHandle & nh, CloudPtr cloud, double distanceThreshold = 0.05) : nh_(nh) {
    cloud_ = cloud;
    camera_frame_name = cloud->header.frame_id;
    bg_remove = new BackgroundRemoval(nh, cloud, distanceThreshold);
    debug_ = false;
    vispToTF.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "/stereo", "/cMo", "cMo");
    symmetry_search_ = false;
    planar_symmetry_ = true;

    objectParameterPublisher = nh.advertise<std_msgs::Float32MultiArray>("object/modelParameters", 10);
    objectPosePublisher      = nh.advertise<geometry_msgs::Pose>("object/pose", 10);
    objectCloudPublisher     = nh.advertise<sensor_msgs::PointCloud2>("object/cloud", 1000);
    objectCloudSizePublisher = nh.advertise<std_msgs::Float32>("object/cloudSize", 10);
    estimationStatsPublisher = nh.advertise<std_msgs::Float32>("stats/estimation", 10);
    symmetryStatsPublisher   = nh.advertise<std_msgs::Float32>("stats/symmetry", 10);
  }

  virtual bool initialize(){ return false;}
  virtual bool process(){ return false;}
  virtual void publishResults() { }

  //Set pose estimation debug, manily visualization with PCL Viewer.
  void setDebug( bool debug ){
    debug_ = debug;
  }

  /** Set new input cloud */
  void setNewCloud(CloudPtr cloud){
    cloud_ = cloud;
    bg_remove->setNewCloud(cloud);
  }

  /** Set background removal mode */
  void setRansacBackgroundFilter( bool ransac_background_filter ){
    bg_remove->setRansacBackgroundFilter(ransac_background_filter);
  }

  /** Set plane segmentation parameters: distance to the inliers to the plane
   * and number of iterations.
   */
  void setPlaneSegmentationParams(double distanceThreshold = 0.03, int iterations = 100){
    bg_remove->setPlaneSegmentationParams( distanceThreshold, iterations );
  }

  /** Change camera frame name */
  void setCameraFrameName(std::string name){
    camera_frame_name = name;
  }

  /** Get the object frame with respect to the camera frame */
  vpHomogeneousMatrix get_cMo() {return cMo;}

  /** Get the object cloud extracted */
  CloudPtr getObjectCloud() {return object_cloud_;}

  /** Set to use symmetry search method. angle limit=0 means only distance search */
  void setSymmetrySearchParams(double angle_limit = 0.55, double angle_step = 0.04, double distance_ratio_step = 0.1, bool fixed_half_height = false){
    angle_limit_ = angle_limit;
    angle_step_ = angle_step;
    distance_ratio_step_ = distance_ratio_step;
    symmetry_search_ = true;
    fixed_half_height_ = fixed_half_height;
  }

  void setAxisSymmetryMode(){
    planar_symmetry_ = false;
    if(!symmetry_search_) setSymmetrySearchParams();
  }

  /** For RANSAC based approaches, do symmetry estimation and estimate SQ shape **/
  void estimateSQ( CloudPtr & sq_cloud  );

};

#endif // SAC_POSE_ESTIMATION_H
