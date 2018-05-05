/*
 * Pose Estimation classes
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */

#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H


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

public:

  // Object frame with respect to the camera
  vpHomogeneousMatrix  cMo;
  BackgroundRemoval * bg_remove;

  PoseEstimation(CloudPtr cloud, double distanceThreshold = 0.05){

    cloud_ = cloud;
    camera_frame_name = cloud->header.frame_id;
    bg_remove = new BackgroundRemoval(cloud, distanceThreshold);
    debug_ = false;
    // @TODO Actual camera frame...
    vispToTF.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "/stereo", "/cMo", "cMo");
    symmetry_search_ = false;
    planar_symmetry_ = true;
  }

  virtual bool initialize(){ return false;}
  virtual bool process(){ return false;}

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

/** Pose Estimation using PCA */
class PCAPoseEstimation : public PoseEstimation{

  int cluster_index_, cluster_thereshold_;
  CloudClustering<PointT> * cloud_clustering_;

public:

  PCAPoseEstimation(CloudPtr source, int cluster_thereshold = 400) : PoseEstimation(source){
    cluster_thereshold_ = cluster_thereshold;
  }

  bool initialize(){ return process(); }

  // Use process multiple times with new clouds.
  bool process();

  //Use process next to process other cluster without repeating clustering.
  bool processNext();

};

/** Box Pose Estimation using RANSAC Plane Extraction*/
class BoxPoseEstimation : public PoseEstimation{

public:

  BoxPoseEstimation(CloudPtr source) : PoseEstimation(source){}

  bool initialize(){ return process(); }
  bool process();

};

/** Pose Estimation using SQ */
class SQPoseEstimation : public PoseEstimation{

  int cluster_index_, cluster_thereshold_;
  CloudClustering<PointT> * cloud_clustering_;

  CloudPtr sq_cloud_;

  double sq_params_[5];
  Eigen::Matrix4d sq_transform_;

  bool use_ceres_, use_region_growing_;
  double region_growing_norm_th_, region_growing_curv_th_;
  double euclidian_tolerance_;

public:

  SQPoseEstimation(CloudPtr source, int cluster_thereshold = 400, double euclidian_tolerance = 0.02 ) : PoseEstimation(source),
                                                                                                        cluster_thereshold_(cluster_thereshold),
                                                                                                        euclidian_tolerance_(euclidian_tolerance){
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

  void display();

};

/** Pose Estimation using RANSAC Cylinder extraction */
class CylinderPoseEstimation : public PoseEstimation{

  //Punto central del cilindro y la direccion.
  PointT axis_point_g;
  tf::Vector3 normal_g;

  pcl::ModelCoefficients::Ptr coefficients_cylinder;
  double cylinder_distance_threshold_, radious_limit_;
  int cylinder_iterations_;

public:

  double radious, height;

  /** Constructor.
   * @params: Get pose using RANSAC & the input cloud.
   * */
  CylinderPoseEstimation(CloudPtr cloud, double distanceThreshold = 0.05) : PoseEstimation(cloud, distanceThreshold){//, ros::NodeHandle & nh, std::string object_pose, vpHomogeneousMatrix wMc ){
    setCylinderSegmentationParams();
  }

  /** Where segmentation is done */
  bool initialize();

  /** Online planning **/
  bool process();

  /** Set cylinder segmentation parameters: distance to the inliers to the plane,
   * number of iterations and radious limit.
   */
  void setCylinderSegmentationParams(double distanceThreshold = 0.05,int iterations = 20000, double rlimit = 0.1){
    cylinder_distance_threshold_=distanceThreshold;
    cylinder_iterations_=iterations;
    radious_limit_=rlimit;
  }

  ~CylinderPoseEstimation() {}

private:

  /** Comparer used in the sort function */
  bool sortFunction(const PointT& d1, const PointT& d2);

  /** Get the grasp frame with respect to the camera frame */
  void getMinMax3DAlongAxis(const pcl::PointCloud<PointT>::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT axis_point, tf::Vector3 * normal, double outlier_percentage = 0.1);

};//End CylinderPoseEstimation


/** Pose Estimation using RANSAC Sphere extraction */
class SpherePoseEstimation : public PoseEstimation{

  //Punto central de la esfera y la direccion.
  PointT axis_point_g;
  tf::Vector3 normal_g;

  pcl::ModelCoefficients::Ptr coefficients_sphere;
  double sphere_distance_threshold_, radious_limit_;
  int sphere_iterations_;

public:

  double radious, height;

  /** Constructor.
   * @params: Get pose using RANSAC & the input cloud.
   * */
  SpherePoseEstimation(CloudPtr cloud, double distanceThreshold = 0.05) : PoseEstimation(cloud, distanceThreshold){//, ros::NodeHandle & nh, std::string object_pose, vpHomogeneousMatrix wMc ){
    setSphereSegmentationParams();
  }

  /** Where segmentation is done */
  bool initialize();

  /** Online planning **/
  bool process();

  /** Set sphere segmentation parameters: distance to the inliers to the plane,
   * number of iterations and radious limit.
   */
  void setSphereSegmentationParams(double distanceThreshold = 0.05,int iterations = 20000, double rlimit = 0.08){
    sphere_distance_threshold_=distanceThreshold;
    sphere_iterations_=iterations;
    radious_limit_=rlimit;
  }
  ~SpherePoseEstimation() {}

};//End SpherePoseEstimation


#endif // SAC_POSE_ESTIMATION_H
