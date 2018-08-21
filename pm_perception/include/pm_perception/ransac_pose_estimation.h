/*
 * Pose Estimation classes
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */
#ifndef RANSAC_POSE_ESTIMATION_H
#define RANSAC_POSE_ESTIMATION_H

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

/** Box Pose Estimation using RANSAC Plane Extraction*/
class BoxPoseEstimation : public PoseEstimation{

  float width_;
  float height_;
  float depth_;

public:

  BoxPoseEstimation(ros::NodeHandle & nh, CloudPtr source) : PoseEstimation(nh, source){}

  bool initialize(){ return process(); }
  bool process();
  void publishResults();

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
  CylinderPoseEstimation(ros::NodeHandle & nh, CloudPtr cloud, double distanceThreshold = 0.05)
          : PoseEstimation(nh, cloud, distanceThreshold){//, ros::NodeHandle & nh, std::string object_pose, vpHomogeneousMatrix wMc ){
    setCylinderSegmentationParams();
  }

  /** Where segmentation is done */
  bool initialize();

  /** Online planning **/
  bool process();

  /** Set cylinder segmentation parameters: distance to the inliers to the plane,
   * number of iterations and radious limit.
   */
  void setCylinderSegmentationParams(double distanceThreshold = 0.05,int iterations = 20000, double rlimit = 0.15){
    cylinder_distance_threshold_=distanceThreshold;
    cylinder_iterations_=iterations;
    radious_limit_=rlimit;
  }

  void publishResults();

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
  SpherePoseEstimation(ros::NodeHandle & nh, CloudPtr cloud, double planeDistanceThreshold = 0.05, double sphereDistanceThreshold = 0.05, int iterations = 20000, double rlimit = 0.08)
          : PoseEstimation(nh, cloud, planeDistanceThreshold){//, ros::NodeHandle & nh, std::string object_pose, vpHomogeneousMatrix wMc ){
    setSphereSegmentationParams(sphereDistanceThreshold, iterations, rlimit);
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

  void publishResults();

  ~SpherePoseEstimation() {}

};//End SpherePoseEstimation

#endif
