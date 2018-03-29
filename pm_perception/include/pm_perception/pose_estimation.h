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
#include <pm_perception/background_removal.h>

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

  CloudPtr cloud_;
  std::string camera_frame_name, topic_name;
  FrameToTF vispToTF;
  bool debug_;

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
  }

  virtual void initialize(){}
  virtual void process(){}

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
};

/** Pose Estimation using PCA */
class PCAPoseEstimation : public PoseEstimation{

public:

  PCAPoseEstimation(CloudPtr source) : PoseEstimation(source){}

  void initialize(){process();}
  void process();

};

/** Box Pose Estimation using RANSAC Plane Extraction*/
class BoxPoseEstimation : public PoseEstimation{

public:

  BoxPoseEstimation(CloudPtr source) : PoseEstimation(source){}

  void initialize(){process();}
  void process();

};

/** Pose Estimation using SQ */
class SQPoseEstimation : public PoseEstimation{

public:

  SQPoseEstimation(CloudPtr source) : PoseEstimation(source){}

  void initialize(){}
  void process(){}

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
  void initialize();

  /** Online planning **/
  void process();

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
  void initialize();

  /** Online planning **/
  void process();

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
