/*
 * xxx
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */

#ifndef SAC_POSE_ESTIMATION_H
#define SAC_POSE_ESTIMATION_H


#include <pm_tools/tf_tools.h>
#include <pm_tools/marker_tools.h>
#include <pm_tools/pcl_segmentation.h>

#include <pcl/io/pcd_io.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpPoint.h>
#include <visp/vpHomogeneousMatrix.h>

#include <tf/transform_datatypes.h>
#include <list>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PointTPtr;

class SACPoseEstimation{

  PointTPtr cloud_;
  std::string camera_frame_name, topic_name;

  //Punto central del cilindro y la direccion.
  PointT axis_point_g;
  tf::Vector3 normal_g;
  FrameToTF vispToTF;

  pcl::ModelCoefficients::Ptr coefficients_plane, coefficients_cylinder;
  double plane_distance_threshold_, cylinder_distance_threshold_, radious_limit_;
  int plane_iterations_, cylinder_iterations_;

public:

  vpHomogeneousMatrix  cMo; ///< object frame with respect to the camera after detection
  double radious, height;

  /** Constructor.
   * @params: Get pose using RANSAC & the input cloud.
   * */
  SACPoseEstimation(PointTPtr cloud){//, ros::NodeHandle & nh, std::string object_pose, vpHomogeneousMatrix wMc ){
    setPlaneSegmentationParams();
    setCylinderSegmentationParams();

    cloud_ = cloud;
    camera_frame_name = cloud->header.frame_id;
    coefficients_plane = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;
  }

  void setNewCloud(PointTPtr cloud){cloud_ = cloud;}

  /** Where segmentation is done */
  void doRansac();

  /** Online planning **/
  void redoRansac();

  /** Set plane segmentation parameters: distance to the inliers to the plane
   * and number of iterations.
   */
  void setPlaneSegmentationParams(double distanceThreshold = 0.03, int iterations = 100){
    plane_distance_threshold_=distanceThreshold;
    plane_iterations_=iterations;
  }

  /** Set cylinder segmentation parameters: distance to the inliers to the plane,
   * number of iterations and radious limit.
   */
  void setCylinderSegmentationParams(double distanceThreshold = 0.05,int iterations = 20000, double rlimit = 0.1){
    cylinder_distance_threshold_=distanceThreshold;
    cylinder_iterations_=iterations;
    radious_limit_=rlimit;
  }

  void setCamerFrameName( std::string name){
    camera_frame_name = name;
  }

  /** Get the object frame with respect to the camera frame */
  vpHomogeneousMatrix get_cMo() {return cMo;}

  ~SACPoseEstimation() {}

private:

  /** Comparer used in the sort function */
  bool sortFunction(const PointT& d1, const PointT& d2);

  /** Get the grasp frame with respect to the camera frame */
  void getMinMax3DAlongAxis(const pcl::PointCloud<PointT>::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT axis_point, tf::Vector3 * normal, double outlier_percentage = 0.1);

};//End SACPoseEstimation

#endif // SAC_POSE_ESTIMATION_H
