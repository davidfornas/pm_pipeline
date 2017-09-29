/*
 * PCGraspPlanning.h Complete point cloud grasp planning from
 * MAR (https://github.com/penalvea/irs-ros-pkg). It has been
 * divided in modular components but this is the full version.
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef PMGRASPPLANNINGSPLIT_H_
#define PMGRASPPLANNINGSPLIT_H_

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



/** Grasp planning from object pose */
class PMGraspPlanningSplit {

  //Grasping params (to allow different grasps and radious (for grasp penetration)).
  double angle_, rad_, along_;

  FrameToTF vispToTF;

  std::string camera_frame_name, topic_name;
  bool do_ransac;
  ros::Publisher pos_pub;

public:


  boost::shared_ptr<SACPoseEstimation> sac_pose_estimation;

  vpHomogeneousMatrix cMg, cMo; ///< Grasp frame with respect to the camera after planning
  double radious, height;

  //With integuers to use trackbars
  int iangle, irad, ialong;
  bool change_z_;

  /** Constructor.
   * @params: Get pose from topic
   * */
  PMGraspPlanningSplit( std::string object_topic_name, std::string frame_id = "world", bool change_z = false){
    angle_=0;iangle=0;
    rad_=0;irad=0;
    along_=0;ialong=0;

    camera_frame_name = frame_id; //cloud->header.frame_id;
    topic_name = object_topic_name;
    do_ransac = false;
    change_z_ = change_z;

  }


  /** Constructor.
   * @params: Get pose using RANSAC & the input cloud.
   * */
  PMGraspPlanningSplit(PointTPtr cloud, ros::NodeHandle & nh, std::string object_pose ){

    angle_=0;iangle=0;
    rad_=0;irad=0;
    along_=0;ialong=0;

    camera_frame_name = cloud->header.frame_id;
    pos_pub = nh.advertise<geometry_msgs::Pose>( object_pose, 1); //"/gripperPose"
    do_ransac = true;

    sac_pose_estimation = boost::shared_ptr<SACPoseEstimation>( new SACPoseEstimation(cloud) );

  }

  void setNewCloud(PointTPtr cloud){
    sac_pose_estimation->setNewCloud(cloud);
  }

  /** Main function */
  void perceive();

  /** Online planning **/
  void redoRansac();

  void setCamerFrameName( std::string name){ camera_frame_name = name; }

  /** Get the grasp frame with respect to the camera frame */
  vpHomogeneousMatrix get_cMg() {return cMg;}

  void getBestParams( double & angle, double & rad, double & along );
  void computeMatrix( double angle, double rad, double along );

  /** Recalculate cMg with current parameters */
  void recalculate_cMg();

  /** Get the grasp frame pose with respect to an arbitrary frame 'b', given as input relative to the camera frame
   * @param bMc an homogeneous matrix with the camera frame given wrt the frame 'b'
   * @returns an homogeneous matrix with the grasp frame given wrt the frame 'b'
   */
  vpHomogeneousMatrix get_bMg(vpHomogeneousMatrix bMc) {return bMc*cMg;}

  /** Publish object pose for RANSAC. Mainly to display in UWSim **/
  void publishObjectPose(){
    pos_pub.publish( VispTools::geometryPoseFromVispHomog(cMo) );
    ros::spinOnce();
  }

  ~PMGraspPlanningSplit() {}

private:

  /** Configure the camera based in int slider parameters */
  void intToConfig();

};


#endif /* PMGRASPPLANNINGSPLIT_H_ */
