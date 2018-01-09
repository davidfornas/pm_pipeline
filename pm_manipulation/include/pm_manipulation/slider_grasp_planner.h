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
#include <pm_perception/sac_pose_estimation.h>

#include <pcl/io/pcd_io.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpPoint.h>
#include <visp/vpHomogeneousMatrix.h>

#include <tf/transform_datatypes.h>
#include <list>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PointTPtr;



/** Grasp planning from object pose */
class SliderGraspPlanner {

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
  SliderGraspPlanner( std::string object_topic_name, std::string frame_id = "world", bool change_z = false){
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
  SliderGraspPlanner(PointTPtr cloud, ros::NodeHandle & nh, std::string object_pose ){

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

  ~SliderGraspPlanner() {}

private:

  /** Configure the camera based in int slider parameters */
  void intToConfig();

};


#endif /* PMGRASPPLANNINGSPLIT_H_ */
