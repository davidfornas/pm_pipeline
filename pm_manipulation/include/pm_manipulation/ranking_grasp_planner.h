/*
 * Grasp planning based on Pose Estimation in pm_perception
 *
 *  Created on: 05/02/2018
 *      Author: dfornas
 */

#ifndef RANKINGGRASPPLANNING_H_
#define RANKINGGRASPPLANNING_H_

#include <pm_tools/tf_tools.h>
#include <pm_perception/pose_estimation.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

/** Grasp planning from object pose, CylinderPoseEstimation version */
class RankingGraspPlanner {

  // Visp to TF for Visualization, @TODO add Eigen to Visp to TF...
  FrameToTF vispToTF;
  ros::Publisher grasp_pub;

  //Start having the cloud stored in the pose estimator.
  //CloudPtr cloud_;

public:

//  boost::shared_ptr<CylinderPoseEstimation> pose_estimation;
  boost::shared_ptr<CylinderPoseEstimation> pose_estimation;

  vpHomogeneousMatrix cMg, cMo; // Camera to grasp, camera to object
  std::vector<vpHomogeneousMatrix> grasp_list; // Grasp positions list

  //Geometric parameters of the object
  double radious, height;

  /** Constructor  * */
  RankingGraspPlanner(CloudPtr cloud, ros::NodeHandle & nh, std::string object_pose ){
    //cloud_ = cloud;
    pose_estimation = boost::shared_ptr<CylinderPoseEstimation>( new CylinderPoseEstimation(cloud) );
  }

  /** Cloud set */
  void setNewCloud(CloudPtr cloud){
    pose_estimation->setNewCloud(cloud);
  }

  /** Start pose estimation, generate a grasp list and rank it. */
  void generateGraspList();

  /** Get best rasp based on ranking */
  vpHomogeneousMatrix getBestGrasp();

  /** Publish object pose. Mainly to display in UWSim **/
  void publishObjectPose(){
    //pos_pub.publish( VispTools::geometryPoseFromVispHomog(cMo) );
    //ros::spinOnce();
  }

  /** Publish grasp pose. **/
  void publishBestGrasp(){}

  /** Publish grasp list sequentially. **/
  void publishGraspList(){}

  ~RankingGraspPlanner() {}


};


#endif /* RANKINGGRASPPLANNING_H_ */
