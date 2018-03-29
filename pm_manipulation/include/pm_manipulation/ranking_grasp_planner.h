/*
 * Grasp planning based on Pose Estimation in pm_perception
 *
 * This class (or classes) obtain from the object pose a list of possible grasps and ranks them depending on
 * different parameters. See pcl_manipulation. Kinematic constraints, similarity with the target pose, angle with floor,
 * among other options.
 *
 *  Created on: 05/02/2018
 *      Author: dfornas
 */

#ifndef RANKINGGRASPPLANNING_H_
#define RANKINGGRASPPLANNING_H_

#include <pm_tools/tf_tools.h>
#include <pm_perception/pose_estimation.h>

#include <mar_robot_arm5e/ARM5Arm.h>

struct GraspHypothesis{
  vpHomogeneousMatrix cMg, cMg_ik;
  double distance_score, distance_ik_score, angle_ik_score, angle_axis_score, overall_score;
};

bool sortByScore(GraspHypothesis a, GraspHypothesis b){
  return a.overall_score < b.overall_score;
}

double angle(vpColVector a, vpColVector b){
  return acos(vpColVector::dotProd(a, b) / (a.euclideanNorm() * b.euclideanNorm()));
}

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

/** Grasp planning from object pose, CylinderPoseEstimation version */
class RankingGraspPlanner {

  // Visp to TF for Visualization, @TODO add Eigen to Visp to TF...
  FrameToTF vispToTF;
  ros::Publisher grasp_pub;

public:

  boost::shared_ptr<PoseEstimation> pose_estimation;

  vpHomogeneousMatrix cMo, bMc; // Camera to object, camera to kinematic base
  ARM5Arm robot;
  std::list<vpHomogeneousMatrix> grasp_list; // Grasp positions list, cMg list

  //Geometric parameters of the object, this depends on the Pose Estimator, atm is Cylinder RANSAC
  double radious, height;

  /** Constructor, @TODO Add METHOD Enum  * */
  RankingGraspPlanner(CloudPtr cloud, ros::NodeHandle & nh){
    // @TODO SWITCH METHOD.
    pose_estimation = boost::shared_ptr<CylinderPoseEstimation>( new CylinderPoseEstimation(cloud) );
  }

  /** Cloud set */
  void setNewCloud(CloudPtr cloud){
    pose_estimation->setNewCloud(cloud);
  }

  /** Start pose estimation, generate a grasp list and rank it. */
  void generateGraspList();
  GraspHypothesis generateGraspHypothesis( vpHomogeneousMatrix cMg );
  void filterGraspList();

  /** Get best rasp based on ranking */
  vpHomogeneousMatrix getBestGrasp();

  /** Publish one object pose. Mainly to display in UWSim **/
  void publishObjectPose(){
    //pos_pub.publish( VispTools::geometryPoseFromVispHomog(cMo) );
    //ros::spinOnce();
  }

  /** Publish grasp pose. **/
  void publishBestGrasp(){}

  /** Publish grasp list sequentially. **/
  void publishGraspList(){}

  //  @TODO Get camera to base transform from TF.
  void getbMc();
  vpHomogeneousMatrix tfToVisp(tf::StampedTransform matrix_tf);


  ~RankingGraspPlanner() {}

};




#endif /* RANKINGGRASPPLANNING_H_ */
