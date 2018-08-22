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

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>

#include <mar_robot_arm5e/ARM5Arm.h>

#include <pm_tools/tf_tools.h>
#include <pm_perception/ransac_pose_estimation.h>
#include <pm_perception/pca_pose_estimation.h>
#include <pm_perception/sq_pose_estimation.h>

//GraspHypothesis Desired cMg and cMg_ik the reachable pose
class GraspHypothesis{

public:

  vpHomogeneousMatrix cMg, cMg_ik, oMg, cMo;
  double preshapes[2];
  double measures[6];
  double distance_score, distance_ik_score, angle_ik_score, angle_axis_score, overall_score;

  GraspHypothesis(){}

  GraspHypothesis(vpHomogeneousMatrix cmg, vpHomogeneousMatrix cmgik, double d, double dik, double aik, double aa, double overall ):
          cMg(cmg), cMg_ik(cmgik), distance_score(d), distance_ik_score(dik), angle_ik_score(aik),
          angle_axis_score(aa), overall_score(overall){}

  void print(){
    ROS_INFO_STREAM(cMg);
    ROS_INFO_STREAM(cMg_ik);
    ROS_INFO_STREAM(oMg);
  }
  ~GraspHypothesis(){}
};

// Sort function
bool sortByScore( GraspHypothesis a, GraspHypothesis b ){
  return a.overall_score < b.overall_score;
}

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

/** Grasp planning from object pose */
class RankingGraspPlanner {

protected:

  ros::NodeHandle nh_;
  ros::Publisher grasp_pub_, pos_pub_;
  ros::Publisher score_pub_, score_description_pub_, metrics_pub_, list_size_pub_;

  FrameToTF vispToTF_;
  bool debug_;

public:

  boost::shared_ptr<PoseEstimation> pose_estimation;

  // Camera to object, camera to kinematic base
  vpHomogeneousMatrix cMo, bMc;
  ARM5Arm robot;
  std::list<GraspHypothesis> grasps;

  /** Constructor **/
  RankingGraspPlanner(CloudPtr cloud, ros::NodeHandle & nh, std::string object_pose, bool debug = false) : nh_(nh), debug_(debug) {
    vispToTF_.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "/stereo", "/cMg", "cMg");
    vispToTF_.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "/stereo", "/cMg_ik", "cMg_ik");
    vispToTF_.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "/stereo", "/object", "cMo");
    vispToTF_.addTransform(vpHomogeneousMatrix(0, 0, 0, 0, 0, 0), "/object", "/grasp", "oMg");
    pos_pub_ = nh.advertise<geometry_msgs::Pose>( object_pose, 1);
    score_pub_ = nh.advertise<std_msgs::Float32>( "/score", 1);
    list_size_pub_ = nh.advertise<std_msgs::Int32>( "/list_size", 1);
    score_description_pub_ = nh.advertise<std_msgs::String>( "/score_description", 1);
    metrics_pub_ = nh.advertise<std_msgs::String>( "/metrics", 1);
  }

  /** Cloud set */
  void setNewCloud(CloudPtr cloud){
    pose_estimation->setNewCloud(cloud);
  }

  /** Start pose estimation, generate a grasp list and rank it. */
  bool generateGraspList();

  GraspHypothesis generateGraspHypothesis( vpHomogeneousMatrix cMg );

  void filterGraspList();

  /** Get best rasp based on ranking. Bigger score is worse. */
  vpHomogeneousMatrix getBestGrasp(){
    ROS_INFO_STREAM("Best grasp "  << 0 << grasps.front().overall_score);
    ROS_INFO_STREAM("Worst grasp " << grasps.size() << grasps.back().overall_score);
    return grasps.front().cMg;
  }

  /** @TODO Publish one object pose. Mainly to display in UWSim **/
  void publishObjectPose(){
    vispToTF_.resetTransform( cMo, "cMo");
    pos_pub_.publish( VispTools::geometryPoseFromVispHomog(cMo) );
    ros::spinOnce();
  }

  /** @TODO Publish grasp pose. Once sorted publish first grasp. **/
  void publishBestGrasp(){}

  /** Publish grasp list sequentially in TF. **/
  void publishGraspList();

  /** Publish grasp scores and metrics */
  void publishGraspData( int grasp_id );

  vpHomogeneousMatrix getGrasp_cMg( int grasp_id );

  vpHomogeneousMatrix getGrasp_cMg_ik( int grasp_id );

  // Get camera to base transform from TF.
  void getbMc();

  // Helper function
  vpHomogeneousMatrix tfToVisp(tf::StampedTransform matrix_tf);

  ~RankingGraspPlanner() {}

};

class CylinderRankingGraspPlanner : public RankingGraspPlanner {

public:

  /**   * */
  CylinderRankingGraspPlanner(CloudPtr cloud, ros::NodeHandle & nh, std::string object_pose, bool debug = false) : RankingGraspPlanner(cloud, nh, object_pose, debug){
    pose_estimation = boost::shared_ptr<CylinderPoseEstimation>( new CylinderPoseEstimation(nh, cloud) );
    pose_estimation->setDebug(debug);
  }

  ~CylinderRankingGraspPlanner() {
    grasps.clear();
    pose_estimation.reset();
  }

};

class SQRankingGraspPlanner : public RankingGraspPlanner {

  ros::Publisher params_pub;
  ros::Subscriber grasps_sub;

  int grasps_read;

  int num_grasps;
  double anglerange, deltaspace;
  double roll_arange_init, roll_arange_end, roll_arange_step;
  std::vector<double> standoffs;

public:

  /** Constructor  * */
  SQRankingGraspPlanner(CloudPtr cloud, ros::NodeHandle & nh, std::string object_pose, bool debug = false, int num_grasps = 5) : RankingGraspPlanner(cloud, nh, object_pose, debug){
    // @TODO SWITCH METHOD.
    pose_estimation = boost::shared_ptr<SQPoseEstimation>( new SQPoseEstimation(nh, cloud, 400, 0.01) );
    pose_estimation->setRegionGrowingClustering(8.0, 8.0);
    //pose_estimation->setLMFitting();
    //pose_est->setSymmetrySearchParams(0.0);
    //pose_est->setSymmetrySearchParams(0.40, 0.05, 0.2);
    //pose_est->setAxisSymmetryMode();
    pose_estimation->setDebug(debug_);
    params_pub = nh.advertise<std_msgs::Float32MultiArray>("/generate_grasp_parameters", 10);
    grasps_sub = nh.subscribe("/grasp_result", 20, &SQRankingGraspPlanner::graspCallback, this);
    setGraspsParams(num_grasps);
  }

  bool generateGraspList();

  void graspCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

  void generateGraspScores( GraspHypothesis & grasp );

  void setGraspsParams( int n = 20, double arange = 0.4, double dspace = 0.04,
                        double roll1 = 0, double roll2 = 3.1416 * 2, double roll3 = 3.1416 / 2 ){
    num_grasps = n;
    anglerange = arange;
    deltaspace = dspace;
    roll_arange_init = roll1;
    roll_arange_end = roll2;
    roll_arange_step = roll3;
    standoffs.push_back(0.00);
    standoffs.push_back(0.02);
    standoffs.push_back(0.04);
    standoffs.push_back(0.05);
    standoffs.push_back(0.09);
    standoffs.push_back(0.12);
    standoffs.push_back(0.14);
  }

  /** Publish grasp list sequentially in TF. **/
  void publishGraspList( double wait_time = 0.5);

  /** Get best rasp based on ranking. Bigger score is worse. */
  vpHomogeneousMatrix getBestGrasp();

  /** Filter grasp list using different constraints. */
  void filterGraspList();

  ~SQRankingGraspPlanner() {}

};

#endif /* RANKINGGRASPPLANNING_H_ */
