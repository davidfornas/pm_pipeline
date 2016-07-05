/** 
 * Manip
 *
 *  Created on: 233/06/2016
 *      Author: dfornas
 */

#include <pm_manipulation/pm_grasp_planning.h>
#include <pm_manipulation/hypothesis_generation.h>
#include <pm_manipulation/grasp_hypothesis_evaluation.h>

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "grasp_planning_example");
  ros::NodeHandle nh;

  //Class usages examples. TOp down design.
  CloudPtr scene(new Cloud);
  PCLTools<PointType>::cloudFromPCD(scene, std::string(argv[1]) + std::string(".pcd"));

  HypothesisGeneration<PointType> hyp_gen(scene, true);
  hyp_gen.getGraspCandidate();

  /*PMGraspPlanning planner(scene, background_remover, segmentator, hypothesis_generation);

  planner.perceive();
  planner.get_cMg();
  // @ TODO planner.proccessScene();

  GraspHypothesisEvaluation ghyval; // @ TODO  from planner.getGraspHypothesis;
  ghyval.getBestGrasp();
  // @ TODO ghyval.getSortedGraspList();
  // @ TODO SHOW RESULT...
  */

  return 0;
}

