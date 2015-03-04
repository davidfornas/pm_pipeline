/** 
 * ..
 *  Created on: 3/03/2014
 *      Author: dfornas
 */

#include <pm_pcl_tools/pcl_tools.h>

#include <pm_perception/background_removal.h>
#include <pm_perception/object_segmentation.h>

#include <pm_manipulation/pm_grasp_planning.h>
#include <pm_manipulation/hypothesis_generation.h>
#include <pm_manipulation/grasp_hypothesis_evaluation.h>

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "grasp_planning_example");
  ros::NodeHandle nh;

  //Ejemplo de utilizacion de la clase, diseño top-down.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene;
  pm_pcl_tools::PCLoader(scene, "cloud", true);//Load from PCDReader or from topic
  pm_pcl_tools::PassthroughFilter(scene, 0, 0, 1);

  BackgroundRemoval background_remover;
  ObjectSegmentation segmentator;
  HypothesisGeneration hypothesis_generation;
  PMGraspPlanning planner(scene, background_remover, segmentator, hypothesis_generation);

  planner.proccessScene();

  GraspHypothesisEvaluation ghyval;//();//planner.getGraspHypothesis);
  ghyval.getBestGrasp();
  //ghyval.getSortedGraspList();
  //SHOW RESULT...
  return 0;
}
