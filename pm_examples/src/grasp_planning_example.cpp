/** 
 * This program test the classes available in the Perception&Manipulation pipeline
 *  Created on: 03/03/2014
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>

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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>), scene_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string input_basename("in");
  nh.getParam("input_basename", input_basename);
  PCLTools::cloudFromPCD(scene, input_basename + std::string(".pcd"));//Load from PCDReader or from topic
  //PCLTools::cloudFromTopic(scene, "/stereo/points2");
  //PCLTools::applyZAxisPassthrough(scene, scene, 0, 3);

  BackgroundRemoval * background_remover;
  ObjectSegmentation * segmentator;
  HypothesisGeneration * hypothesis_generation;//NEWS!!!!!!!

  PMGraspPlanning planner(scene, background_remover, segmentator, hypothesis_generation);

  planner.perceive();
  planner.get_cMg();
  //planner.proccessScene();

  GraspHypothesisEvaluation ghyval;//();//planner.getGraspHypothesis);
  ghyval.getBestGrasp();
  //ghyval.getSortedGraspList();
  //SHOW RESULT...
  return 0;
}

