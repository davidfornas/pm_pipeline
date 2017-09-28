/** 
 * This program test the classes available in the Perception&Manipulation pipeline.
 * At the moment it is no really testing all of them beacuse the implementation is
 * not finished.
 *
 * Without visualization, only usage is tested.
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>

#include <pm_perception/background_removal.h>
#include <pm_perception/object_segmentation.h>

#include <pm_manipulation/pm_grasp_planning.h>

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "grasp_planning_example");
  ros::NodeHandle nh;

  //Class usages examples. TOp down design.
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>), scene_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::string input_basename("in");
  nh.getParam("input_basename", input_basename);
  PCLTools<pcl::PointXYZ>::cloudFromPCD(scene, input_basename + std::string(".pcd")); //Load from PCDReader or from topic
  //From a topic instead: PCLTools::cloudFromTopic(scene, "/stereo/points2");

  //Optional filtering: PCLTools::applyZAxisPassthrough(scene, scene2, 0, 3); ///Input and output can not be the same.

//  BackgroundRemoval * background_remover;
//  ObjectSegmentation * segmentator;
//  HypothesisGeneration * hypothesis_generation; //NEWS!!!!!!!
//
//  PMGraspPlanning planner(scene, background_remover, segmentator, hypothesis_generation);

    PMGraspPlanning planner(scene);
    planner.perceive();
    planner.get_cMg();

    //SHOW RESULT HOW()

    // @ TODO planner.proccessScene();

  //GraspHypothesisEvaluation ghyval; // @ TODO  from planner.getGraspHypothesis;
  //ghyval.getBestGrasp();
  // @ TODO ghyval.getSortedGraspList();
  // @ TODO SHOW RESULT...

  return 0;
}

