/**
 * 3D interface to plan a grasp using a point cloud. The interface used is HighGUI and visualization
 * is done in a separate UWSim window.
 *
 *  Created on: 24/01/2016
 *      Author: dfornas
 */
#include <ros/ros.h>
#include "../../../pm_test_code/include/pm_test_code/pm_grasp_planning.h"
#include <pm_tools/pcl_tools.h>
#include <pm_tools/marker_tools.h>


//Quick GUI form OpoenCV, should change it to Qt.
#include <vector>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;



/** Plans a grasp on a point cloud and visualizes it using UWSim externally.
 * Expects the following params to be set in the ROS parameter server: @TODO CHANGE
 * input_basename: The full path to the input files without PCD extension, ie. /tmp/scan_2
 * resources_data_path: The path the folder with the models for UWSim
 * eMh: a six-element vector representing the hand frame wrt the end-effector frame. [x y z roll pitch yaw] format; now UNUSED
 */
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "grasp_vis");
  ROS_INFO("Hello world!");

  ros::NodeHandle nh;

  EefFollower follower("/gripper_pose", nh);//(std::string)argv[1]);

  //Variables de configuración: ángulo de agarre, distancias...
  double angle, rad, along;
  bool alignedGrasp;
  int ialigned_grasp = 1;
  std::string input_basename("output");
  nh.getParam("input_basename", input_basename);
  nh.param("alignedGrasp", alignedGrasp, false);
  nh.param("angle", angle, 0.0);
  nh.param("rad", rad, 0.0);
  nh.param("along", along, 0.0);

  //Point Cloud load
  Cloud::Ptr cloud (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromPCD(cloud, input_basename); //Load from PCDReader or from topic
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

  //Init planner
  PMGraspPlanning planner(cloud);
  planner.perceive();

  std::cout << "Starting visualization in UWSim" << std::endl; //Not anymore
  //PONER EL GRIPPER EN LA POSICIÓN INICIAL  cMg
  vpHomogeneousMatrix cMg = planner.get_cMg();

  while (ros::ok())
  {
    //Interface
    cv::namedWindow("Grasp configuration", CV_WINDOW_NORMAL);
    cv::createTrackbar("Radius", "Grasp configuration", &(planner.irad), 100);
    cv::createTrackbar("Angle", "Grasp configuration", &(planner.iangle), 360);
    cv::createTrackbar("Distance", "Grasp configuration", &(planner.ialong), 100);
    cv::createTrackbar("Aligned grasp?", "Grasp configuration", &ialigned_grasp, 1);

    //Compute new grasp frame with the slides
    planner.recalculate_cMg();
    cMg = planner.get_cMg();

    ialigned_grasp==1 ? follower.setMarkerStatus(false) : follower.setMarkerStatus(true) ;
    follower.loop(cMg);

    ros::spinOnce();
    cv::waitKey(5);
  }
  return 0;
}
