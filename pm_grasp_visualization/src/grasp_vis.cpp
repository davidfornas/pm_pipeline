/**
 * 3D interface to plan a grasp using a point cloud.
 *
 *  Created on: 24/01/2016
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <mar_perception/PCAutonomousGraspPlanning.h>
#include <pm_tools/pcl_tools.h>

//Quick GUI form OpoenCV
#include <vector>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Pose.h>



typedef pcl::PointXYZRGB PointT;
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
  //Listen for rechable frame
  //tf::TransformListener listener;

  ros::Publisher position_pub;
  position_pub=nh.advertise<geometry_msgs::Pose>("/gripperPose",1);


  //Variables de configuración: ángulo de agarre, distancias...
  double angle, rad, along;
  bool alignedGrasp;
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
  PCAutonomousGraspPlanning planner(angle, rad, along, alignedGrasp, cloud);
  planner.perceive();

  std::cout << "Starting visualization in UWSim" << std::endl; //Not anymore
  //PONER EL GRIPPER EN LA POSICIÓN INICIAL  cMg
  vpHomogeneousMatrix cMg = planner.get_cMg();

  while (ros::ok())/// && !view.getViewer()->done())
  {
    //Interface
    cv::namedWindow("Grasp configuration", CV_WINDOW_NORMAL);
    cv::createTrackbar("Radius", "Grasp configuration", &(planner.irad), 100);
    cv::createTrackbar("Angle", "Grasp configuration", &(planner.iangle), 360);
    cv::createTrackbar("Distance", "Grasp configuration", &(planner.ialong), 100);
    cv::createTrackbar("Aligned grasp?", "Grasp configuration", &(planner.ialigned_grasp), 1);
    //cv::createTrackbar("A. Distance", "Grasp configuration", &distance, 20);
    //Compute adn display new grasp frame
    planner.recalculate_cMg();
    cMg = planner.get_cMg();
    //SHOW DESIRED
    geometry_msgs::Pose p;
    p.position.x=cMg[0][3];
    p.position.y=cMg[1][3];
    p.position.z=cMg[2][3];
    vpQuaternionVector q; cMg.extract(q);
    p.orientation.x=q.x();
    p.orientation.y=q.y();
    p.orientation.z=q.z();
    p.orientation.w=q.w();
    position_pub.publish(p);
    ros::spinOnce();

    cv::waitKey(5);
  }
  return 0;
}
