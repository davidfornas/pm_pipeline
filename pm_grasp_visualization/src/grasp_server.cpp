/**
 * Grasp server for MERBOTS GUI.
 *
 *  Created on: 8/02/2017
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_manipulation/pm_grasp_planning.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/marker_tools.h>


#include <std_msgs/String.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;


bool markerStatus, compute_initial_cMg, resetMarker;

void stringCallback(const std_msgs::String &msg ){
  if( msg.data == "init"){
    compute_initial_cMg = true;
  }else if( msg.data != "markerReset"){
    markerStatus = (msg.data == "guided" ?  false : true );
  }else{
    resetMarker = true;
  }
}

/** Plans a grasp on a point cloud and visualizes it using UWSim externally.
 *  Subscribes to GUI Commands...
 */
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "grasp_server");
  ros::NodeHandle nh;

  compute_initial_cMg = false;
  resetMarker = false;

  //SETUP GUI SUBSCRIBER for specification_status
  ros::Subscriber status_sub = nh.subscribe("/specification_status", 1, stringCallback);

  EefFollower follower("/gripperPose", nh);


  //Variables de configuración: ángulo de agarre, distancias...
  //GET FROM GUI
  double angle = 0, rad = 0, along = 0;
  bool alignedGrasp = true;
  std::string input_basename("output");
  nh.getParam("input_basename", input_basename);

  //Point Cloud load
  Cloud::Ptr cloud (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromPCD(cloud, input_basename);
  ROS_DEBUG_STREAM("PointCloud has: " << cloud->points.size() << " data points.");

  //WAIT FOR INIT MESSAGE
//CHANGE THIS
  while (!compute_initial_cMg)
  {
    ros::spinOnce();
  }

  //Init planner
  PMGraspPlanning planner(cloud);
  planner.perceive();
  vpHomogeneousMatrix cMg = planner.get_cMg();

  markerStatus=false;

  while (ros::ok())
  {
    planner.irad = follower.irad;
    planner.ialong = follower.ialong;
    planner.iangle = follower.iangle;
    //Compute new grasp frame with the slides
    planner.recalculate_cMg();
    cMg = planner.get_cMg();
    follower.setMarkerStatus(markerStatus);
    follower.loop(cMg);
    ros::spinOnce();
    if(resetMarker){
      resetMarker = false;
      follower.resetMarker();
    }
  }
  return 0;
}
