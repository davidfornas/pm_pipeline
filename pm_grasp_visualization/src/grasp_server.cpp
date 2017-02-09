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
#include <std_msgs/Float32MultiArray.h>

#include <boost/thread/thread.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

vpHomogeneousMatrix cMg;
bool markerStatus, compute_initial_cMg;
int a,b,c,d,e,f,g;

void paramsCallback(const std_msgs::Float32MultiArray &msg){
  if(msg.data.size()==4){
    a = msg.data[0];
    b = msg.data[1];
    c = msg.data[2];
    g = msg.data[3];//Hand opening
  }else{
    a = msg.data[0];
    b = msg.data[1];
    c = msg.data[2];
    d = msg.data[3];
    e = msg.data[4];
    f = msg.data[5];
    g = msg.data[6];//Hand opening
  }
  //Same names lead to memory loss of the last position, need to solve this better.
}

void stringCallback(const std_msgs::String &msg ){
  if( msg.data == "init"){
    compute_initial_cMg = true;
  }else
    markerStatus = (msg.data == "guided" ?  false : true );
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

  //SETUP GUI SUBSCRIBERS.....specification_status
  ros::Publisher param_pub = nh.advertise<std_msgs::Float32MultiArray>("/specification_params", 1);

  ros::Subscriber status_sub = nh.subscribe("/specification_status", 1, stringCallback);
  ros::Subscriber param_sub = nh.subscribe("/specification_params", 1, paramsCallback);

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
  cMg = planner.get_cMg();

  //TODO CHange this somehow
  a=0;b=0;c=0;
  markerStatus=false;

  while (ros::ok())
  {
    planner.irad = a;
    planner.iangle = b;
    planner.ialong = c;
    //Compute new grasp frame with the slides
    planner.recalculate_cMg();
    cMg = planner.get_cMg();
    follower.setMarkerStatus(markerStatus);
    follower.loop(cMg);
    ros::spinOnce();
  }
  return 0;
}
