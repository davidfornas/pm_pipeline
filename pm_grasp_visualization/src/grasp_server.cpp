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


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

vpHomogeneousMatrix cMg;
bool markerStatus;
int a,b,c;

void floatArraySubs(const std_msgs::Float32MultiArray &msg){
  a = msg.data[0];
  b = msg.data[1];
  c = msg.data[2];
}

void stringCallback(const std_msgs::String &msg ){
  markerStatus = (msg.data == "init" ?  false : true );
  //msg->data == "init" ? follower.setMarkerStatus(false) : follower.setMarkerStatus(true) ;
}



/** Plans a grasp on a point cloud and visualizes it using UWSim externally.
 *  Subscribes to GUI Commands...
 */
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "grasp_server");
  ros::NodeHandle nh;

  bool compute_initial_cMg = false;

  //SETUP GUI SUBSCRIBERS.....
  //pos_pub = nh.advertise<geometry_msgs::Pose>(topic, 1);
  //pos_pub = nh.advertise<geometry_msgs::Pose>(topic, 1);
//ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


  EefFollower follower("/gripperPose", nh);


  //Variables de configuración: ángulo de agarre, distancias...
  //GET FROM GUI
  double angle = 0, rad = 0, along = 0;
  bool alignedGrasp = true;
  std::string input_basename("output");
  nh.getParam("input_basename", input_basename);

  //Point Cloud load
  Cloud::Ptr cloud (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromPCD(cloud, input_basename); //Load from PCDReader or from topic
  ROS_DEBUG_STREAM("PointCloud has: " << cloud->points.size() << " data points.");

  //WAIT FOR INIT MESSAGE

  while (!compute_initial_cMg)
  {
    std_msgs::String::ConstPtr message = ros::topic::waitForMessage< std_msgs::String >("/grasp");
    if( message->data == "init" ) compute_initial_cMg = true;
    //ros::spinOnce();
  }

  //Init planner
  PMGraspPlanning planner(cloud);
  planner.perceive();
  cMg = planner.get_cMg();

  //TODO CHange this
  a=0;b=0;c=0;

  while (ros::ok())
  {
    planner.irad = a;
    planner.iangle = b;
    planner.ialong = c;
    //Compute new grasp frame with the slides
    planner.recalculate_cMg();
    cMg = planner.get_cMg();
    follower.loop(cMg);
    ros::spinOnce();
  }
  return 0;
}
