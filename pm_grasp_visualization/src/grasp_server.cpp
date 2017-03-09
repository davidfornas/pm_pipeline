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
#include <pm_tools/tf_tools.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;


bool markerStatus, compute_initial_cMg, resetMarker, execute;

void stringCallback(const std_msgs::String &msg ){
  if( msg.data == "init"){
    compute_initial_cMg = true;
  }else if( msg.data == "guided" || msg.data == "interactive"){
    markerStatus = (msg.data == "guided" ?  false : true );
  }else if(msg.data == "markerReset"){
    resetMarker = true;
  }else{
    execute = true;
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

  std::string input_topic("/input_cloud"), final_grasp_pose_topic("/desired_grasp_pose"), cloud_frame_id("sense3d");
  nh.getParam("input_topic", input_topic);
  nh.getParam("grasp_pose_topic", final_grasp_pose_topic);
  nh.getParam("cloud_frame_id", cloud_frame_id);


  compute_initial_cMg = false;
  resetMarker = false;

  //SETUP GUI SUBSCRIBER for specification_status
  ros::Subscriber status_sub = nh.subscribe("/specification_status", 1, stringCallback);

  ros::Publisher params_pub = nh.advertise<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1000);
  ros::Publisher final_pose_pub = nh.advertise<geometry_msgs::Pose>(final_grasp_pose_topic, 1000);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/specification_cloud", 1000);


  tf::TransformListener *listener_ = new tf::TransformListener();
  tf::TransformBroadcaster *broadcaster = new tf::TransformBroadcaster();

  //Variables de configuración: ángulo de agarre, distancias...
  //GET FROM GUI
  bool alignedGrasp = true;

  bool got_wMc = false;
  tf::StampedTransform wMc;
  while (!compute_initial_cMg && !got_wMc)
  {
    try{
      listener_->lookupTransform( "world", cloud_frame_id, ros::Time(0), wMc); // "sense3d"
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    ros::spinOnce();
  }

  //Point Cloud load
  Cloud::Ptr cloud (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromTopic(cloud, input_topic);
  PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.01);
  ROS_DEBUG_STREAM("PointCloud loaded and filtered has: " << cloud->points.size() << " data points.");

  sensor_msgs::PointCloud2 message;
  pcl::PCLPointCloud2 pcl_pc;
  pcl::toPCLPointCloud2(*cloud, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, message);
  cloud_pub.publish(message);
  ros::spinOnce();

  //Init planner
  PMGraspPlanning planner(cloud);
  planner.setPlaneSegmentationParams(0.06, 100);
  planner.perceive();
  vpHomogeneousMatrix cMg = planner.get_cMg();


  double angle = 45, rad = 30, along = 20;
  planner.getBestParams(angle, rad, along);

  EefFollower follower("/gripper_pose", nh, angle, rad, along);
  follower.setWorldToCamera( VispTools::vispHomogFromTfTransform( wMc ) );

  markerStatus=false, execute=false;

  std_msgs::Float32MultiArray msg;
  msg.data.push_back(follower.irad);
  msg.data.push_back(follower.iangle);
  msg.data.push_back(follower.ialong);
  msg.data.push_back(10);
  params_pub.publish(msg);


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
    //execute = true;
    if(execute){
      //PUBLISH POSE FRAME FOR EXECUTION in TF and in POSE.
      geometry_msgs::PoseStamped gp;
      gp.pose.position.x = follower.grasp_pose_world.position.x;
      gp.pose.position.y = follower.grasp_pose_world.position.y;
      gp.pose.position.z = follower.grasp_pose_world.position.z;
      gp.pose.orientation.x = follower.grasp_pose_world.orientation.x;
      gp.pose.orientation.y = follower.grasp_pose_world.orientation.y;
      gp.pose.orientation.z = follower.grasp_pose_world.orientation.z;
      gp.pose.orientation.w = follower.grasp_pose_world.orientation.w;
      gp.header.stamp = ros::Time::now();
      gp.header.frame_id = "world";

      geometry_msgs::Pose gp2;
      gp2.position.x = follower.grasp_pose_world.position.x;
      gp2.position.y = follower.grasp_pose_world.position.y;
      gp2.position.z = follower.grasp_pose_world.position.z;
      gp2.orientation.x = follower.grasp_pose_world.orientation.x;
      gp2.orientation.y = follower.grasp_pose_world.orientation.y;
      gp2.orientation.z = follower.grasp_pose_world.orientation.z;
      gp2.orientation.w = follower.grasp_pose_world.orientation.w;
      final_pose_pub.publish(gp2);

      //final_pose_pub.publish(gp);

      tf::Stamped<tf::Pose> pose;
      tf::poseStampedMsgToTF( gp, pose );
      tf::StampedTransform t(pose, ros::Time::now(), "/world", final_grasp_pose_topic);

      broadcaster->sendTransform(t);
      execute = false;
    }
  }
  return 0;
}
