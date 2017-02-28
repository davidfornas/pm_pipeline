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

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

tf::TransformListener *listener_;
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

bool transformPose(const geometry_msgs::PoseStamped& pose_in, geometry_msgs::PoseStamped &pose_out, const std::string& target_frame_id)
{

  try{
    listener_->transformPose(target_frame_id, ros::Time(0), pose_in, "/sense3d", pose_out);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  pose_out.header.frame_id = target_frame_id;
  return true;
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

  ros::Publisher params_pub = nh.advertise<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1000);
  ros::Publisher final_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/final_pose", 1000);

  listener_ = new tf::TransformListener();

  tf::TransformBroadcaster *broadcaster;

  //Variables de configuración: ángulo de agarre, distancias...
  //GET FROM GUI
  double angle = 0, rad = 0, along = 0;
  bool alignedGrasp = true;
  std::string input_topic("/input_cloud");
  nh.getParam("input_topic", input_topic);

  //Point Cloud load
  Cloud::Ptr cloud (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromTopic(cloud, input_topic);
  PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.005

                                         );
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

  EefFollower follower("/gripper_pose", nh);

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
    if(execute){
      //PUBLISH POSE FRAME FOR EXECUTION
      geometry_msgs::PoseStamped in, out;
      in.pose.position.x = follower.grasp_pose.position.x;
      in.pose.position.y = follower.grasp_pose.position.y;
      in.pose.position.z = follower.grasp_pose.position.z;
      in.pose.orientation.x = follower.grasp_pose.orientation.x;
      in.pose.orientation.y = follower.grasp_pose.orientation.y;
      in.pose.orientation.z = follower.grasp_pose.orientation.z;
      in.pose.orientation.w = follower.grasp_pose.orientation.w;
      in.header.stamp = ros::Time::now();
      in.header.frame_id = cloud->header.frame_id;
      out = in;

      transformPose(in, out, "world");
      final_pose_pub.publish(out);

      tf::Stamped<tf::Pose> pose;
      tf::poseStampedMsgToTF( out, pose );
      tf::StampedTransform t(pose, ros::Time::now(), "/world", "/desired_grasp_pose");
      broadcaster->sendTransform(t);

      execute = false;
    }
  }
  return 0;
}
