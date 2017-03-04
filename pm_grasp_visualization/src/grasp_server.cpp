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
#include <geometry_msgs/PoseStamped.h>

typedef pcl::PointXYZRGB PointT;
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

  compute_initial_cMg = false;
  resetMarker = false;

  //SETUP GUI SUBSCRIBER for specification_status
  ros::Subscriber status_sub = nh.subscribe("/specification_status", 1, stringCallback);

  ros::Publisher params_pub = nh.advertise<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1000);
  ros::Publisher final_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/final_pose", 1000);

  tf::TransformListener *listener_ = new tf::TransformListener();

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
  PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.001);
  ROS_DEBUG_STREAM("PointCloud has: " << cloud->points.size() << " data points.");


  bool got_wMc = false;
  tf::StampedTransform wMc;;
  while (!compute_initial_cMg && !got_wMc)
  {
    try{
      listener_->lookupTransform("sense3d", "world", ros::Time(0), wMc);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return false;
    }
    ros::spinOnce();
  }

  //Init planner
  PMGraspPlanning planner(cloud);
  planner.setPlaneSegmentationParams(0.06, 100);
  planner.perceive();
  vpHomogeneousMatrix cMg = planner.get_cMg();

  EefFollower follower("/gripper_pose", nh);
  follower.setWorldToCamera(VispTools::vispHomogFromTfTransform( wMc ));

  markerStatus=false, execute=false;

  std_msgs::Float32MultiArray msg;
  msg.data.push_back(follower.irad);
  msg.data.push_back(follower.iangle);
  msg.data.push_back(follower.ialong);
  msg.data.push_back(10);
  params_pub.publish(msg);



  //TODO, AL EMPEZAR HAY QUE GUARDAR EL MARCO ENTRE EL WORLD Y SESNSE 3D

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
    execute = true;
    if(execute){
      //PUBLISH POSE FRAME FOR EXECUTION in TF and in POSE.
      /*geometry_msgs::PoseStamped gp;
      gp.pose.position.x = follower.grasp_pose_final.position.x;
      gp.pose.position.y = follower.grasp_pose_final.position.y;
      gp.pose.position.z = follower.grasp_pose_final.position.z;
      gp.pose.orientation.x = follower.grasp_pose_final.orientation.x;
      gp.pose.orientation.y = follower.grasp_pose_final.orientation.y;
      gp.pose.orientation.z = follower.grasp_pose_final.orientation.z;
      gp.pose.orientation.w = follower.grasp_pose_final.orientation.w;
      gp.header.stamp = ros::Time::now();
      gp.header.frame_id = cloud->header.frame_id;
      final_pose_pub.publish(gp);

      tf::Stamped<tf::Pose> pose;
      tf::poseStampedMsgToTF( gp, pose );
      tf::StampedTransform t(pose, ros::Time::now(), "/world", "/desired_grasp_pose");

      broadcaster->sendTransform(t);*/

      //Mejorar y automatizar
      //Cambiar el frame del brazo para que la X apunte hacia...preguntar a Toni.
      //Introducir el coloreado en UWSim para evitar procesar la nube cada vez...

      execute = false;
    }
  }
  return 0;
}
