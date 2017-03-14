/**
 * Grasp server for MERBOTS GUI.
 *
 *  Created on: 8/02/2017
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_manipulation/pm_grasp_planning_split.h>
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


bool marker_status, compute_initial_cMg, reset_marker, execute, do_ransac;

void stringCallback(const std_msgs::String &msg ){
  // DF initFromPose initFromRansac
  if( msg.data == "initFromPose" || msg.data == "initFromRansac" ){
    compute_initial_cMg = true;
    if( msg.data == "initFromRansac" )
      do_ransac = true;
  }else if( msg.data == "guided" || msg.data == "interactive"){
    marker_status = (msg.data == "guided" ?  false : true );
  }else if(msg.data == "markerReset"){
    reset_marker = true;
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

  std::string input_topic("/input_cloud"), final_grasp_pose_topic("/desired_grasp_pose"), cloud_frame_id("sense3d"), object_pose_topic("/object");
  nh.getParam("input_topic", input_topic);
  nh.getParam("grasp_pose_topic", final_grasp_pose_topic);
  nh.getParam("object_pose_topic", object_pose_topic);
  nh.getParam("cloud_frame_id", cloud_frame_id);


  compute_initial_cMg = false;
  reset_marker = false;
  do_ransac = false;

  //SETUP GUI SUBSCRIBER for specification_status
  ros::Subscriber status_sub = nh.subscribe("/specification_status", 1, stringCallback);

  ros::Publisher params_pub = nh.advertise<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1000);
  ros::Publisher final_pose_pub = nh.advertise<geometry_msgs::Pose>(final_grasp_pose_topic, 1000);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/specification_cloud", 1000);

  ros::Publisher vehicle_pub = nh.advertise<geometry_msgs::Pose>("sim_vehicle_pose", 1);


  tf::TransformListener *listener_ = new tf::TransformListener();
  tf::TransformBroadcaster *broadcaster = new tf::TransformBroadcaster();

  //Variables de configuración: ángulo de agarre, distancias...
  //GET FROM GUI
  bool alignedGrasp = true;

  bool got_wMc = false;

  tf::StampedTransform wMc; // World to camera (sense3d or stereo_camera)

  //For UWSim visualization
  tf::StampedTransform wMv; // World to vehicle Girona500
  tf::StampedTransform cMv; // Camera to vehicle Girona500

  while (!compute_initial_cMg || !got_wMc)
  {
    try{
      listener_->lookupTransform( "world", cloud_frame_id, ros::Time(0), wMc); // "sense3d"
      got_wMc = true;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Getting TF.");

  //Display the simulated vehicle for better visualization
  if( do_ransac ){
    try{
      listener_->lookupTransform( cloud_frame_id, "girona500", ros::Time(0), cMv);
      geometry_msgs::Pose p;
      p.position.x = cMv.getOrigin().x();
      p.position.y = cMv.getOrigin().y();
      p.position.z = cMv.getOrigin().z();
      p.orientation.x = cMv.getRotation().x();
      p.orientation.y = cMv.getRotation().y();
      p.orientation.z = cMv.getRotation().z();
      p.orientation.w = cMv.getRotation().w();
      vehicle_pub.publish( p );
    }
    catch (tf::TransformException ex){
      ROS_DEBUG("%s",ex.what());
    }
  }else{
    try{
      listener_->lookupTransform( "world", "girona500", ros::Time(0), wMv);
      geometry_msgs::Pose p;
      p.position.x = wMv.getOrigin().x();
      p.position.y = wMv.getOrigin().y();
      p.position.z = wMv.getOrigin().z();
      p.orientation.x = wMv.getRotation().x();
      p.orientation.y = wMv.getRotation().y();
      p.orientation.z = wMv.getRotation().z();
      p.orientation.w = wMv.getRotation().w();
      vehicle_pub.publish( p );
    }
    catch (tf::TransformException ex){
      ROS_DEBUG("%s",ex.what());
    }
  }

  ROS_INFO_STREAM("Getting TF done.");

  // Load cloud if required.
  Cloud::Ptr cloud (new pcl::PointCloud<PointT>);
  if( do_ransac ){
    //Point Cloud load
    PCLTools<PointT>::cloudFromTopic(cloud, input_topic);
    PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.01);
    ROS_DEBUG_STREAM("PointCloud loaded and filtered has: " << cloud->points.size() << " data points.");

    sensor_msgs::PointCloud2 message;
    pcl::PCLPointCloud2 pcl_pc;
    pcl::toPCLPointCloud2(*cloud, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, message);
    cloud_pub.publish(message);
    ros::spinOnce();
  }


  //Init planner
  PMGraspPlanningSplit * planner;
  if( do_ransac ){
    planner = new PMGraspPlanningSplit(cloud, nh);
    planner->setPlaneSegmentationParams(0.06, 100);
  }else{
    planner = new PMGraspPlanningSplit(object_pose_topic);
  }
  planner->perceive();
  vpHomogeneousMatrix cMg = planner->get_cMg();

  double angle = 75, rad = 30, along = 20;
  planner->getBestParams(angle, rad, along);

  EefFollower follower("/gripper_pose", nh, angle, rad, along);
  follower.setWorldToCamera( VispTools::vispHomogFromTfTransform( wMc ) );

  marker_status=false, execute=false;

  std_msgs::Float32MultiArray msg;
  msg.data.push_back(follower.irad);
  msg.data.push_back(follower.iangle);
  msg.data.push_back(follower.ialong);
  msg.data.push_back(10);
  params_pub.publish(msg);


  while (ros::ok())
  {
    planner->irad = follower.irad;
    planner->ialong = follower.ialong;
    planner->iangle = follower.iangle;
    //Compute new grasp frame with the slides
    planner->recalculate_cMg();
    cMg = planner->get_cMg();
    follower.setMarkerStatus(marker_status);
    follower.loop(cMg);
    ros::spinOnce();
    if(reset_marker){
      reset_marker = false;
      follower.resetMarker();
    }

    if(execute){
      //PUBLISH POSE FRAME FOR EXECUTION in TF and in POSE.
      geometry_msgs::Pose gp;
      if( do_ransac ){
        gp.position.x = follower.grasp_pose_world.position.x;
        gp.position.y = follower.grasp_pose_world.position.y;
        gp.position.z = follower.grasp_pose_world.position.z;
        gp.orientation.x = follower.grasp_pose_world.orientation.x;
        gp.orientation.y = follower.grasp_pose_world.orientation.y;
        gp.orientation.z = follower.grasp_pose_world.orientation.z;
        gp.orientation.w = follower.grasp_pose_world.orientation.w;
        final_pose_pub.publish(gp);
      }else{
        gp.position.x = follower.grasp_pose.position.x;
        gp.position.y = follower.grasp_pose.position.y;
        gp.position.z = follower.grasp_pose.position.z;
        gp.orientation.x = follower.grasp_pose.orientation.x;
        gp.orientation.y = follower.grasp_pose.orientation.y;
        gp.orientation.z = follower.grasp_pose.orientation.z;
        gp.orientation.w = follower.grasp_pose.orientation.w;
        final_pose_pub.publish(gp);
      }
      geometry_msgs::PoseStamped gps;
      gps.pose = gp;
      gps.header.stamp = ros::Time::now();
      gps.header.frame_id = "world";

      tf::Stamped<tf::Pose> pose;
      tf::poseStampedMsgToTF( gps, pose );
      tf::StampedTransform t(pose, ros::Time::now(), "/world", final_grasp_pose_topic);

      broadcaster->sendTransform(t);

    }
  }
  return 0;
}
