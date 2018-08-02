/**
 * Grasp server for MERBOTS GUI.
 *
 *  Created on: 8/02/2017
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_manipulation/slider_grasp_planner.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/marker_tools.h>
#include <pm_tools/tf_tools.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

bool marker_status, compute_initial_cMg, reset_marker, execute, do_ransac;
Method method;

void stringCallback(const std_msgs::String &msg ){
  if( msg.data == "initFromPose" || msg.data == "initFromRansac" || msg.data == "initFromBox"
      || msg.data == "initFromPCA" || msg.data == "initFromSphere" || msg.data == "initFromSQ"){
    compute_initial_cMg = true;
    if( msg.data == "initFromRansac" || msg.data == "initFromBox" || msg.data == "initFromPCA"
        || msg.data == "initFromSphere" || msg.data == "initFromSQ"){
      do_ransac = true;
      if(msg.data == "initFromRansac") method = RANSACCylinder;
      if(msg.data == "initFromSphere") method = RANSACSphere;
      if(msg.data == "initFromBox") method = BoxPlane;
      if(msg.data == "initFromPCA") method = PCA;
      if(msg.data == "initFromSQ") method = SQ;
    }
  }else if( msg.data == "guided" || msg.data == "interactive"){
    marker_status = (msg.data == "guided" ?  false : true );
  }else if(msg.data == "markerReset"){
    reset_marker = true;
  }else{
    execute = true;
  }
}

/** Plans a grasp on a point cloud and visualizes it using UWSim externally. Subscribes to GUI Commands... */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "slider_grasp_server_split");
  ros::NodeHandle nh;

  std::string input_topic("/input_cloud"), final_grasp_pose_topic("/desired_grasp_pose"), cloud_frame_id("sense3d"), object_pose_topic("/object");
  nh.getParam("input_topic", input_topic);
  nh.getParam("grasp_pose_topic", final_grasp_pose_topic);
  nh.getParam("object_pose_topic", object_pose_topic);
  nh.getParam("cloud_frame_id", cloud_frame_id);

  compute_initial_cMg = false;
  reset_marker = false;
  do_ransac = false;

  ros::Subscriber status_sub = nh.subscribe("/specification_status", 1, stringCallback);

  ros::Publisher params_pub = nh.advertise<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1000);
  ros::Publisher final_pose_pub = nh.advertise<geometry_msgs::Pose>(final_grasp_pose_topic, 1000);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/specification_cloud", 1000);
  ros::Publisher vehicle_pub = nh.advertise<geometry_msgs::Pose>("/sim_vehicle_pose", 1);

  ros::Publisher fullProcessTimePublisher = nh.advertise<std_msgs::Float32>("/stats/processCloud", 1);
  ros::Publisher loadTimePublisher = nh.advertise<std_msgs::Float32>("/stats/loadCloud", 1);
  ros::Publisher filterTimePublisher = nh.advertise<std_msgs::Float32>("/stats/filterCloud", 1);

  tf::TransformListener *listener_ = new tf::TransformListener();
  tf::TransformBroadcaster *broadcaster = new tf::TransformBroadcaster();

  //For UWSim visualization
  tf::StampedTransform wMv; // World to vehicle Girona500
  tf::StampedTransform cMv; // Camera to vehicle Girona500
  tf::StampedTransform wMc; // World to camera (sense3d or stereo_camera)

  ROS_INFO_STREAM("Getting world to cloud frame id...");
  bool got_wMc = false;
  while (!compute_initial_cMg || !got_wMc)
  {
    try{
      listener_->lookupTransform( "world", cloud_frame_id, ros::Time(0), wMc);
      got_wMc = true;
    }
    catch (tf::TransformException ex){
      ROS_DEBUG("%s",ex.what());
    }
    ros::spinOnce();
  }

  ROS_INFO_STREAM("World to camera OK. Getting vehicle frames...");
  //Display the simulated vehicle for better visualization
  if( do_ransac ){
    try{
      listener_->lookupTransform( cloud_frame_id, "girona500", ros::Time(0), cMv);
      geometry_msgs::Pose p;
      p.position.x = cMv.getOrigin().x(); p.position.y = cMv.getOrigin().y(); p.position.z = cMv.getOrigin().z();
      p.orientation.x = cMv.getRotation().x(); p.orientation.y = cMv.getRotation().y();
      p.orientation.z = cMv.getRotation().z(); p.orientation.w = cMv.getRotation().w();
      vehicle_pub.publish( p );
    }
    catch (tf::TransformException & ex){
      ROS_INFO("Cannot retrieve vehicle position. Not displaying vehicle position. %s",ex.what());
    }
  }else{
    try{
      listener_->lookupTransform( "world", "girona500", ros::Time(0), wMv);
      geometry_msgs::Pose p;
      p.position.x = wMv.getOrigin().x(); p.position.y = wMv.getOrigin().y();p.position.z = wMv.getOrigin().z();
      p.orientation.x = wMv.getRotation().x(); p.orientation.y = wMv.getRotation().y();
      p.orientation.z = wMv.getRotation().z(); p.orientation.w = wMv.getRotation().w();
      vehicle_pub.publish( p );
    }
    catch (tf::TransformException & ex){
      ROS_INFO("Cannot retrieve vehicle position. Not displaying vehicle position. %s",ex.what());
    }
  }

  // Load cloud if required.
  Cloud::Ptr cloud (new pcl::PointCloud<PointT>), aux_cloud (new pcl::PointCloud<PointT>);
  if( do_ransac ){
    Timing tick;
    PCLTools<PointT>::cloudFromTopic(aux_cloud, input_topic);
    loadTimePublisher.publish(tick.getTotalTimeMsg());
    ROS_INFO_STREAM("Cloud load time: " << tick.getTotalTimeMsg());
    ROS_INFO_STREAM("Initial cloud has: " << PCLTools<PointT>::nanAwareCount(aux_cloud) << " data points (not NaN).");

    tick.resetTimer();
    PCLTools<PointT>::applyZAxisPassthrough(aux_cloud, cloud, 0.5, 3);//Removes far away points
    PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.008);//Was 0.01

    filterTimePublisher.publish(tick.getTotalTimeMsg());
    ROS_INFO_STREAM("Downsample time: " << tick.getTotalTimeMsg());

    pcl::copyPointCloud(*cloud, *aux_cloud);
    ROS_INFO_STREAM("PointCloud loaded and filtered has: " << cloud->points.size() << " data points.");

    sensor_msgs::PointCloud2 message;
    pcl::PCLPointCloud2 pcl_pc;
    pcl::toPCLPointCloud2(*aux_cloud, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, message);
    message.header.frame_id = "camera"; //New
    cloud_pub.publish(message);
    ros::spinOnce();
  }

  SliderGraspPlanner * planner;
  if( do_ransac ){
    ROS_INFO_STREAM("Specification using pose estimation");
    planner = new SliderGraspPlanner(cloud, nh, object_pose_topic, method); //, VispTools::vispHomogFromTfTransform( wMc ));
    planner->pose_estimation->setPlaneSegmentationParams(0.08, 100);//006
  }else{
    ROS_INFO_STREAM("Specification using a input object Pose from topic.");
    planner = new SliderGraspPlanner(object_pose_topic);
  }

  Timing tick;
  planner->perceive();
  fullProcessTimePublisher.publish(tick.getTotalTimeMsg());
  ROS_INFO_STREAM("First computing time: " << tick.getTotalTimeMsg());

  vpHomogeneousMatrix cMg = planner->get_cMg();

  double angle = 75, rad = 18, along = 20;
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
  ros::spinOnce();

  while (ros::ok())
  {
    planner->irad = follower.irad;
    planner->ialong = follower.ialong;
    planner->iangle = follower.iangle;

    if( !do_ransac ){
      planner->perceive();
    }else{

      Timing tick;
      PCLTools<PointT>::cloudFromTopic(aux_cloud, input_topic);
      ROS_INFO_STREAM("Cloud load time: " << tick.getTotalTimeMsg());
      loadTimePublisher.publish(tick.getTotalTimeMsg());

      tick.resetTimer();
      PCLTools<PointT>::applyZAxisPassthrough(aux_cloud, cloud, 0.5, 3);//Removes far away points
      PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.008);
      filterTimePublisher.publish(tick.getTotalTimeMsg());
      ROS_INFO_STREAM("Online downsample time: " << tick.getTotalTimeMsg());

      pcl::copyPointCloud(*cloud, *aux_cloud);
      ROS_DEBUG_STREAM("PointCloud loaded and filtered has: " << cloud->points.size() << " data points.");


      planner->setNewCloud(cloud);
      tick.resetTimer();
      planner->redoRansac();
      fullProcessTimePublisher.publish(tick.getTotalTimeMsg());
      ROS_INFO_STREAM("Finished. Pose estimation processing time: " << tick.getTotalTimeMsg());

      sensor_msgs::PointCloud2 message;
      pcl::PCLPointCloud2 pcl_pc;
      pcl::toPCLPointCloud2(*aux_cloud, pcl_pc);
      pcl_conversions::fromPCL(pcl_pc, message);
      message.header.frame_id = "camera";
      cloud_pub.publish(message);
      ros::spinOnce();
    }

    //Compute new grasp frame with the slider values
    planner->recalculate_cMg();
    cMg = planner->get_cMg();
    follower.setMarkerStatus(marker_status);
    follower.loop(cMg);
    ros::spinOnce();
    if(reset_marker){
      reset_marker = false;
      follower.resetMarker();
    }
    if(do_ransac && method == RANSACCylinder )
      planner->publishObjectPose();

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
    }else{
      gp.position.x = follower.grasp_pose.position.x;
      gp.position.y = follower.grasp_pose.position.y;
      gp.position.z = follower.grasp_pose.position.z;
      gp.orientation.x = follower.grasp_pose.orientation.x;
      gp.orientation.y = follower.grasp_pose.orientation.y;
      gp.orientation.z = follower.grasp_pose.orientation.z;
      gp.orientation.w = follower.grasp_pose.orientation.w;
    }
    geometry_msgs::PoseStamped gps;
    gps.pose = gp;
    gps.header.stamp = pcl_conversions::fromPCL( cloud->header.stamp );
    gps.header.frame_id = "world";

    tf::Stamped<tf::Pose> pose;
    tf::poseStampedMsgToTF( gps, pose );
    tf::StampedTransform t(pose, ros::Time::now(), "/world", final_grasp_pose_topic);

    broadcaster->sendTransform(t);
    if(execute){
      final_pose_pub.publish(gp);
    }
  }
  return 0;
}
