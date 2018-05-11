/**
 * Grasp server for MERBOTS GUI and ranking grasps.
 *
 *  Created on: 11/04/2018
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_manipulation/ranking_grasp_planner.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/marker_tools.h>
#include <pm_tools/tf_tools.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

//bool marker_status, compute_initial_cMg, reset_marker, execute, do_ransac;

// True: use RANSAC Cylinder, false: use PCA + OpernRave
bool ransac_method, compute_cMg_list;

void stringCallback(const std_msgs::String &msg ){
  if( msg.data == "initFromRansac" || msg.data == "initFromSQ") {
    compute_cMg_list = true;
    if (msg.data == "initFromRansac")
      ransac_method = true;
    else
      ransac_method = false;
  }//Option to execute...
}

/** Plans a grasp on a point cloud and visualize the list of grasps using UWSim externally. Subscribes to GUI Commands...
 */
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ranking_grasp_server");
  ros::NodeHandle nh;

  std::string input_topic("/input_cloud"), final_grasp_pose_topic("/desired_grasp_pose"), cloud_frame_id("stereo"), object_pose_topic("/object");
  nh.getParam("input_topic", input_topic);
  nh.getParam("grasp_pose_topic", final_grasp_pose_topic);
  nh.getParam("object_pose_topic", object_pose_topic);
  nh.getParam("cloud_frame_id", cloud_frame_id);

  compute_cMg_list = false;
  ransac_method = true;

  //SETUP GUI SUBSCRIBER for specification_status
  ros::Subscriber status_sub = nh.subscribe("/specification_status", 1, stringCallback);
  ros::Publisher params_pub = nh.advertise<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1000);
  ros::Publisher final_pose_pub = nh.advertise<geometry_msgs::Pose>(final_grasp_pose_topic, 1000);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/specification_cloud", 1000);

  tf::TransformListener *listener_ = new tf::TransformListener();
  tf::TransformBroadcaster *broadcaster = new tf::TransformBroadcaster();

  //For UWSim visualization
  tf::StampedTransform wMv; // World to vehicle Girona500
  tf::StampedTransform cMv; // Camera to vehicle Girona500
  tf::StampedTransform wMc; // World to camera (sense3d or stereo_camera)

  ROS_INFO_STREAM("Getting world to cloud frame id...");
  bool got_wMc = false;
  while (!compute_cMg_list || !got_wMc)
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

  clock_t begin, end;

  Cloud::Ptr cloud (new pcl::PointCloud<PointT>), aux_cloud (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromTopic(aux_cloud, input_topic);
  ROS_INFO_STREAM("Initial cloud has: " << PCLTools<PointT>::nanAwareCount(aux_cloud) << " data points (not NaN).");
  begin = clock();
  PCLTools<PointT>::applyZAxisPassthrough(aux_cloud, cloud, 0.5, 3);//Removes far away points
  PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.008);//Was 0.01
  end = clock();
  pcl::copyPointCloud(*cloud, *aux_cloud);
  ROS_INFO_STREAM("Downsample time: " << double(end - begin) / CLOCKS_PER_SEC);
  ROS_INFO_STREAM("PointCloud loaded and filtered has: " << cloud->points.size() << " data points.");

  sensor_msgs::PointCloud2 message;
  pcl::PCLPointCloud2 pcl_pc;
  pcl::toPCLPointCloud2(*aux_cloud, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, message);
  message.header.frame_id = "camera";
  cloud_pub.publish(message);
  ros::spinOnce();

  //Init planners
  CylinderRankingGraspPlanner ransac_planner(cloud, nh);
  SQRankingGraspPlanner sq_planner(cloud, nh);

  if(ransac_method){
    CylinderRankingGraspPlanner crgp(cloud, nh);
    crgp.generateGraspList();
    crgp.getBestGrasp();

    while(ros::ok()) {
      ROS_INFO_STREAM("Publish grasp list in TF..");
      crgp.publishGraspList();
    }
  }else{
    SQRankingGraspPlanner srgp(cloud, nh, true);
    srgp.setGraspsParams(300, 0.5, 0.03, 0, 3.1416*2, 3.1416/4);
    bool success = srgp.generateGraspList();

    while(ros::ok() && !success){
      PCLTools<PointT>::cloudFromTopic(cloud, input_topic);
      PCLTools<PointT>::removeNanPoints(cloud);
      PCLTools<PointT>::applyZAxisPassthrough(cloud, 0, 3.5);
      PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.01);
      srgp.setNewCloud(cloud);
      success = srgp.generateGraspList();
    }

    vpHomogeneousMatrix best;
    srgp.filterGraspList();
    best = srgp.getBestGrasp();

    while(ros::ok()) {
      ROS_INFO_STREAM("Publish grasp list in TF..");
      srgp.publishGraspList( 1.5 );
    }
  }
/**
  EefFollower follower("/gripper_pose", nh, angle, rad, along);
  follower.setWorldToCamera( VispTools::vispHomogFromTfTransform( wMc ) );


    cMg = planner->get_cMg();
    follower.setMarkerStatus(marker_status);
    follower.loop(cMg);
    ros::spinOnce();
    if(ransac_method)
      planner->publishObjectPose();*/

  return 0;
}
