/** 
 * Registration with wMc
 *  Created on: 21/11/2017
 *      Author: dfornas
 */

#include <pm_tools/tf_tools.h>
#include <pm_tools/visp_tools.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

typedef message_filters::Subscriber<geometry_msgs::PoseStamped> PoseSub;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> CloudSub;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> CloudPoseSync;

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

class MarkerRegistration{

  //TF only needed as alternative to Pose input.
  tf::TransformListener *listener_;
  tf::TransformBroadcaster *broadcaster_;

  //Subscribers and sync
  PoseSub *pose_sub_;
  CloudSub *cloud_sub_;
  message_filters::Synchronizer<CloudPoseSync> *sync;


  geometry_msgs::PoseStamped current_pose_, map_pose_;
  CloudPtr current_cloud_, map_cloud_;

  ros::Publisher cloud_pub_;

  bool first_cloud_;


public:

  MarkerRegistration(ros::NodeHandle &nh){

    broadcaster_ = new tf::TransformBroadcaster();
    listener_ = new tf::TransformListener();

    //Subscribe both to Pose and Cloud to have marker position properly
    pose_sub_ = new PoseSub(nh, "/pose", 1);
    cloud_sub_ = new CloudSub(nh, "/stereo_camera/points2", 1);

    sync = new message_filters::Synchronizer<CloudPoseSync>(CloudPoseSync(5), *pose_sub_, *cloud_sub_);
    sync->registerCallback(boost::bind(&MarkerRegistration::detectionCallback, this, _1, _2));

    current_cloud_ = boost::shared_ptr<Cloud>( new Cloud() );
    map_cloud_ = boost::shared_ptr<Cloud>( new Cloud() );

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/out", 1000);

    first_cloud_ = true;



  }





  void run(){


    //WIP THAT WILL BE USED TO TRANSFORM USING POSE INSTEAD OF TF. DO NOT DELETE JUST IN CASE TF IS PREFERED (not with bags)

    ros::Rate r(10);
    while(ros::ok()){

      bool gotTransform = false;
      tf::StampedTransform markerMcamera; // World to camera (sense3d or stereo_camera)

      while (!gotTransform)
      {
        try{
          listener_->lookupTransform( "ar_marker", "/bumblebee2/left", ros::Time(0), markerMcamera); // "sense3d" or any stereo_frame
          gotTransform = true;
        }
        catch (tf::TransformException ex){
          ROS_DEBUG("%s",ex.what());
        }
      }

      //pose_out.header.frame_id = target_frame_id;
      //SUSCRIBE TO cMm e invertir Ó TO mMc
      vpHomogeneousMatrix mMc = VispTools::vispHomogFromTfTransform( markerMcamera );
      vpHomogeneousMatrix wMm(0, 0, -1, 0, 0, 3.1416);
      vpHomogeneousMatrix wMc = wMm * mMc;

      tf::Transform t = VispTools::tfTransFromVispHomog(wMc);
      tf::StampedTransform transform(t, ros::Time::now(), "world", "/bumblebee2/left");//Not now but the same as the listened: markerMcamera.stamp_
      broadcaster_->sendTransform(transform);
      ros::spinOnce();
      r.sleep();
    }
  }


  void detectionCallback(const geometry_msgs::PoseStamped::ConstPtr& pose, const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {

    //Convert input cloud to PCL
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*cloud, pcl_pc);
    //PCL Generic cloud to PointT strong type.
    pcl::fromPCLPointCloud2(pcl_pc, *current_cloud_);
    ROS_INFO_STREAM("PointCloud loaded: " << current_cloud_->points.size() << " data points." << std::endl);

    current_pose_.header = pose->header;
    current_pose_.pose = pose->pose;

    if(first_cloud_){

      //Initialize map cloud and pose
      map_pose_ = current_pose_;
      first_cloud_ = false;
      map_cloud_->header = current_cloud_->header;
      for(int i=0; i< current_cloud_->points.size(); i++){
        map_cloud_->points.push_back( current_cloud_->points[i] );
      }

    }else{

      //Get trasform between map and current
      vpHomogeneousMatrix wMc1 = VispTools::vispHomogFromGeometryPose(  current_pose_.pose );
      vpHomogeneousMatrix wMc2 = VispTools::vispHomogFromGeometryPose(  map_pose_.pose );
      vpHomogeneousMatrix c1Mc2 = wMc1.inverse() * wMc2;


      //Add cloud points to map

      for(int i=0; i< current_cloud_->points.size(); i++){
        PointT new_point = current_cloud_->points[i];
        vpHomogeneousMatrix point(current_cloud_->points[i].x, current_cloud_->points[i].y, current_cloud_->points[i].z, 0, 0, 0);
        point = point * c1Mc2;
        new_point.x = point [0][2];
        new_point.y = point [1][2];
        new_point.z = point [2][2];
        map_cloud_->points.push_back( new_point );
      }

      sensor_msgs::PointCloud2 out_message;

      pcl::PCLPointCloud2 out_pcl_pc;
      //PointT strong type to PCL Generic cloud.
      pcl::toPCLPointCloud2(*map_cloud_, out_pcl_pc);
      pcl_conversions::fromPCL(out_pcl_pc, out_message);
      cloud_pub_.publish(out_message);

      float summd = (current_pose_.pose.position.x - map_pose_.pose.position.x) + (current_pose_.pose.position.y - map_pose_.pose.position.y) + (current_pose_.pose.position.z - map_pose_.pose.position.z);
      ROS_INFO_STREAM("Marker difference" << summd);
    }

  }

};








int main (int argc, char** argv)
{

  // Set up ROS.
  ros::init(argc, argv, "grasp_server_split");
  ros::NodeHandle nh;



  MarkerRegistration mr(nh);
  mr.run();

  return (0);
}

