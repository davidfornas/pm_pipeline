/** 
 * Test PoseEstimation and variants
 *
 *  Created on: 01/02/2018
 *      Author: dfornas
 */

#include <pm_tools/pcl_tools.h>
#include <pm_perception/ransac_pose_estimation.h>

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pose_estimation_test");
  ros::NodeHandle nh;
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/specification_cloud", 1000);

  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>), aux_cloud(new pcl::PointCloud<PointT>),
                               original_cloud(new pcl::PointCloud<PointT>), object_cloud(new pcl::PointCloud<PointT>);
  //std::string filename("/stereo_camera/points2");
  std::string filename("/stereo/points2");

  SpherePoseEstimation * pose_est;
  pose_est = new SpherePoseEstimation(nh, point_cloud_ptr);

  pose_est->setDebug(true);
  pose_est->setPlaneSegmentationParams(0.05);
  pose_est->setSphereSegmentationParams(0.1, 20000, 0.15);

  while(ros::ok()){
    PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
    pcl::copyPointCloud(*point_cloud_ptr, *original_cloud);
    PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
    PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
    PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);

    sensor_msgs::PointCloud2 message;
    pcl::PCLPointCloud2 pcl_pc;
    pcl::toPCLPointCloud2(*point_cloud_ptr, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, message);
    message.header.frame_id = "camera"; //New
    cloud_pub.publish(message);
    ros::spinOnce();

    pose_est->setNewCloud(point_cloud_ptr);
    bool success = pose_est->process();
    if(!success) continue;
    object_cloud = pose_est->getObjectCloud();

    pcl::visualization::PCLVisualizer *p;
    p = new pcl::visualization::PCLVisualizer("Sphere RANSAC detection");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h(aux_cloud, 0, 0, 230);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h2(object_cloud, 250, 0, 0);
    p->setBackgroundColor(0.8, 0.8, 0.8);
    p->addPointCloud(original_cloud, "c1");
    p->addPointCloud(aux_cloud, src_h, "c2");
    p->addPointCloud(object_cloud, src_h2, "c3");
    p->spinOnce();

  }
  return 0;
}

