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

  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>), aux_cloud(new pcl::PointCloud<PointT>),
                               original_cloud(new pcl::PointCloud<PointT>), symmetry_cloud(new pcl::PointCloud<PointT>);
  //std::string filename("/stereo_camera/points2");
  std::string filename("/stereo/points2");
  PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
  PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
  PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);

  PoseEstimation * pose_est;
  //pose_est = new SpherePoseEstimation(nh, point_cloud_ptr);
  pose_est = new CylinderPoseEstimation(nh, point_cloud_ptr);
  //pose_est = new BoxPoseEstimation(nh, point_cloud_ptr);

  pose_est->setDebug(true);
  pose_est->setPlaneSegmentationParams(0.04);
  pose_est->initialize();
  ROS_INFO_STREAM(pose_est->get_cMo());

  while(ros::ok()){
    PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
    pcl::copyPointCloud(*point_cloud_ptr, *original_cloud);
    PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
    PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
    PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);

    pose_est->setNewCloud(point_cloud_ptr);
    bool success = pose_est->process();
    pose_est->estimateSQ(aux_cloud);
    symmetry_cloud = pose_est->getObjectCloud();
    PCLView<PointT>::showCloud(aux_cloud);

    ROS_INFO("TEST");
    pcl::visualization::PCLVisualizer *p;
    p = new pcl::visualization::PCLVisualizer("Cylinde + SQ colored");
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h(point_cloud_ptr, 0, 230, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h(aux_cloud, 0, 0, 230);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h2(symmetry_cloud, 250, 0, 0);
    p->setBackgroundColor(0.8, 0.8, 0.8);
    p->addPointCloud(original_cloud, "c1");
    p->addPointCloud(aux_cloud, src_h, "c2");
    p->addPointCloud(symmetry_cloud, src_h2, "c3");
    p->spin();

  }
  return 0;
}

