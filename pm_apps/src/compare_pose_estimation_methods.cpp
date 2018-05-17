/** 
 * Compare/log different pose estimation methods...
 *
 *  Created on: 17/05/2018
 *      Author: dfornas
 */

#include <pm_tools/pcl_tools.h>
#include <pm_tools/logger.h>
#include <pm_perception/pose_estimation.h>

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pose_estimation_comparison");
  ros::NodeHandle nh;

  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>), aux_cloud(new pcl::PointCloud<PointT>),
                               original_cloud(new pcl::PointCloud<PointT>), point_cloud_ptr2(new pcl::PointCloud<PointT>),
          point_cloud_ptr3(new pcl::PointCloud<PointT>);

  std::string filename("/stereo/points2");
  PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
  PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
  PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);
  pcl::copyPointCloud(*point_cloud_ptr, *point_cloud_ptr2);
  pcl::copyPointCloud(*point_cloud_ptr, *point_cloud_ptr3);

  CylinderPoseEstimation * cylinder_pose_est;
  BoxPoseEstimation * box_pose_est;
  SpherePoseEstimation * sphere_pose_est;

  sphere_pose_est = new SpherePoseEstimation(point_cloud_ptr);
  cylinder_pose_est = new CylinderPoseEstimation(point_cloud_ptr2);
  box_pose_est = new BoxPoseEstimation(point_cloud_ptr3);

  cylinder_pose_est->setDebug(true);
  cylinder_pose_est->setPlaneSegmentationParams(0.04);
  cylinder_pose_est->initialize();
  ROS_INFO_STREAM(cylinder_pose_est->get_cMo());
  //Same with the others

  while(ros::ok()){
    PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
    pcl::copyPointCloud(*point_cloud_ptr, *original_cloud);
    PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
    PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
    PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);
    pcl::copyPointCloud(*point_cloud_ptr, *point_cloud_ptr2);
    pcl::copyPointCloud(*point_cloud_ptr, *point_cloud_ptr3);


    sphere_pose_est->setNewCloud(point_cloud_ptr);
    cylinder_pose_est->setNewCloud(point_cloud_ptr2);
    box_pose_est->setNewCloud(point_cloud_ptr3);

    bool success = cylinder_pose_est->process();
    cylinder_pose_est->estimateSQ(aux_cloud);
    symmetry_cloud = cylinder_pose_est->getObjectCloud();
    PCLView<PointT>::showCloud(aux_cloud);

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

