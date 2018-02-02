/** 
 * Test PoseEstimation and variants
 *
 *  Created on: 01/02/2018
 *      Author: dfornas
 */

#include <pm_tools/pcl_tools.h>
#include <pm_perception/pose_estimation.h>

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pose_estimation_test");
  ros::NodeHandle nh;

  //Get cloud from topic
  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, "/stereo_camera/points2");
  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);

  //Filtering
  PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
  PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);


  PCAPoseEstimation pose_est(point_cloud_ptr);
  pose_est.initialize();
  ROS_INFO_STREAM(pose_est.get_cMo());


  return 0;
}

