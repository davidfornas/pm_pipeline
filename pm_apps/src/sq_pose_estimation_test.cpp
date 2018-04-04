/** 
 * Test SQPoseEstimation
 *
 *  Created on: 03/02/2018
 *      Author: dfornas
 */

#include <pm_tools/pcl_tools.h>
#include <pm_perception/pose_estimation.h>

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "sq_pose_estimation_test");
  ros::NodeHandle nh;

  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>), original_cloud(new pcl::PointCloud<PointT>);
  std::string filename("/stereo/points2");

  PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
  PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
  PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);

  SQPoseEstimation * pose_est;
  pose_est = new SQPoseEstimation(point_cloud_ptr);

  pose_est->setDebug(true);
  while(ros::ok()){
    PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
    PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
    PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
    PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);

    pcl::copyPointCloud<PointT,PointT>(*point_cloud_ptr, *original_cloud);

    pose_est->setNewCloud(point_cloud_ptr);
    bool success = pose_est->process();
    ROS_INFO_STREAM(pose_est->get_cMo());

    while(success){
      *original_cloud += *((SQPoseEstimation*)pose_est)->getSQCloud();
      *pose_est->getObjectCloud() += *((SQPoseEstimation*)pose_est)->getSQCloud();
      PCLView<PointT>::showCloudDuring(pose_est->getObjectCloud());
      //PCLView<PointT>::showCloudDuring(((SQPoseEstimation*)pose_est)->getSQCloud());
      success = ((SQPoseEstimation*)pose_est)->processNext();
    }

    PCLView<PointT>::showCloudDuring(original_cloud);
    PCLTools<PointT>::cloudToPCD(original_cloud, "result.pcd");
  }
  return 0;
}

