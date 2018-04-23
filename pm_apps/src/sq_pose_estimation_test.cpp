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

  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>), original_cloud(new pcl::PointCloud<PointT>), sq_cloud(new pcl::PointCloud<PointT>);
  std::string filename("/stereo/points2");

  PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
  PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
  PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);

  SQPoseEstimation * pose_est;
  pose_est = new SQPoseEstimation(point_cloud_ptr, 400, 0.01);
  //pose_est->setRegionGrowingClustering(8.0, 8.0);
  //pose_est->setLMFitting();
  //pose_est->setSymmetrySearchParams(0.0);
  pose_est->setSymmetrySearchParams(0.40, 0.05, 0.2);
  pose_est->setAxisSymmetryMode();

  pose_est->setDebug(true);
  while(ros::ok()){
    PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
    pcl::copyPointCloud<PointT,PointT>(*point_cloud_ptr, *original_cloud);
    PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
    PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
    PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);


    pose_est->setNewCloud(point_cloud_ptr);
    bool success = pose_est->process();

    while(success){
      ROS_INFO_STREAM(pose_est->get_cMo());
      *sq_cloud += *((SQPoseEstimation*)pose_est)->getSQCloud();
      //*pose_est->getObjectCloud() += *((SQPoseEstimation*)pose_est)->getSQCloud();
      //PCLView<PointT>::showCloudDuring(pose_est->getObjectCloud());
      success = ((SQPoseEstimation*)pose_est)->processNext();
    }

    pcl::visualization::PCLVisualizer *p;
    p = new pcl::visualization::PCLVisualizer("SQ colored");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h(sq_cloud, 0, 0, 230);
    p->setBackgroundColor(0.8,0.8,0.8);
    p->addPointCloud(original_cloud, "c1");
    p->addPointCloud(sq_cloud, src_h, "c2");
    p->spin();
    PCLTools<PointT>::cloudToPCD(sq_cloud, "sq_result.pcd");
  }
  return 0;
}

