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

  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>);
  //std::string filename("/stereo_camera/points2");
  std::string filename("/stereo/points2");
  PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
  PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
  PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);

  PoseEstimation * pose_est;
//  pose_est = new SpherePoseEstimation(point_cloud_ptr);
//  pose_est = new CylinderPoseEstimation(point_cloud_ptr);
  pose_est = new PCAPoseEstimation(point_cloud_ptr, 200); //200 thereshold gets also the stone.
  //pose_est = new BoxPoseEstimation(point_cloud_ptr);

  pose_est->setDebug(true);
  //Only required for CYLINDER pose_est->initialize();
  //pose_est->setPlaneSegmentationParams(0.7);
  ROS_INFO_STREAM(pose_est->get_cMo());
  while(ros::ok()){
    PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
    PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
    PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
    PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);

    pose_est->setNewCloud(point_cloud_ptr);
    bool success = pose_est->process();
    ROS_INFO_STREAM(pose_est->get_cMo());

    int i = 0;
    while(success) {
      PCLTools<PointT>::moveToOrigin(pose_est->getObjectCloud());
      std::stringstream cloudid;
      cloudid << "cluster" << i++ << ".pcd";
      PCLTools<PointT>::cloudToPCD(pose_est->getObjectCloud(), cloudid.str() );
      PCLView<PointT>::showCloudDuring(pose_est->getObjectCloud(), 500);
      success = ((PCAPoseEstimation *) pose_est)->processNext();
    }
  }
  return 0;
}

