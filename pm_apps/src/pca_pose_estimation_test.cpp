/** 
 * Test PoseEstimation and variants
 *
 *  Created on: 01/02/2018
 *      Author: dfornas
 */

#include <pm_tools/pcl_tools.h>
#include <pm_perception/pca_pose_estimation.h>

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_estimation_test");
  ros::NodeHandle nh;
  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>),
                               original_cloud(new pcl::PointCloud<PointT>), symmetry_cloud(new pcl::PointCloud<PointT>);
  std::string filename("/stereo/points2");

  PoseEstimation * pose_est;
  pose_est = new PCAPoseEstimation(nh, point_cloud_ptr, 200); //200 thereshold gets also the stone.
  pose_est->setDebug(true);

  // Enable search and Search only for distance.
  //pose_est->setSymmetrySearchParams(0.0, 0.1, 0.05);
  pose_est->setSymmetrySearchParams(1.2, 0.3, 0.3);
  pose_est->setAxisSymmetryMode();

  while(ros::ok()){
    PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
    pcl::copyPointCloud(*point_cloud_ptr, *original_cloud);
    PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
    PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr, 0, 3.5);
    PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);

    pose_est->setNewCloud(point_cloud_ptr);
    bool success = pose_est->process();
    *symmetry_cloud += *pose_est->getObjectCloud();
    ROS_INFO_STREAM(pose_est->get_cMo());

    int i = 0;
    while(success) {
      PCLTools<PointT>::moveToOrigin(pose_est->getObjectCloud());
      std::stringstream cloudid;
      cloudid << "cluster" << i++ << ".pcd";
      PCLTools<PointT>::cloudToPCD(pose_est->getObjectCloud(), cloudid.str() );
      //PCLView<PointT>::showCloudDuring(pose_est->getObjectCloud(), 500);
      success = ((PCAPoseEstimation *) pose_est)->processNext();
      if(success) *symmetry_cloud += *pose_est->getObjectCloud();
    }
    pcl::visualization::PCLVisualizer *p;
    p = new pcl::visualization::PCLVisualizer("SQ colored");
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h(point_cloud_ptr, 0, 230, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h(symmetry_cloud, 0, 0, 230);
    p->setBackgroundColor(0.8,0.8,0.8);
    p->addPointCloud(original_cloud, "c1");
    p->addPointCloud(symmetry_cloud, src_h, "c2");
    p->spin();
  }
  return 0;
}

