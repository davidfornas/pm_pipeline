/** 
 * Test GraspRankingPlanner
 *
 *  Created on: 29/03/2018
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <pm_manipulation/ranking_grasp_planner.h>

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "grasp_ranking_test");
  ros::NodeHandle nh;

  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>);
  std::string filename("/stereo/points2");
  PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, filename);
  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);
  PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0, 3.5);
  PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);


  RankingGraspPlanner rgp(point_cloud_ptr, nh);
  ROS_INFO("XXX");
  rgp.generateGraspList();
  ROS_INFO("Voy a mostrar la lista:");
  rgp.publishGraspList();
  ros::spinOnce();

  return 0;
}

