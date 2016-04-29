/** 
 * This program is used to filter and modify point clouds easily.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <iostream>
#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CPtr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  if(argc < 4 || argc > 8){
	  std::cerr << "rosrun pm_examples process_cloud <filename> <passMinZ> <passMaxZ> <meanK> <stdThresh> <leafSize> <planeThr>" << std::endl;
	  std::cerr << "Example: <filename> 0 2 50 1.0 0.03 0.06 <meanK> <stdThresh> <leafSize> <planeThr>" << std::endl;
	  return 0;
  }

  CPtr cloud(new Cloud);
  PCLTools<PointType>::cloudFromPCD(cloud, std::string(argv[1]) + std::string(".pcd"));
  PCLTools<PointType>::applyZAxisPassthrough(cloud, atof(argv[2]), atof(argv[3]));
  if(argc > 4) PCLTools<PointType>::applyStatisticalOutlierRemoval(cloud, atoi(argv[4]), atof(argv[5]));
  if(argc > 6) PCLTools<PointType>::applyVoxelGridFilter(cloud, atof(argv[6]));
  if(argc > 7) PlaneSegmentation<PointType>::removeBackground(cloud, 100, atof(argv[7]));

  PCLTools<PointType>::cloudToPCD(cloud, std::string(argv[1]) + std::string("_processed.pcd"));

  return (0);
}



