/** 
 * This program is used to filter and modify point clouds easily.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <iostream>

#include <pm_tools/pcl_tools.h>

#include <pm_tools/pcl_segmentation.h>
#include <ros/ros.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CPtr;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  if(argc!=5){
	  std::cerr << "rosrun pm_examples process cloud <filename> <passMinZ> <passMaxZ> <planeThr>" << std::endl;
	  return 0;
  }

  //Load cloud...
  CPtr cloud(new Cloud);
  PCLTools<PointType>::cloudFromPCD(cloud, std::string(argv[1]));
  PCLTools<PointType>::applyZAxisPassthrough(cloud, atof(argv[2]), atof(argv[3]));
  PCLTools<PointType>::applyStatisticalOutlierRemoval(cloud);
  PCLTools<PointType>::applyVoxelGridFilter(cloud, 0.03);

  //Outlier rejection...

  //Voxel grid...

  //Segmentation
  PlaneSegmentation<PointType>::removeBackground(cloud, 100, 0.08);

  PCLTools<PointType>::cloudToPCD(cloud, std::string(argv[1]) + std::string("_processed.pcd"));

  return (0);
}



