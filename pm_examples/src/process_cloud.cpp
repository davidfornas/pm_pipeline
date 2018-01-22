/** 
 * This program is used to filter and modify point clouds easily.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <iostream>
#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>

#include <pcl/filters/fast_bilateral.h>

#include <pcl/console/parse.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CPtr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  if(argc < 4){
	  std::cerr << "rosrun pm_examples process_cloud -f <filename> -p <passMinZ> <passMaxZ>  -r <meanK> <stdThresh>  -d <leafSize>  -s <planeThr>" << std::endl;
      std::cerr << "Other options: -b <billateralFilter> 15,0.05" << std::endl;
	  std::cerr << "Example: -f <filename> -p 0,2 -r 50,1.0 -d 0.03 -s 0.06 -> result in <filename>_processed.pcd" << std::endl;	  std::cerr << "rosrun pm_examples process_cloud -f <filename> -p <passMinZ> <passMaxZ>  -r <meanK> <stdThresh>  -d <leafSize>  -s <planeThr>" << std::endl;
	  std::cerr << "Example: Also with -t <topic> instead." << std::endl;
	  return 0;
  }
  std::string source("");
  float passthroughMinZ = 0, passthroughMaxZ = 0, outRemMeanK = 0, outRemStdTh = 0, leafSize = 0, planeTh = 0;
  float bilateralSigmaS = 0.0, bilateralSigmaR = 0.0;

  CPtr cloud(new Cloud);


  if (pcl::console::find_argument (argc, argv, "-f") > 0){
	  pcl::console::parse_argument (argc, argv, "-f", source);
	  PCLTools<PointT>::cloudFromPCD(cloud, source + std::string(".pcd"));
      std::cerr << "LOADED FROM FILE" << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-t") > 0){
	  pcl::console::parse_argument (argc, argv, "-t", source);
	  PCLTools<PointT>::cloudFromTopic(cloud, source); // From UWSim
	  std::cerr << "LOADED FROM TOPIC" << std::endl;
  }


  pcl::console::parse_2x_arguments (argc, argv, "-p", passthroughMinZ, passthroughMaxZ);
  pcl::console::parse_2x_arguments (argc, argv, "-r", outRemMeanK, outRemStdTh);
  pcl::console::parse_argument (argc, argv, "-d", leafSize);
  pcl::console::parse_argument (argc, argv, "-s", planeTh);
  pcl::console::parse_2x_arguments (argc, argv, "-b", bilateralSigmaS, bilateralSigmaR);

  //REMOVE NaN if needed
  PCLTools<PointT>::removeNanPoints(cloud);

  if(passthroughMinZ != 0 && passthroughMaxZ != 0) 	PCLTools<PointT>::applyZAxisPassthrough(cloud, passthroughMinZ, passthroughMaxZ);
  if(outRemMeanK != 0 && outRemStdTh != 0) 			PCLTools<PointT>::applyStatisticalOutlierRemoval(cloud, outRemMeanK, outRemStdTh);
  if(leafSize != 0)  	PCLTools<PointT>::applyVoxelGridFilter(cloud, leafSize);
  if(planeTh != 0) 		PlaneSegmentation<PointT>::removeBackground(cloud, 100, planeTh);

  PCLTools<PointT>::cloudToPCD(cloud, /*std::string() +*/ std::string("_preprocessed.pcd"));
  if(bilateralSigmaS != 0){
    pcl::FastBilateralFilter<PointT> filter;
    filter.setSigmaS( bilateralSigmaS );
    filter.setSigmaR( bilateralSigmaR );
    filter.applyFilter(*cloud);
  }

  PCLTools<PointT>::cloudToPCD(cloud, /*std::string() +*/ std::string("_processed.pcd"));

  return (0);
}



