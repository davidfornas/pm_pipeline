/** 
 * This program is used to filter and modify point clouds easily.
 * @TODO Move bilateral to pm_tools
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <iostream>
#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>
#include <pcl/filters/fast_bilateral.h>

#include <pcl/console/parse.h>
#include "../../pm_registration/include/pm_registration/marker_registration.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CPtr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  if(argc < 4){
	  std::cerr << "rosrun pm_apps process_cloud -f <filename> -p <passMinZ> <passMaxZ>  -r <meanK> <stdThresh>  -d <leafSize>  -s <planeThr>" << std::endl;
      std::cerr << "-b <billateralFilter> 15,0.05  " << std::endl;
      std::cerr << "Options: -v Visualization -s Save -n View Normals -nan Remove NaN" << std::endl << std::endl;
	  std::cerr << "Example: -f <filename> -p 0,2 -r 50,1.0 -d 0.03 -s 0.06 -> result in <filename>_processed.pcd" << std::endl;	  std::cerr << "rosrun pm_apps process_cloud -f <filename> -p <passMinZ> <passMaxZ>  -r <meanK> <stdThresh>  -d <leafSize>  -s <planeThr>" << std::endl;
	  std::cerr << "Load cloud from topic with -t <topic> instead." << std::endl;
	  return 0;
  }
  std::string source("");
  float passthroughMinZ = 0.0, passthroughMaxZ = 0.0, outRemMeanK = 0.0, outRemStdTh = 0.0;
  float leafSize = 0.0, planeTh = 0.0, bilateralSigmaS = 0.0, bilateralSigmaR = 0.0;
  int normalSampligGrid = 0, normalSamplingSamples = 0;

  CPtr cloud(new Cloud), original(new Cloud);

  if (pcl::console::find_argument (argc, argv, "-f") > 0){
	  pcl::console::parse_argument (argc, argv, "-f", source);
	  PCLTools<PointT>::cloudFromPCD(cloud, source + std::string(".pcd"));
      std::cerr << "Cloud loaded from file. " << std::endl;
  }else if (pcl::console::find_argument (argc, argv, "-t") > 0){
	  pcl::console::parse_argument (argc, argv, "-t", source);
	  PCLTools<PointT>::cloudFromTopic(cloud, source);
      source = "topic";
	  std::cerr << "Cloud loaded from topic. " << std::endl;
  }else{
    return 0;
  }
  original = cloud;

  pcl::console::parse_2x_arguments (argc, argv, "-p", passthroughMinZ, passthroughMaxZ);
  pcl::console::parse_2x_arguments (argc, argv, "-r", outRemMeanK, outRemStdTh);
  pcl::console::parse_argument (argc, argv, "-d", leafSize);
  pcl::console::parse_argument (argc, argv, "-s", planeTh);
  pcl::console::parse_2x_arguments (argc, argv, "-b", bilateralSigmaS, bilateralSigmaR);
  pcl::console::parse_2x_arguments (argc, argv, "-ns", normalSampligGrid, normalSamplingSamples);

  // @TODO This should be optional
  if(pcl::console::find_argument (argc, argv, "-nan") > 0) PCLTools<PointT>::removeNanPoints(cloud);

  if(pcl::console::find_argument (argc, argv, "-p") > 0) PCLTools<PointT>::applyZAxisPassthrough(cloud, passthroughMinZ, passthroughMaxZ);

  if(outRemMeanK != 0 && outRemStdTh != 0) PCLTools<PointT>::applyStatisticalOutlierRemoval(cloud, outRemMeanK, outRemStdTh);

  if(bilateralSigmaS != 0) PCLTools<PointT>::apllyFastBilateralFilter(cloud, bilateralSigmaS, bilateralSigmaR);

  if(leafSize != 0) PCLTools<PointT>::applyVoxelGridFilter(cloud, leafSize);

  if(planeTh != 0) PlaneSegmentation<PointT>::removeBackground(cloud, 100, planeTh);


  if (pcl::console::find_argument (argc, argv, "-s") > 0) {
    std::cerr << "Cloud save with name " << source + std::string("_processed.pcd") << std::endl;
    PCLTools<PointT>::cloudToPCD(cloud, source + std::string("_processed.pcd"));
  }

  if (pcl::console::find_argument (argc, argv, "-v") > 0){
    std::cerr << "Visualizing point cloud..." << std::endl;
    pcl::visualization::PCLVisualizer *p;
    int vp_1, vp_2;
    p = new pcl::visualization::PCLVisualizer (argc, argv, "Simple cloud filtering.");
    p->createViewPort (0.0, 0.0, 0.5, 1.0, vp_1);
    p->createViewPort (0.5, 0.0, 1.0, 1.0, vp_2);
    p->addPointCloud (original, "c1", vp_1);
    p->addPointCloud (cloud, "c2", vp_2);
    p->spin();
  }

  if(pcl::console::find_argument (argc, argv, "-ns") > 0){
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    PCLTools<PointT>::estimateNormals(cloud, cloud_normals);
    CloudPtr new_cloud(new Cloud);
    PCLTools<PointT>::normalSpaceSampling(cloud, cloud_normals, new_cloud, normalSampligGrid, normalSamplingSamples);
    cloud = new_cloud;
  }

  //Show clouds with normals.
  if (pcl::console::find_argument (argc, argv, "-n") > 0){
    std::cerr << "Visualizing point cloud with normals..." << std::endl;

    pcl::PointCloud<pcl::PointNormal>::Ptr original_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    PCLTools<PointT>::estimateNormals(original, original_normals);
    PCLTools<PointT>::estimateNormals(cloud, cloud_normals);

    pcl::visualization::PCLVisualizer *p;
    int vp_1, vp_2;
    p = new pcl::visualization::PCLVisualizer (argc, argv, "Simple cloud filtering. Normals viewer.");
    p->createViewPort (0.0, 0.0, 0.5, 1.0, vp_1);
    p->createViewPort (0.5, 0.0, 1.0, 1.0, vp_2);
    p->addPointCloud (original, "c1", vp_1);
    p->addPointCloudNormals<PointT, pcl::PointNormal>(original, original_normals, 200, 0.03, "n1", vp_1);
    p->addPointCloud (cloud, "c2", vp_2);
    p->addPointCloudNormals<PointT, pcl::PointNormal>(cloud, cloud_normals, 200, 0.03, "n2", vp_2);
    p->spin();
  }

  return (0);
}


