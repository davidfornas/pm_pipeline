/** 
 * This program is used to register a pair of point clouds.
 *  Created on: 25/01/2016
 *      Author: dfornas
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/common/transforms.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main (int argc, char** argv)
{
  //Load and remove NaN from clouds.
	PointCloud::Ptr cloud1 (new PointCloud), cloud1_filtered (new PointCloud), cloud2 (new PointCloud), cloud2_filtered (new PointCloud),
                                         cloud1_aux (new PointCloud), cloud2_aux (new PointCloud), cloud1_transformed(new PointCloud);

  PCLTools<PointT>::cloudFromPCD(cloud1, std::string(argv[1]) + std::string(".pcd"));
  PCLTools<PointT>::cloudFromPCD(cloud2, std::string(argv[2]) + std::string(".pcd"));
  
  PCLTools<PointT>::removeNanPoints(cloud1, cloud1_aux);
  PCLTools<PointT>::removeNanPoints(cloud2, cloud2_aux);

  PlaneSegmentation<PointT>::removeBackground(cloud1_aux, cloud1_filtered, 100, 0.06);
  PlaneSegmentation<PointT>::removeBackground(cloud2_aux, cloud2_filtered, 100, 0.06);


  //Perform ICP
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(200);
  icp.setRANSACIterations(10000);
  icp.setRANSACOutlierRejectionThreshold(0.05);
  icp.setMaxCorrespondenceDistance(0.04);//1best 0.05+0.04; 2best 0.1+0.05
  //TODO: FInde better/worse values
  icp.setInputSource(cloud1_filtered);
  icp.setInputTarget(cloud2_filtered);
  pcl::PointCloud<PointT> Final;
  icp.align(Final);
  std::cout << "Has converged:" << icp.hasConverged() << " score!: " <<
  icp.getFitnessScore() << std::endl;
  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  std::cout << icp.getFinalTransformation() << std::endl;

  // ----  VISUALIZATION  ---
  // -----Open 3D viewer and add point clouds-----
  pcl::visualization::PCLVisualizer viewer("ICP Registration");
  viewer.setBackgroundColor(1, 1, 1);

  //Remove far points
  PointCloud::Ptr cloud1_vis (new PointCloud), cloud2_vis (new PointCloud);
  PCLTools<PointT>::applyZAxisPassthrough(cloud2, cloud2_vis, -2, 2);
  PCLTools<PointT>::applyZAxisPassthrough(cloud1, cloud1_vis, -2, 2);

  //Show cloud 2. Filtered or not

  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler(cloud2_filtered, 250, 100, 0);
  //viewer.addPointCloud(cloud2_filtered,  "target");

  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler(cloud2_vis, 250, 100, 0);
  viewer.addPointCloud(cloud2_vis,  "target");

  //Show cloud 1 transformed to target. Filtered or not
  pcl::transformPointCloud (*cloud1_vis, *cloud1_transformed, transformation);
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler2(cloud1_transformed, 100, 250, 0);
  viewer.addPointCloud(cloud1_transformed, "origin_transformed");

  std::cerr<<"REBUILD";

  //Final should be the same
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_ptr(&Final);
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler2(final_ptr, 100, 250, 0);
  //viewer.addPointCloud(final_ptr, "taget");
  //PCLTools::cloudToPCD(final_ptr, std::string("result.pcd"));

  // -----Main loop-----
  long long path_counter = 0;
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }

  return (0);
}

