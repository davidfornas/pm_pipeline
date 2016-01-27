/** 
 * This program is used to filter and modify point clouds easily.
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

int main (int argc, char** argv)
{
  //Load and remove NaN from clouds.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>), cloud1_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
                                         cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>), cloud2_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
                                         cloud1_aux (new pcl::PointCloud<pcl::PointXYZRGB>), cloud2_aux (new pcl::PointCloud<pcl::PointXYZRGB>),
                                         cloud1_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

  PCLTools::cloudFromPCD(cloud1, std::string(argv[1]) + std::string(".pcd"));
  PCLTools::cloudFromPCD(cloud2, std::string(argv[2]) + std::string(".pcd"));
  
  PCLTools::removeNanPoints(cloud1, cloud1_aux);
  PCLTools::removeNanPoints(cloud2, cloud2_aux);

  PlaneSegmentation::removeBackground(cloud1_aux, cloud1_filtered, 100, 0.06);
  PlaneSegmentation::removeBackground(cloud2_aux, cloud2_filtered, 100, 0.06);


  //Perform ICP
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setMaximumIterations(200);
  icp.setRANSACIterations(10000);
  icp.setRANSACOutlierRejectionThreshold(0.05);
  icp.setMaxCorrespondenceDistance(0.04);//1best 0.05+0.04; 2best 0.1+0.05
  //TODO: FInde better/worse values
  icp.setInputSource(cloud1_filtered);
  icp.setInputTarget(cloud2_filtered);
  pcl::PointCloud<pcl::PointXYZRGB> Final;
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_vis (new pcl::PointCloud<pcl::PointXYZRGB>), cloud2_vis (new pcl::PointCloud<pcl::PointXYZRGB>);
  PCLTools::applyZAxisPassthrough(cloud2, cloud2_vis, -2, 2);
  PCLTools::applyZAxisPassthrough(cloud1, cloud1_vis, -2, 2);

  //Show cloud 2. Filtered or not

  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler(cloud2_filtered, 250, 100, 0);
  //viewer.addPointCloud(cloud2_filtered,  "target");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler(cloud2_vis, 250, 100, 0);
  viewer.addPointCloud(cloud2_vis,  "target");

  //Show cloud 1 transformet to target. Filtered or not
  pcl::transformPointCloud (*cloud1_vis, *cloud1_transformed, transformation);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler2(cloud1_transformed, 100, 250, 0);
  viewer.addPointCloud(cloud1_transformed, "origin_transformed");

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

