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
  
  PCLTools<PointT>::removeNanPoints(cloud1, cloud1_filtered);
  PCLTools<PointT>::removeNanPoints(cloud2, cloud2_filtered);

  //Perform ICP
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(200);
  icp.setRANSACIterations(10000);
  icp.setRANSACOutlierRejectionThreshold(0.05);
  icp.setMaxCorrespondenceDistance(0.03);//1best 0.05+0.04; 2best 0.1+0.05
  //icp.setTransformationEpsilon ((1e-6)*2);
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
  int vp_1, vp_2;
  viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
  viewer.setBackgroundColor(1, 1, 1);

  //Remove far points
  PointCloud::Ptr cloud1_vis (new PointCloud), cloud2_vis (new PointCloud);
  PCLTools<PointT>::applyZAxisPassthrough(cloud1, cloud2_vis, -2, 2);
  PCLTools<PointT>::applyZAxisPassthrough(cloud2, cloud1_vis, -2, 2);

  viewer.addPointCloud(cloud2_vis,  "target", vp_1);
  pcl::transformPointCloud (*cloud1_vis, *cloud1_transformed, transformation);
  viewer.addPointCloud(cloud1_transformed, "origin_transformed", vp_1);

  PointCloud::Ptr cloud1_vis2 (new PointCloud), cloud2_vis2 (new PointCloud);
  PCLTools<PointT>::applyZAxisPassthrough(cloud1_filtered, cloud2_vis2, -2, 2);
  PCLTools<PointT>::applyZAxisPassthrough(cloud2_filtered, cloud1_vis2, -2, 2);

  viewer.addPointCloud(cloud2_vis2,  "target2", vp_2);
  pcl::transformPointCloud (*cloud1_vis2, *cloud1_transformed, transformation);
  viewer.addPointCloud(cloud1_transformed, "origin_transformed2", vp_2);

  viewer.spin();

  return (0);
}

