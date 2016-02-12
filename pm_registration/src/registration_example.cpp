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

typedef pcl::PointXYZ PointT;
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

  icp.setRANSACIterations(1000);
  icp.setRANSACOutlierRejectionThreshold(0.03);
  //icp.setUseReciprocalCorrespondences(true);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.2);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (3000);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1e-6);

  icp.setInputSource(cloud1_filtered);
  icp.setInputTarget(cloud2_filtered);
  PointCloud Final;
  icp.align(Final);
  std::cout << "Has converged:" << icp.hasConverged() << " score!: " <<
  icp.getFitnessScore() << std::endl;
  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  std::cout << transformation << std::endl;

  // ----  VISUALIZATION  ---
  // -----Open 3D viewer and add point clouds-----
  pcl::visualization::PCLVisualizer viewer("ICP Registration");

  viewer.addPointCloud(cloud2,  "target");//, vp_1);
  pcl::transformPointCloud (*cloud1, *cloud1_transformed, transformation);
  viewer.addPointCloud(cloud1_transformed, "origin_transformed");//, vp_1);

  viewer.spin();

  return (0);
}

