/*
 * pcl_tools Point cloud processing tools using PCL
 *  Created on: 03/03/2015
 *      Author: dfornas
 */
#ifndef PCLTOOLS_H_
#define PCLTOOLS_H_
/*
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
*/
#include <pcl/io/pcd_io.h>

/** Usually using color 3D point clouds. */
//typedef pcl::PointXYZRGB PointT;

namespace pm_pcl_tools{


  void PCLoader(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, std::string name, bool fromFile);//Load from PCDReader or from topic
  void PassthroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, double x, double y, double z);
///** Pass through fileter */
//void passThrough(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr, double, double);
///** Statistical Outlier Removal filter */
//void statisticalOutlierRemoval();
///** Voxel Grid filter filter */
//void voxelGridFilter();
///** Compute normals */
//void estimateNormals(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
///** RANSAC plane estimation */
//pcl::ModelCoefficients::Ptr planeSegmentation(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
//                                              pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
//                                              pcl::PointCloud<PointT>::Ptr, double distanceThreshold = 0.03, int iterations = 100);
///** RANSAC cylinder estimation */
//pcl::ModelCoefficients::Ptr cylinderSegmentation(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
//                                                 pcl::PointCloud<PointT>::Ptr, double distanceThreshold = 0.05,
//                                                 int iterations = 20000, double rlimit = 0.1);
///** Show segmented cloud and plane by coefficients and inliers */
//void showClouds(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr, pcl::ModelCoefficients::Ptr, pcl::ModelCoefficients::Ptr);
}

#endif
