/*
 * pcl_tools Point cloud processing tools using PCL
 *  Created on: 03/03/2015
 *      Author: dfornas
 */
#ifndef PCLTOOLS_H_
#define PCLTOOLS_H_

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
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/kdtree.h>

/** Usually using color 3D point clouds. B&W clouds are represented by RGB too */
typedef pcl::PointXYZRGB PointT;
//Should make pcl::PCLPointCloud2 version if it is interesting

class PCLTools
{

public:

  static void cloudFromPCD(pcl::PointCloud<PointT>::Ptr cloud, std::string fileName);

  static void cloudFromTopic(pcl::PointCloud<PointT>::Ptr cloud, std::string topicName);

  /** Pass through fileter in  */
  static void applyZAxisPassthrough(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out, double min, double max);

  /** Statistical Outlier Removal filter */
  static void applyStatisticalOutlierRemoval(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out);

  /** Voxel Grid filter filter */
  static void applyVoxelGridFilter(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out);

  /** Compute normals */
  static void estimateNormals(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

  ///@TODO Make classes for this more complex operations.

  /** RANSAC plane estimation */
  static pcl::ModelCoefficients::Ptr planeSegmentation(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
                                                pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
                                                pcl::PointCloud<PointT>::Ptr, double distanceThreshold = 0.03, int iterations = 100);

  /** RANSAC cylinder estimation */
  static pcl::ModelCoefficients::Ptr cylinderSegmentation(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
                                                   pcl::PointCloud<PointT>::Ptr, double distanceThreshold = 0.05,
                                                   int iterations = 20000, double rlimit = 0.1);

  /** Show segmented cloud and plane by coefficients and inliers */
  static void showClouds(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr, pcl::ModelCoefficients::Ptr, pcl::ModelCoefficients::Ptr);

};

#endif
