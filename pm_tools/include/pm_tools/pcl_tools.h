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

// @ TODO Write binary pcl::PCLPointCloud2 versions if it is interesting

class PCLTools
{

public:

  /** Init cloud from a PCD file  */
  static void cloudFromPCD(pcl::PointCloud<PointT>::Ptr cloud, std::string fileName);

  /** Init cloud from a ROS topic  */
  static void cloudFromTopic(pcl::PointCloud<PointT>::Ptr cloud, std::string topicName);

  /** Pass through filter in  Z AXIS. @ TODO XY axis */
  static void applyZAxisPassthrough(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out, double min, double max);

  /** Statistical Outlier Removal filter */
  static void applyStatisticalOutlierRemoval(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out);

  /** Voxel grid filter */
  static void applyVoxelGridFilter(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out);

  /** Compute point cloud normals */
  static void estimateNormals(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

  /** Display segmented cloud and plane by coefficients and inliers */
  static void showClouds(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr, pcl::ModelCoefficients::Ptr, pcl::ModelCoefficients::Ptr);

  /** Count number of NaN values present in a point cloud */
  static int nanCount(pcl::PointCloud<PointT>::Ptr);

  /** Fill gaps in cloud A using B  */
  static void mergeOrganizedClouds(pcl::PointCloud<PointT>::Ptr a, pcl::PointCloud<PointT>::Ptr b);

};

#endif
