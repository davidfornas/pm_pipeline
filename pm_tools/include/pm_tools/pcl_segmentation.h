/*
 * pcl_segmentation Point cloud processing tools using PCL
 *  Created on: 03/03/2015
 *      Author: dfornas
 */
#ifndef PCLSEGMENTATION_H_
#define PCLSEGMENTATION_H_

#include <pm_tools/pcl_tools.h>

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

class PlaneSegmentation {
  pcl::PointCloud<PointT>::Ptr in_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr in_normals_;
  double distance_threshold_;//@ TODO set defaults...
  int num_iterations_;
public:
  PlaneSegmentation(pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr in_normals) : in_cloud_(in_cloud), in_normals_(in_normals) {
    distance_threshold_=0.05;
    num_iterations_=100;
  }
  bool apply(pcl::PointCloud<PointT>::Ptr out_cloud, pcl::PointCloud<pcl::Normal>::Ptr out_normals, pcl::PointCloud<PointT>::Ptr cloud_plane, pcl::ModelCoefficients::Ptr coeffs);
  void setIterations( int iterations ){ num_iterations_ = iterations; }
  void setDistanceThreshold( double threshold ){ distance_threshold_ = threshold; }
  ~PlaneSegmentation(){}
};

class CylinderSegmentation {
  pcl::PointCloud<PointT>::Ptr in_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr in_normals_;
  pcl::PointIndices::Ptr inliers_cylinder_;
  double distance_threshold_, radious_limit_;//@ TODO set defaults...
  int num_iterations_;

public:
  CylinderSegmentation(pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr in_normals) : in_cloud_(in_cloud), in_normals_(in_normals) {
    distance_threshold_=0.05;
    num_iterations_=100;
    radious_limit_=0.05;
    inliers_cylinder_=boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices);
  }
  bool apply(pcl::PointCloud<PointT>::Ptr cloud_cylinder, pcl::ModelCoefficients::Ptr coeffs);
  void setIterations( int iterations ){ num_iterations_ = iterations; }
  void setDistanceThreshold( double threshold ){ distance_threshold_ = threshold; }
  void setRadiousLimit( double radious ){ radious_limit_ = radious; }
  void getInliers( pcl::PointIndices::Ptr & inliers ){ inliers = inliers_cylinder_; }
  ~CylinderSegmentation(){}
};

#endif
