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

  /** RANSAC plane estimation */
  /*static pcl::ModelCoefficients::Ptr planeSegmentation(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
                                                pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
                                                pcl::PointCloud<PointT>::Ptr, double distanceThreshold = 0.03, int iterations = 100);*/

  /** RANSAC cylinder estimation */
  static pcl::ModelCoefficients::Ptr cylinderSegmentation(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
                                                   pcl::PointCloud<PointT>::Ptr, double distanceThreshold = 0.05,
                                                   int iterations = 20000, double rlimit = 0.1);

  /** Show segmented cloud and plane by coefficients and inliers */
  static void showClouds(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr, pcl::ModelCoefficients::Ptr, pcl::ModelCoefficients::Ptr);

  /** Count number of NaN values present in a point cloud. */
  static int nanCount(pcl::PointCloud<PointT>::Ptr);

  /** Fill gaps in cloud A using B.  */
  static void mergeOrganizedClouds(pcl::PointCloud<PointT>::Ptr a, pcl::PointCloud<PointT>::Ptr b);

};


class CloudMerge{

  //Point counters for each possible point in an organized point cloud with standard resolution.
  int coeffs[307200];
  float  xvar[307200], yvar[307200], zvar[307200];

  /** Accumulate point b over a, taking care of NaN values. */
  pcl::PointXYZRGB accumPoints(pcl::PointXYZRGB a, pcl::PointXYZRGB b, int idx);

public:

  /** Accumulate cloud b over a, filling gaps and averaging when necessary.  */
  void nanAwareOrganizedConcatenateMean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr a, pcl::PointCloud<pcl::PointXYZRGB>::Ptr b);

};

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
