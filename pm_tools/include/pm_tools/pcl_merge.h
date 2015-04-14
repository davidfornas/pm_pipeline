/*
 * pcl_merge Point cloud processing tools using PCL
 *  Created on: 14/04/2015
 *      Author: dfornas
 */
#ifndef PCLMERGE_H_
#define PCLMERGE_H_

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

#endif
