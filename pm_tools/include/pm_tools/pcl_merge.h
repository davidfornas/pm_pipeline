/*
 * pcl_merge Point cloud merging. NaN values are filled with non-NaN values,
 *           then non-NaN values are averaged in position. Color is taken
 *           from the first non-NaN value.
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

template <typename PointT>
class CloudMerge
{

  //Point counters for each possible point in an organized point cloud with 640 x 480 resolution.
  // @ TODO Implement variable resolution
  int coeffs[307200];
  float xvar[307200], yvar[307200], zvar[307200];

  /** Accumulate point b over a, taking care of NaN values. */
  PointT accumPoints(PointT a, PointT b, int idx);

public:

  /** Accumulate cloud b over a, filling gaps and averaging when necessary.  */
  void nanAwareOrganizedConcatenateMean(typename pcl::PointCloud<PointT>::Ptr a, typename pcl::PointCloud<PointT>::Ptr b);

};

template<typename PointT>
PointT CloudMerge<PointT>::accumPoints(PointT a, PointT b, int idx)
{
  PointT c(a);
  c.x = (a.x * coeffs[idx] + b.x) / (coeffs[idx] + 1);
  c.y = (a.y * coeffs[idx] + b.y) / (coeffs[idx] + 1);
  c.z = (a.z * coeffs[idx] + b.z) / (coeffs[idx] + 1);
  xvar[idx] += b.x * b.x;
  yvar[idx] += b.y * b.y;
  zvar[idx] += b.z * b.z;
  return c;
}

template<typename PointT>
void CloudMerge<PointT>::nanAwareOrganizedConcatenateMean(typename pcl::PointCloud<PointT>::Ptr a, typename pcl::PointCloud<PointT>::Ptr b)
{
  //A,B should be organized clouds of the same size. @ TODO Check and throw exception.
  for (size_t i = 0; i < a->points.size(); ++i){
    //First time a point is seen count=1;
    if(pcl::isFinite(a->points[i]) && coeffs[i] == 0){
      coeffs[i] = 1;
      xvar[i] = a->points[i].x * a->points[i].x;
      yvar[i] = a->points[i].y * a->points[i].y;
      zvar[i] = a->points[i].z * a->points[i].z;
    }
    if (pcl::isFinite(b->points[i])){
      if (!pcl::isFinite(a->points[i]))
      {
        //Point in B not found in A, add it.
        a->points[i] = b->points[i];
        //TODO: Search nearest neighbor color. Right now color is empty.
        xvar[i] = a->points[i].x * a->points[i].x;
        yvar[i] = a->points[i].y * a->points[i].y;
        zvar[i] = a->points[i].z * a->points[i].z;
        coeffs[i] = 1;
      }
      else
      {
        //Point found on both clouds. Weighted average.
        a->points[i] = accumPoints(a->points[i], b->points[i], i); //c+d;//=(a->points[i]*coeffs[i]+b->points[i])/(coeffs[i]+1);
        coeffs[i]++;
      }
    }
  }
}

#endif
