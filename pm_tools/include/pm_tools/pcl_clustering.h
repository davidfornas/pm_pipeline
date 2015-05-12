/*
 * pcl_clustering Description
 *  Created on: 12/05/2015
 *      Author: dfornas
 */
#ifndef PCLCLUSTERING_H_
#define PCLCLUSTERING_H_

#include <pm_tools/pcl_tools.h>
/*
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
*/

/** Usually using color 3D point clouds. B&W clouds are represented by RGB too */
typedef pcl::PointXYZRGB PointT;
// @ TODO pcl::PCLPointCloud2 versions if it is interesting

class CloudClustering
{

public:

  /** Apply clustering  */
  void apply();

  /** Display result  */
  void display();

};
#endif
