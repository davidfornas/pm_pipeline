/*
 * pcl_clustering Description
 *  Created on: 12/05/2015
 *      Author: dfornas
 */
#ifndef PCLCLUSTERING_H_
#define PCLCLUSTERING_H_


#include <pm_tools/pcl_tools.h>
#include <pcl/point_types.h>

/** Usually using color 3D point clouds. B&W clouds are represented by RGB too */
typedef pcl::PointXYZRGB PointT;
// @ TODO pcl::PCLPointCloud2 versions if it is interesting

class CloudClustering
{

  pcl::PointCloud<PointT>::Ptr in_cloud_;

public:

  CloudClustering(pcl::PointCloud<PointT>::Ptr in_cloud) :
      in_cloud_(in_cloud)
  {
  }

  /** Apply clustering  */
  void apply();

  /** Display result  */
  void display();

};
#endif
