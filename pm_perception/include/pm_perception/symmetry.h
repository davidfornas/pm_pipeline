/*
 * symmetry This is used to obtain a full cloud of an object using a symmetry plane
 *  Created on: 28/03/2018
 *      Author: dfornas
 */
#ifndef PCLSYMMETRY_H_
#define PCLSYMMETRY_H_

#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>
#include <pm_perception/cluster_measure.h>

#include <pcl/console/parse.h>
#include <pcl/common/geometry.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

class MirrorCloud
{
  // Input and output
  CloudPtr cloud_, mirrored_;
  //For visualization
  CloudPtr mirror_, projection_;

  Eigen::Vector3f plane_origin_, plane_normal_;

public:

  MirrorCloud(CloudPtr in_cloud, Eigen::Vector3f plane_origin, Eigen::Vector3f plane_normal) :
          cloud_(in_cloud), plane_origin_(plane_origin), plane_normal_(plane_normal)
  {
    mirrored_ = boost::shared_ptr<Cloud>(new Cloud());
    mirror_ = boost::shared_ptr<Cloud>(new Cloud());
    projection_ = boost::shared_ptr<Cloud>(new Cloud());

    pcl::copyPointCloud(*cloud_, *mirrored_);
  }

  /** Get the mirrored cloud  */
  void apply( CloudPtr & mirrored );

  /** Display the result color coded and centered */
  void display();

  /** Display the result*/
  void displayMirrored();

};

class SymmetryPlaneEstimation
{
  CloudPtr cloud_;

public:

  SymmetryPlaneEstimation(CloudPtr cloud) : cloud_(cloud){

  }


};



#endif
