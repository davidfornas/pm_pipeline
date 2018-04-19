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

class SymmetryEstimation
{

protected:

  // Input, plane and output clouds. Plane Coeffs
  CloudPtr cloud_, plane_cloud_, mirrored_;
  pcl::ModelCoefficients plane_coeffs_;
  bool use_background_plane;

  // For visualization
  CloudPtr mirror_, projection_;

public:

  SymmetryEstimation(CloudPtr cloud, CloudPtr plane_cloud, pcl::ModelCoefficients plane_coeffs)
          : cloud_(cloud), plane_cloud_(plane_cloud), plane_coeffs_(plane_coeffs), use_background_plane(true)
  {
    mirrored_ = boost::shared_ptr<Cloud>(new Cloud());
    mirror_ = boost::shared_ptr<Cloud>(new Cloud());
    projection_ = boost::shared_ptr<Cloud>(new Cloud());

    pcl::copyPointCloud(*cloud_, *mirrored_);
  }

  SymmetryEstimation(CloudPtr cloud)
          : cloud_(cloud), use_background_plane(false)
  {
    mirrored_ = boost::shared_ptr<Cloud>(new Cloud());
    mirror_ = boost::shared_ptr<Cloud>(new Cloud());
    projection_ = boost::shared_ptr<Cloud>(new Cloud());

    pcl::copyPointCloud(*cloud_, *mirrored_);
  }

  /** Get the mirrored cloud and score  */
  double apply( CloudPtr & mirrored );

  /** Display the result color coded and centered */
  void display();

  /** Display the result*/
  void displayMirrored();

};


class PlaneSymmetryEstimation : public SymmetryEstimation
{
  Eigen::Vector3f plane_origin_, plane_normal_, reference_point_;

public:

  PlaneSymmetryEstimation(CloudPtr cloud, CloudPtr plane_cloud, pcl::ModelCoefficients plane_coeffs) :
          SymmetryEstimation(cloud, plane_cloud, plane_coeffs) {}

  PlaneSymmetryEstimation(CloudPtr cloud) : SymmetryEstimation(cloud) {}

  /** Get the mirrored cloud and score */
  double apply( CloudPtr & mirrored );

  

  //Obtain the plane in the middle of the furhtest point and the background
  void applyFurthest();

  //Obtain the plane in the middle of the centroid and the background
  void applyCentroid();

  void estimatePlane();

  void setPlane(Eigen::Vector3f plane_origin, Eigen::Vector3f plane_normal){
    plane_origin_ = plane_origin;
    plane_normal_ = plane_normal;
  }

};

class AxisSymmetryEstimation : public SymmetryEstimation
{
  Eigen::Vector3f line_origin_, line_direction_;

public:

  AxisSymmetryEstimation(CloudPtr cloud, CloudPtr plane_cloud, pcl::ModelCoefficients plane_coeffs) :
  SymmetryEstimation(cloud, plane_cloud, plane_coeffs) {}

  AxisSymmetryEstimation(CloudPtr cloud) : SymmetryEstimation(cloud) {}

  /** Get the mirrored cloud  */
  double apply( CloudPtr & mirrored );

  void setAxis(Eigen::Vector3f line_origin, Eigen::Vector3f line_direction){
    line_origin_ = line_origin;
    line_direction_ = line_direction;
  }

private:

  void estimateAxis();

};










#endif
