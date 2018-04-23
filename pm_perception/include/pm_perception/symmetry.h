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

  bool debug;

  //Search parameters
  double angle_limit_, angle_step_, distance_ratio_step_;


public:

  SymmetryEstimation(CloudPtr cloud, CloudPtr plane_cloud, pcl::ModelCoefficients plane_coeffs)
          : cloud_(cloud), plane_cloud_(plane_cloud), plane_coeffs_(plane_coeffs), use_background_plane(true)
  {
    mirrored_ = boost::shared_ptr<Cloud>(new Cloud());
    mirror_ = boost::shared_ptr<Cloud>(new Cloud());
    projection_ = boost::shared_ptr<Cloud>(new Cloud());

    pcl::copyPointCloud(*cloud_, *mirrored_);
    setSearchParams();
  }

  SymmetryEstimation(CloudPtr cloud)
          : cloud_(cloud), use_background_plane(false)
  {
    mirrored_ = boost::shared_ptr<Cloud>(new Cloud());
    mirror_ = boost::shared_ptr<Cloud>(new Cloud());
    projection_ = boost::shared_ptr<Cloud>(new Cloud());

    pcl::copyPointCloud(*cloud_, *mirrored_);
    setSearchParams();
  }

  /** Get the mirrored cloud and score  */
  double apply( CloudPtr & mirrored );

  /** Display the result color coded and centered */
  void display();

  /** Display the result*/
  void displayMirrored();

  void setSearchParams(double angle_limit = 0.55, double angle_step = 0.04, double distance_ratio_step = 0.1){
    angle_limit_ = angle_limit;
    angle_step_ = angle_step;
    distance_ratio_step_ = distance_ratio_step;
  }

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

  /** Get the mirrored cloud and score */
  double searchBest( CloudPtr & mirrored, bool fixed_half_height = false );

  //Obtain the plane in the middle of the furhtest point and the background
  void applyFurthest();

  //Obtain the plane in the middle of the centroid and the background
  void applyCentroid();

  //Obtain the plane at d distance from the furthest in direction to the floor
  // with an angle rotation in x and y
  //void applyParameters( double distance_ratio, double x_angle, double y_angle );

  //Distance ratio is the distance between the furthest points and the background plane expressed in ratio. Normally 0.5.
  void estimatePlane( double distance_ratio = 0.5, double x = 0.0, double y = 0.0, double z = 0.0  );

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

  /** Get the mirrored cloud and score */
  double searchBest( CloudPtr & mirrored, bool fixed_half_height = false );

  /** Get the mirrored cloud  */
  double apply( CloudPtr & mirrored );

  void setAxis(Eigen::Vector3f line_origin, Eigen::Vector3f line_direction){
    line_origin_ = line_origin;
    line_direction_ = line_direction;
  }

private:

  //Distance ratio is the distance between the centroid and the background plane expressed in ratio. Normally 0.5.
  void estimateAxis( double distance_ratio = 0.0, double x = 0.0, double y = 0.0, double z = 0.0 );

};










#endif
