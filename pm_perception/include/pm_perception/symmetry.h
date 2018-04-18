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

protected:

  // Input and output
  CloudPtr cloud_, mirrored_;
  //For visualization
  CloudPtr mirror_, projection_;


public:

  MirrorCloud(CloudPtr in_cloud) : cloud_(in_cloud)
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


class PlaneMirrorCloud : public MirrorCloud
{
  Eigen::Vector3f plane_origin_, plane_normal_;

public:

  PlaneMirrorCloud(CloudPtr in_cloud, Eigen::Vector3f plane_origin, Eigen::Vector3f plane_normal) :
          MirrorCloud(in_cloud), plane_origin_(plane_origin), plane_normal_(plane_normal)
  {
  }

  /** Get the mirrored cloud  */
  void apply( CloudPtr & mirrored );

private:

  double score();

};

class AxisMirrorCloud : public MirrorCloud
{
  Eigen::Vector3f line_origin_, line_direction_;

public:

  AxisMirrorCloud(CloudPtr in_cloud, Eigen::Vector3f line_origin, Eigen::Vector3f line_direction) :
          MirrorCloud(in_cloud), line_origin_(line_origin), line_direction_(line_direction)
  {
  }

  /** Get the mirrored cloud  */
  void apply( CloudPtr & mirrored );

};

class SymmetryEstimation
{
protected:

  CloudPtr cloud_, plane_cloud_;
  pcl::ModelCoefficients plane_coeffs_;

public:
  SymmetryEstimation(CloudPtr cloud, CloudPtr plane_cloud, pcl::ModelCoefficients plane_coeffs)
          : cloud_(cloud), plane_cloud_(plane_cloud), plane_coeffs_(plane_coeffs){
  }

};

class SymmetryPlaneEstimation : public SymmetryEstimation
{

public:
  SymmetryPlaneEstimation(CloudPtr cloud, CloudPtr plane_cloud, pcl::ModelCoefficients plane_coeffs)
          : SymmetryEstimation(cloud, plane_cloud, plane_coeffs){
  }

  //Obtain the plane in the middle of the furhtest point and the background
  void applyFurthest(Eigen::Vector3f & plane_origin_out, Eigen::Vector3f & plane_normal_out);

  //Obtain the plane in the middle of the centroid and the background
  void applyCentroid(Eigen::Vector3f & plane_origin_out, Eigen::Vector3f & plane_normal_out);

private:

  void apply(Eigen::Vector3f & plane_origin_out, Eigen::Vector3f & plane_normal_out, Eigen::Vector3f reference_point);

};

class SymmetryAxisEstimation : public SymmetryEstimation
{

public:

  SymmetryAxisEstimation(CloudPtr cloud, CloudPtr plane_cloud, pcl::ModelCoefficients plane_coeffs)
          : SymmetryEstimation(cloud, plane_cloud, plane_coeffs){
  }

  void apply(Eigen::Vector3f & axis_origin_out, Eigen::Vector3f & axis_dir_out);

};





#endif
