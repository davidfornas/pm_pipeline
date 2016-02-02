/*
 * pcl_segmentation Point cloud segmentation using RANSAC. Cylinder model and plane model.
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

template <typename PointT>
class PlaneSegmentation
{
  typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

  CloudPtr in_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr in_normals_;

  double distance_threshold_;
  int num_iterations_;

public:

  PlaneSegmentation(CloudPtr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr in_normals) :
      in_cloud_(in_cloud), in_normals_(in_normals)
  {
    //Default values
    distance_threshold_ = 0.05;
    num_iterations_ = 100;
  }

  /** Apply segmentation **/
  bool apply(CloudPtr out_cloud, pcl::PointCloud<pcl::Normal>::Ptr out_normals,
		  CloudPtr cloud_plane, pcl::ModelCoefficients::Ptr coeffs);

  /** Set number of iterations for the RANSAC method **/
  void setIterations(int iterations)
  {
    num_iterations_ = iterations;
  }

  /** Set the distance threshold for the inliers for the RANSAC method **/
  void setDistanceThreshold(double threshold)
  {
    distance_threshold_ = threshold;
  }

  ~PlaneSegmentation()
  {
  }

  /** Remove the plane from the cloud, easy to use method.  */
  static void removeBackground(CloudPtr in, CloudPtr out, int iteration=100, double threshold=0.06);

};

template <typename PointT>
class CylinderSegmentation
{

  typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

  CloudPtr in_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr in_normals_;
  pcl::PointIndices::Ptr inliers_cylinder_;

  double distance_threshold_, radious_limit_;
  int num_iterations_;

public:
  CylinderSegmentation(CloudPtr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr in_normals) :
      in_cloud_(in_cloud), in_normals_(in_normals)
  {
    //Default values
    distance_threshold_ = 0.05;
    num_iterations_ = 100;
    radious_limit_ = 0.05;
    inliers_cylinder_ = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices);
  }
  bool apply(CloudPtr cloud_cylinder, pcl::ModelCoefficients::Ptr coeffs);
  void setIterations(int iterations)
  {
    num_iterations_ = iterations;
  }
  void setDistanceThreshold(double threshold)
  {
    distance_threshold_ = threshold;
  }
  void setRadiousLimit(double radious)
  {
    radious_limit_ = radious;
  }
  void getInliers(pcl::PointIndices::Ptr & inliers)
  {
    inliers = inliers_cylinder_;
  }
  ~CylinderSegmentation()
  {
  }
};

/** Remove the plane from the cloud, easy to use method. ZPassthrough used.  */
template<typename PointT>
void PlaneSegmentation<PointT>::removeBackground(CloudPtr in, CloudPtr out, int iterations, double threshold){

	CloudPtr plane(new typename pcl::PointCloud<PointT>), in_filtered(new typename pcl::PointCloud<PointT>);

	  pcl::PointCloud<pcl::Normal>::Ptr in_cloud_normals (new pcl::PointCloud<pcl::Normal>), out_cloud_normals (new pcl::PointCloud<pcl::Normal>);
	  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

	  // TODO: This filter may be applied externaly or optionaly.
	  PCLTools<PointT>::applyZAxisPassthrough(in, in_filtered, -2, 2);
	  PCLTools<PointT>::estimateNormals(in_filtered, in_cloud_normals);

	  PlaneSegmentation<PointT> plane_seg(in_filtered, in_cloud_normals);
	  plane_seg.setDistanceThreshold(threshold);
	  plane_seg.setIterations(iterations);
	  plane_seg.apply(out, out_cloud_normals, plane, coefficients_plane);

}

/** RANSAC cylinder estimation */
template<typename PointT>
bool CylinderSegmentation<PointT>::apply(CloudPtr cloud_cylinder, pcl::ModelCoefficients::Ptr coeffs)
{

  clock_t begin = clock();
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  typename pcl::ExtractIndices<PointT> extract;
  pcl::PCDWriter writer;
  typename pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(num_iterations_); //10000
  seg.setDistanceThreshold(distance_threshold_); //0.05
  seg.setRadiusLimits(0, radious_limit_); //0, 0.1
  seg.setInputCloud(in_cloud_);
  seg.setInputNormals(in_normals_);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder_, *coeffs);
  ROS_INFO_STREAM("Cylinder coefficients: " << *coeffs);
  clock_t end = clock();
  ROS_DEBUG_STREAM("Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC);

  // Write the cylinder inlier1s to disk
  extract.setInputCloud(in_cloud_);
  extract.setIndices(inliers_cylinder_);
  extract.setNegative(false);
  extract.filter(*cloud_cylinder);
  if (cloud_cylinder->points.empty())
    ROS_DEBUG_STREAM("Can't find the cylindrical component.");
  else
  {
    ROS_INFO_STREAM(
        "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points.");
    writer.write("/home/dfornas/data/scene_cylinder.pcd", *cloud_cylinder, false);
  }
  return true;
}

#endif
