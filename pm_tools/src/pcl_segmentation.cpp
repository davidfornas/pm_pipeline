/*
 * pcl_tools
 * 			 
 *  Created on: 03/03/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_segmentation.h>
#include <ros/ros.h>


 template class PlaneSegmentation<pcl::PointXYZRGB>;
 template class PlaneSegmentation<pcl::PointXYZ>;

/** RANSAC plane estimation */
template<typename PointT>
bool PlaneSegmentation<PointT>::apply(CloudPtr out_cloud, pcl::PointCloud<pcl::Normal>::Ptr out_normals,
		CloudPtr cloud_plane, pcl::ModelCoefficients::Ptr coeffs)
{

  clock_t begin = clock();
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  typename pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PCDWriter writer;
  typename pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(num_iterations_);
  seg.setDistanceThreshold(distance_threshold_);
  seg.setInputCloud(in_cloud_);
  seg.setInputNormals(in_normals_);
  // Obtain the plane inliers and coefficients
  seg.segment(*inliers_plane, *coeffs);
  ROS_DEBUG_STREAM("Plane coefficients: " << *coeffs);
  clock_t end = clock();
  ROS_DEBUG_STREAM("Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC);

  // Extract the planar inliers from the input cloud
  extract.setInputCloud(in_cloud_);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);

  // Write the planar inliers to disk
  extract.filter(*cloud_plane);
  ROS_DEBUG_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points. Not saved.");
  //writer.write("/home/dfornas/data/scene_plane.pcd", *cloud_plane, false); //DEBUG

  // Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*out_cloud);

  extract_normals.setNegative(true);
  extract_normals.setInputCloud(in_normals_);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*out_normals);

  // @ TODO check if plane not found
  return true;
}

/** Remove plane */
template<typename PointT>
bool PlaneSegmentation<PointT>::removeFromCoefficients(CloudPtr out_cloud, pcl::PointCloud<pcl::Normal>::Ptr out_normals,
    CloudPtr cloud_plane, pcl::ModelCoefficients::Ptr coeffs)
{

  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  typename pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PCDWriter writer;
  typename pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(num_iterations_);
  seg.setDistanceThreshold(distance_threshold_);
  seg.setInputCloud(in_cloud_);
  seg.setInputNormals(in_normals_);
  // Obtain the plane inliers and coefficients
  seg.segment(*inliers_plane, *coeffs);
  ROS_DEBUG_STREAM("Plane coefficients: " << *coeffs);
  // Extract the planar inliers from the input cloud
  extract.setInputCloud(in_cloud_);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);

  // Write the planar inliers to disk
  extract.filter(*cloud_plane);
  ROS_DEBUG_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points. Not saved.");

  // Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*out_cloud);

  extract_normals.setNegative(true);
  extract_normals.setInputCloud(in_normals_);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*out_normals);

  // @ TODO check if plane not found
  return true;
}




