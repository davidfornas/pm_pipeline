/*
 * pcl_tools
 * 			 
 *  Created on: 03/03/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_segmentation.h>
#include <ros/ros.h>


/** RANSAC plane estimation */
bool PlaneSegmentation::apply(pcl::PointCloud<PointT>::Ptr out_cloud, pcl::PointCloud<pcl::Normal>::Ptr out_normals,
                              pcl::PointCloud<PointT>::Ptr cloud_plane, pcl::ModelCoefficients::Ptr coeffs)
{

  clock_t begin = clock();
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PCDWriter writer;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

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
  ROS_INFO_STREAM("Plane coefficients: " << *coeffs);
  clock_t end = clock();
  ROS_DEBUG_STREAM("Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC);

  // Extract the planar inliers from the input cloud
  extract.setInputCloud(in_cloud_);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);

  // Write the planar inliers to disk
  extract.filter(*cloud_plane);
  ROS_INFO_STREAM(
      "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points. Saved.");
  writer.write("/home/dfornas/data/scene_plane.pcd", *cloud_plane, false); //DEBUG

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

/** RANSAC cylinder estimation */
bool CylinderSegmentation::apply(pcl::PointCloud<PointT>::Ptr cloud_cylinder, pcl::ModelCoefficients::Ptr coeffs)
{

  clock_t begin = clock();
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  pcl::ExtractIndices<PointT> extract;
  pcl::PCDWriter writer;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

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

/** Show segmented cloud and plane by coefficients and inliers */
void PCLTools::showClouds(pcl::PointCloud<PointT>::Ptr c1, pcl::PointCloud<PointT>::Ptr c2,
                          pcl::ModelCoefficients::Ptr plane_coeffs, pcl::ModelCoefficients::Ptr cylinder_coeffs)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("Segmentation Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(c1, 20, 20, 90);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color2(c1, 20, 200, 20);

  viewer->addPointCloud<PointT>(c1, single_color, "Plane cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Plane cloud");
  viewer->addPointCloud<PointT>(c2, single_color2, "Cylinder cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cylinder cloud");

  if (plane_coeffs != 0)
  {
    viewer->addPlane(*plane_coeffs, "Plane");
  }
  if (cylinder_coeffs != 0)
  {
    viewer->addCylinder(*cylinder_coeffs, "Cylinder");
  }
  viewer->initCameraParameters();
  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

