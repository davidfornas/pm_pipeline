/*
 * background_removal.cpp
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#include <pm_perception/background_removal.h>
#include <pm_tools/timing.h>

void BackgroundRemoval::initialize(CloudPtr &output, pcl::PointCloud<pcl::Normal>::Ptr &output_normals) {

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  coefficients_plane = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;

  ProgramTimer tick;
  PCLTools<PointT>::estimateNormals(cloud_, cloud_normals);

  PlaneSegmentation<PointT> plane_seg(cloud_, cloud_normals);
  plane_seg.setDistanceThreshold(plane_distance_threshold_);
  plane_seg.setIterations(plane_iterations_);
  plane_seg.apply(output, output_normals, cloud_plane, coefficients_plane);
  backgroundExtractionStatsPublisher.publish(tick.getTotalTimeMsg());

}


void BackgroundRemoval::process(CloudPtr & output, pcl::PointCloud<pcl::Normal>::Ptr &output_normals){

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  coefficients_plane = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;

  ProgramTimer tick;
  PCLTools<PointT>::estimateNormals(cloud_, cloud_normals);

  if( ransac_background_filter_ ) {
    PlaneSegmentation<PointT> plane_seg(cloud_, cloud_normals);
    plane_seg.setDistanceThreshold(plane_distance_threshold_);
    plane_seg.setIterations(plane_iterations_);
    plane_seg.apply(output, output_normals, cloud_plane, coefficients_plane);
    //// @TODO plane_seg.removeFromCoefficients(cloud_filtered2, cloud_normals2, cloud_plane, coefficients_plane);
  }else{
    PCLTools<PointT>::applyZAxisPassthrough(cloud_, output, 0, 0.89);//REAL 0.89 SIM 1.4 "REMOVE FILTER" 3
    PCLTools<PointT>::estimateNormals(output, output_normals);
  }
  backgroundExtractionStatsPublisher.publish(tick.getTotalTimeMsg());

}

/* Obtain signed distance to plane */
double BackgroundRemoval::signedDistanceToPlane( PointT p ){
  return PCLTools<PointT>::signedDistanceToPlane( p, coefficients_plane->values[0], coefficients_plane->values[1],
                                                     coefficients_plane->values[2], coefficients_plane->values[3] );
}

void BackgroundRemoval::removeIteratively(CloudPtr & output){

  ROS_ERROR("NOT IMPLEMENTED YET");

/*    ITERATIVE PLANE EXTRACTION
// Create the segmentation object for the planar model and set all the parameters
typename pcl::SACSegmentation<PointT> seg;
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
CloudPtr cloud_plane (new typename pcl::PointCloud<PointT> ());
pcl::PCDWriter writer;
seg.setOptimizeCoefficients (true);
seg.setModelType (pcl::SACMODEL_PLANE);
seg.setMethodType (pcl::SAC_RANSAC);
seg.setMaxIterations (100);
seg.setDistanceThreshold (0.02);

int i=0, nr_points = (int) cloud_filtered->points.size ();
while (cloud_filtered->points.size () > 0.3 * nr_points)
{
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    break;
  }

  // Extract the planar inliers from the input cloud
  typename pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);
  *cloud_filtered = *cloud_f;

  std::cout << "PointCloud representing cloud filterd: " << cloud_filtered->points.size () << " data points." << std::endl;
}*/

}

