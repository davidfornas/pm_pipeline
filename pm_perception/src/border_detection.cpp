/*
 * border_detection.cpp
 *
 *  Created on: 18/03/2014
 *      Author: dfornas
 */

#include <pm_perception/border_detection.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>

//Time measures
#include <ctime>

void ConcaveHullBorderDetection::process(){
  pcl::PointCloud<PointT>::Ptr  cloud_filtered (new pcl::PointCloud<PointT>), cloud_projected (new pcl::PointCloud<PointT>) ;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ()), cloud_plane (new pcl::PointCloud<PointT> ());

    // Build a filter to remove spurious NaNs

    clock_t begin = clock();

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-10, 10);
    pass.filter (*cloud_filtered);

    // Create the segmentation object
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
    pcl::NormalEstimation<PointT, pcl::Normal> ne;  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Estimate point normals
      ne.setSearchMethod (tree);
      ne.setInputCloud (cloud_filtered);
      ne.setKSearch (50);
      ne.compute (*cloud_normals);

      // Create the segmentation object for the planar model and set all the parameters
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      seg.setNormalDistanceWeight (0.1);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (100);
      seg.setDistanceThreshold (0.12);
      seg.setInputCloud (cloud_filtered);
      seg.setInputNormals (cloud_normals);

      // Obtain the plane inliers and coefficients
      seg.segment (*inliers_plane, *coefficients_plane);

  pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.03);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud_cylinder);

    clock_t end = clock();
    std::cerr << "Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

    begin = clock();

    // Project the CILINDER inliers into the PLANE model
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers_cylinder);
    proj.setInputCloud (cloud_filtered2);
    proj.setModelCoefficients (coefficients_plane);
    proj.filter (*cloud_projected);

    end = clock();
    std::cerr << "Elapsed projection time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
    begin = clock();

    //pcl::PCDWriter writer;
    //writer.write ("projected_cylinder.pcd", *cloud_projected, false);

    // Create a Concave Hull representation of the projected inliers
    pcl::ConcaveHull<PointT> chull;
    chull.setInputCloud (cloud_projected);
    chull.setAlpha (0.1);
    border_cloud_ = boost::shared_ptr< pcl::PointCloud<PointT> >(new pcl::PointCloud<PointT>);
    chull.reconstruct (*border_cloud_);

    end = clock();
    std::cerr << "Elapsed concave hull time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

    //writer.write ("chull.pcd", *border_cloud_, false);
    std::cout << border_cloud_->points.size() << std::endl;

    end = clock();
    std::cerr << "Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
}

void RangeImageBorderDetection::process(){

  // -----Parameters-----
  float angular_resolution = 0.5f;
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  bool setUnseenToMaxRange = false;

  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (cloud_->sensor_origin_[0],
                                                             cloud_->sensor_origin_[1],
                                                             cloud_->sensor_origin_[2])) *
                      Eigen::Affine3f (cloud_->sensor_orientation_);

  // -----Create RangeImage from the PointCloud-----
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud(*cloud_, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();



  // -----Extract borders-----
  pcl::RangeImageBorderExtractor border_extractor (&range_image);
  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  border_extractor.compute (border_descriptions);

  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
                                      & veil_points = * veil_points_ptr,
                                      & shadow_points = *shadow_points_ptr;
  for (int y=0; y< (int)range_image.height; ++y)
  {
    for (int x=0; x< (int)range_image.width; ++x)
    {
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
        border_points.points.push_back (range_image.points[y*range_image.width + x]);
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
        veil_points.points.push_back (range_image.points[y*range_image.width + x]);
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
        shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
    }
  }
  border_cloud_with_ranges_=border_points_ptr;

}
