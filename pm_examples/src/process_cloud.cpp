/** 
 * This program is used to filter and modify point clouds easily.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
/*#include <pm_tools/pcl_segmentation.h>
#include <ros/ros.h>

typedef PointTRGB PointType;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>), point_cloud_ptr2(new pcl::PointCloud<PointType>) ,
      point_cloud_ptr3(new pcl::PointCloud<PointType>),  plane(new pcl::PointCloud<PointType>);
  PCLTools::cloudFromPCD(point_cloud_ptr, std::string(argv[1]) + std::string(".pcd")); //Load from PCDReader or from topic

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>), cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);

  PCLTools::applyZAxisPassthrough(point_cloud_ptr, point_cloud_ptr2, atoi(argv[2]), atoi(argv[3]));

  PCLTools::estimateNormals(point_cloud_ptr2, cloud_normals);

  PlaneSegmentation plane_seg(point_cloud_ptr2, cloud_normals);
  plane_seg.setDistanceThreshold(atof(argv[4]));
  plane_seg.setIterations(100);
  plane_seg.apply(point_cloud_ptr3, cloud_normals2, plane, coefficients_plane);

  PCLTools::cloudToPCD(point_cloud_ptr3, std::string(argv[1]) + std::string("_processed.pcd"));
  PCLTools::cloudToPCD(plane, std::string(argv[1]) + std::string("_plane.pcd"));

  return (0);
}*/

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <pcl/filters/filter.h>

typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<PointT>::Ptr cloud00 (new pcl::PointCloud<PointT>), cloud0 (new pcl::PointCloud<PointT>), cloud (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);
  reader.read (std::string(argv[1]), *cloud00);
  std::cout << "PointCloud before filtering has: " << cloud0->points.size () << " data points." << std::endl; //*
  PCLTools::applyZAxisPassthrough(cloud00, cloud0, 0.2, 1.8);

  std::vector <int> idx;
  pcl::removeNaNFromPointCloud (*cloud0, *cloud, idx);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointT> vg;
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
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
    pcl::ExtractIndices<PointT> extract;
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
  }

  std::cout << "REMAINING " << cloud_filtered->points.size () << " data points." << std::endl;

  writer.write<PointT> ("remaining", *cloud_filtered, false); //*
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "clusters/cloud_cluster_" << j << ".pcd";
    writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}

