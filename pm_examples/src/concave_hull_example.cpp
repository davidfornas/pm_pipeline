/** 
 * This program test the border detection http://pointclouds.org/documentation/tutorials/range_image_border_extraction.php
 *  Created on: 17/03/2014
 *      Author: dfornas
 */
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

#include <pcl/visualization/pcl_visualizer.h>

//Time measures
#include <ctime>

typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>), cloud_filtered (new pcl::PointCloud<PointT>), cloud_projected (new pcl::PointCloud<PointT>) ;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ()), cloud_plane (new pcl::PointCloud<PointT> ());
  pcl::PCDReader reader;

  reader.read ("uwsimFeb15.pcd", *cloud);
  // Build a filter to remove spurious NaNs

  clock_t begin = clock();

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
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
  pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT>);
  pcl::ConcaveHull<PointT> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.1);
  chull.reconstruct (*cloud_hull);

  end = clock();
  std::cerr << "Elapsed concave hull time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  //writer.write ("chull.pcd", *cloud_hull, false);
  std::cout << cloud_hull->points.size() << std::endl;

  end = clock();
  std::cerr << "Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  // ----  VISUALIZATION  ---
  // -----Open 3D viewer and add point cloud-----
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    //viewer.addCoordinateSystem (1.0f);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> point_cloud_color_handler (cloud_filtered, 0, 0, 0);
    viewer.addPointCloud (cloud_filtered);//, point_cloud_color_handler, "original point cloud");
    for (int i=1; i<cloud_hull->points.size(); ++i)
    {
      std::ostringstream id;
      id << "name: " << i ;
      viewer.addLine<PointT>(cloud_hull->points[i-1],cloud_hull->points[i],0,255,0,id.str());
    }

  // -----Main loop-----
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }
  return (0);
}
