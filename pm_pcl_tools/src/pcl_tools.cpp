/*
 * pcl_tools
 * 			 
 *  Created on: 03/03/2015
 *      Author: dfornas
 */
#include <pm_pcl_tools/pcl_tools.h>

//Time measures
#include <ctime>



namespace pm_pcl_tools{

void PCLoader(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, std::string name, bool fromFile = true){}//Load from PCDReader or from topic
void PassthroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, double x, double y, double z){}
	/*
void passThrough(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out, double z_min, double z_max){
  pcl::PassThrough<PointT> pass;
  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-10, 10);
  pass.filter (*out);
}

void statisticalOutlierRemoval(pcl::PCLPointCloud2::Ptr in, pcl::PCLPointCloud2::Ptr out){
  pcl::VoxelGrid< pcl::PCLPointCloud2 > vg;
  vg.setInputCloud (in);
  vg.setLeafSize (0.01, 0.01, 0.01);
  vg.filter (*out);
}

void voxelGridFilter(pcl::PCLPointCloud2::Ptr in, pcl::PCLPointCloud2::Ptr out){
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (in);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter(*out);
		
}

void estimateNormals(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){  
  
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());  
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (in);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
	
}

pcl::ModelCoefficients::Ptr planeSegmentation(pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr in_normals,
                                                pcl::PointCloud<PointT>::Ptr out_cloud, pcl::PointCloud<pcl::Normal>::Ptr out_normals,
                                                pcl::PointCloud<PointT>::Ptr cloud_plane, double distanceThreshold, int iterations){

  clock_t begin = clock();
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PCDWriter writer;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (iterations);
  seg.setDistanceThreshold (distanceThreshold);
  seg.setInputCloud (in_cloud);
  seg.setInputNormals (in_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
  clock_t end = clock();
  std::cerr << "Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (in_cloud);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points. Saved." << std::endl;
  writer.write ("/home/dfornas/data/scene_plane.pcd", *cloud_plane, false); 

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*out_cloud);
  
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (in_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*out_normals);

  return coefficients_plane;
}

pcl::ModelCoefficients::Ptr cylinderSegmentation(pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr in_normals,
                                                    pcl::PointCloud<PointT>::Ptr cloud_cylinder, double distanceThreshold,
                                                    int iterations, double rlimit){

  clock_t begin = clock();
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);  
  pcl::ExtractIndices<PointT> extract;
  pcl::PCDWriter writer;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (iterations);//10000
  seg.setDistanceThreshold (distanceThreshold);//0.05
  seg.setRadiusLimits (0, rlimit);//0, 0.1
  seg.setInputCloud (in_cloud);
  seg.setInputNormals (in_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
  clock_t end = clock();
  std::cerr << "Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  // Write the cylinder inlier1s to disk
  extract.setInputCloud (in_cloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    std::cerr << *coefficients_cylinder << std::endl;
    writer.write ("/home/dfornas/data/scene_cylinder.pcd", *cloud_cylinder, false);
  }


  return coefficients_cylinder;
}

void showClouds(pcl::PointCloud<PointT>::Ptr c1, pcl::PointCloud<PointT>::Ptr c2, pcl::ModelCoefficients::Ptr plane_coeffs, pcl::ModelCoefficients::Ptr cylinder_coeffs){

     //pcl::visualization::PCLVisualizer viewer ("3D Viewer");
     //viewer.setBackgroundColor (0, 0, 0);
     //viewer.addPointCloud<PointT> (c1, "sample cloud");
     //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
     //viewer.addCoordinateSystem (1.0);
     //viewer.initCameraParameters ();
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Segmentation Viewer"));
     viewer->setBackgroundColor (0, 0, 0);
     pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(c1, 0, 255, 0);
     pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color2(c1, 0, 255, 255);

     viewer->addPointCloud<PointT>(c1, single_color,  "sample cloud");
     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
     viewer->addPointCloud<PointT>(c2, single_color2, "sample cloud2");
     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");

     if(plane_coeffs!=0){
       viewer->addPlane (*plane_coeffs, "plane");
     }
     if(cylinder_coeffs!=0){
       viewer->addCylinder(*cylinder_coeffs, "cylinder");
     }

     //viewer->addCoordinateSystem (0.1);
     viewer->initCameraParameters ();
     while (!viewer->wasStopped ())
     {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     }
}*/

} //end namespace
