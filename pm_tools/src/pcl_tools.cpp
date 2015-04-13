/*
 * pcl_tools
 * 			 
 *  Created on: 03/03/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>

#include <ros/topic.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

void PCLTools::cloudFromPCD(pcl::PointCloud<PointT>::Ptr cloud, std::string fileName){
  pcl::PCDReader reader;
  reader.read(fileName, *cloud);
  std::cerr << "PointCloud loaded: " << cloud->points.size() << " data points." << std::endl;
}

void PCLTools::cloudFromTopic(pcl::PointCloud<PointT>::Ptr cloud, std::string topicName){
  sensor_msgs::PointCloud2::ConstPtr message = ros::topic::waitForMessage< sensor_msgs::PointCloud2 >(topicName);
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*message, pcl_pc);
  //PCL Generic cloud to XYZRGB strong type.
  pcl::fromPCLPointCloud2(pcl_pc, *cloud);
  std::cerr << "PointCloud loaded: " << cloud->points.size() << " data points." << std::endl;
}

void PCLTools::applyZAxisPassthrough(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out, double min, double max){
  pcl::PassThrough<PointT> pass;
  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-10, 10);
  pass.filter (*out);
}

/** Statistical Outlier Removal filter */
void PCLTools::applyStatisticalOutlierRemoval(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out){
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud (in);
  vg.setLeafSize (0.01, 0.01, 0.01);
  vg.filter (*out);
}

/** Voxel Grid filter filter */
void PCLTools::applyVoxelGridFilter(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out){
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (in);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter(*out);
}

/** Compute normals */
void PCLTools::estimateNormals(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
  std::cerr << "Applying NORMAL ESTIMATION..." << std::endl;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (in);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
  std::cerr << "Applying NORMAL ESTIMATION2..." << std::endl;
}

int PCLTools::nanCount(pcl::PointCloud<PointT>::Ptr p)
{
  int count = 0;
  for (size_t i = 0; i < p->points.size(); ++i)
    if (pcl::isFinite(p->points[i]))
      count++;
  return count;
}

void PCLTools::mergeOrganizedClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr a, pcl::PointCloud<pcl::PointXYZRGB>::Ptr b)
{
  //A,B should be organized clouds of the same size... @ TODO CHECK
  for (size_t i = 0; i < a->points.size(); ++i)
    if (!pcl::isFinite(a->points[i]) && pcl::isFinite(b->points[i]))
      a->points[i] = b->points[i];
}
/******************** END OF PCL TOOLS ****************/

pcl::PointXYZRGB CloudMerge::accumPoints(pcl::PointXYZRGB a, pcl::PointXYZRGB b, int idx)
{
  pcl::PointXYZRGB c(a);
  c.x = (a.x * coeffs[idx] + b.x) / (coeffs[idx] + 1);
  c.y = (a.y * coeffs[idx] + b.y) / (coeffs[idx] + 1);
  c.z = (a.z * coeffs[idx] + b.z) / (coeffs[idx] + 1);
  xvar[idx] += b.x * b.x;
  yvar[idx] += b.y * b.y;
  zvar[idx] += b.z * b.z;
  return c;
}

void CloudMerge::nanAwareOrganizedConcatenateMean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr a, pcl::PointCloud<pcl::PointXYZRGB>::Ptr b)
{
  //A,B should be organized clouds of the same size...
  for (size_t i = 0; i < a->points.size(); ++i){
    //First time a point is seen count=1;
    if(pcl::isFinite(a->points[i]) && coeffs[i] == 0){
      coeffs[i] = 1;
      xvar[i] = a->points[i].x * a->points[i].x;
      yvar[i] = a->points[i].y * a->points[i].y;
      zvar[i] = a->points[i].z * a->points[i].z;
    }

    if (pcl::isFinite(b->points[i])){
      if (!pcl::isFinite(a->points[i]))
      {
        //Point in B not found in A, add it.
        a->points[i] = b->points[i];
        //TODO: Search nearest neighbor color. Right now color is empty.
        xvar[i] = a->points[i].x * a->points[i].x;
        yvar[i] = a->points[i].y * a->points[i].y;
        zvar[i] = a->points[i].z * a->points[i].z;
        coeffs[i] = 1;
      }
      else
      {
        //Point found on both clouds. Weighted average.
        a->points[i] = accumPoints(a->points[i], b->points[i], i); //c+d;//=(a->points[i]*coeffs[i]+b->points[i])/(coeffs[i]+1);
        coeffs[i]++;
      }
    }
  }
}
/******************** END OF CLOUD MERGER ****************/

/** RANSAC plane estimation */
bool PlaneSegmentation::apply(pcl::PointCloud<PointT>::Ptr out_cloud, pcl::PointCloud<pcl::Normal>::Ptr out_normals, pcl::PointCloud<PointT>::Ptr cloud_plane, pcl::ModelCoefficients::Ptr coeffs){

  clock_t begin = clock();
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PCDWriter writer;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (num_iterations_);
  seg.setDistanceThreshold (distance_threshold_);
  seg.setInputCloud (in_cloud_);
  seg.setInputNormals (in_normals_);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coeffs);
  std::cerr << "Plane coefficients: " << *coeffs << std::endl;
  clock_t end = clock();
  std::cerr << "Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (in_cloud_);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points. Saved." << std::endl;
  writer.write ("/home/dfornas/data/scene_plane.pcd", *cloud_plane, false); //DEBUG

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*out_cloud);
  
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (in_normals_);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*out_normals);

  // @ TODO check if plane not found
  return true;
}

/** RANSAC cylinder estimation */
bool CylinderSegmentation::apply(pcl::PointCloud<PointT>::Ptr cloud_cylinder, pcl::ModelCoefficients::Ptr coeffs){

  clock_t begin = clock();
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ExtractIndices<PointT> extract;
  pcl::PCDWriter writer;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (num_iterations_);//10000
  seg.setDistanceThreshold (distance_threshold_);//0.05
  seg.setRadiusLimits (0, radious_limit_);//0, 0.1
  seg.setInputCloud (in_cloud_);
  seg.setInputNormals (in_normals_);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder_, *coeffs);
  std::cerr << "Cylinder coefficients: " << *coeffs << std::endl;
  clock_t end = clock();
  std::cerr << "Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  // Write the cylinder inlier1s to disk
  extract.setInputCloud (in_cloud_);
  extract.setIndices (inliers_cylinder_);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    std::cerr << *coeffs << std::endl;
    writer.write ("/home/dfornas/data/scene_cylinder.pcd", *cloud_cylinder, false);
  }
  return true;
}

/** Show segmented cloud and plane by coefficients and inliers */
void PCLTools::showClouds(pcl::PointCloud<PointT>::Ptr c1, pcl::PointCloud<PointT>::Ptr c2, pcl::ModelCoefficients::Ptr plane_coeffs, pcl::ModelCoefficients::Ptr cylinder_coeffs){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Segmentation Viewer"));
     viewer->setBackgroundColor (0, 0, 0);
     pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(c1, 20,20,90);
     pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color2(c1, 20, 200, 20);

     viewer->addPointCloud<PointT>(c1, single_color,  "Plane cloud");
     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Plane cloud");
     viewer->addPointCloud<PointT>(c2, single_color2, "Cylinder cloud");
     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cylinder cloud");

     if(plane_coeffs!=0){
       viewer->addPlane (*plane_coeffs, "Plane");
     }
     if(cylinder_coeffs!=0){
       viewer->addCylinder(*cylinder_coeffs, "Cylinder");
     }

     //viewer->addCoordinateSystem (0.1);
     viewer->initCameraParameters ();
     while (!viewer->wasStopped ())
     {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     }
}


