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

void PCLTools::cloudToPCD(pcl::PointCloud<PointT>::Ptr cloud, std::string fileName){
  pcl::PCDWriter writer;
  writer.write(fileName, *cloud, false);
  std::cerr << "PointCloud saved." << std::endl;
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
  pass.setFilterLimits (min, max);
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

