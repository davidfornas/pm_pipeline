/*
 * pcl_tools Point cloud processing tools using PCL
 *  Created on: 03/03/2015
 *      Author: dfornas
 */
#ifndef PCLTOOLS_H_
#define PCLTOOLS_H_

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

#include <ros/topic.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

template <typename PointT>
class PCLTools
{
	typedef typename pcl::PointCloud<PointT> Cloud;
	typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

public:

	static void cloudFromPCD(CloudPtr cloud, std::string fileName){
	  pcl::PCDReader reader;
	  reader.read(fileName, *cloud);
	  ROS_DEBUG_STREAM("PointCloud loaded: " << cloud->points.size() << " data points." << std::endl);
	}

	static void cloudToPCD(CloudPtr cloud, std::string fileName){
	  pcl::PCDWriter writer;
	  writer.write(fileName, *cloud, false);
	  ROS_DEBUG_STREAM("PointCloud saved." << cloud->points.size() << " data points."  << std::endl);
	}

	static void cloudFromTopic(CloudPtr cloud, std::string topicName){
	  sensor_msgs::PointCloud2::ConstPtr message = ros::topic::waitForMessage< sensor_msgs::PointCloud2 >(topicName);
	  pcl::PCLPointCloud2 pcl_pc;
	  pcl_conversions::toPCL(*message, pcl_pc);
	  //PCL Generic cloud to PointT strong type.
	  pcl::fromPCLPointCloud2(pcl_pc, *cloud);
	  ROS_DEBUG_STREAM("PointCloud loaded: " << cloud->points.size() << " data points." << std::endl);
	}

	static void applyZAxisPassthrough(CloudPtr in, CloudPtr out, double min, double max){
	  typename pcl::PassThrough<PointT> pass;
	  // Build a passthrough filter to remove spurious NaNs
	  pass.setInputCloud (in);
	  pass.setFilterFieldName ("z");
	  pass.setFilterLimits (min, max);
	  pass.filter (*out);
	}
	//In place filter version
	static void applyZAxisPassthrough(CloudPtr & in, double min, double max){
	  CloudPtr result( new Cloud );
	  applyZAxisPassthrough(in, result, min, max);
	  in = result;
	}

	/** Voxel Grid filter filter */
	static void applyVoxelGridFilter(CloudPtr in, CloudPtr out,  float size = 0.03){
	  typename pcl::VoxelGrid<PointT> vg;
	  vg.setInputCloud (in);
	  vg.setLeafSize (size, size, size);
	  vg.filter (*out);
	}
	static void applyVoxelGridFilter(CloudPtr & in,  float size = 0.03){
	  CloudPtr result( new Cloud );
	  applyVoxelGridFilter(in, result, size);
	  in = result;
	}

	/** Statistical Outlier Removal filter */
	static void applyStatisticalOutlierRemoval(CloudPtr in, CloudPtr out){
	  typename pcl::StatisticalOutlierRemoval<PointT> sor;
	  sor.setInputCloud (in);
	  sor.setMeanK (50);
	  sor.setStddevMulThresh (1.0);
	  sor.filter(*out);
	}
	static void applyStatisticalOutlierRemoval(CloudPtr & in){
	  CloudPtr result( new Cloud );
	  applyVoxelGridFilter(in, result);
	  in = result;
	}

	/** Compute normals */
	static void estimateNormals(CloudPtr in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
	  ROS_DEBUG_STREAM("Applying normal estimation" << std::endl);
	  typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT> ());
	  typename pcl::NormalEstimation<PointT, pcl::Normal> ne;
	  // Estimate point normals
	  ne.setSearchMethod (tree);
	  ne.setInputCloud (in);
	  ne.setKSearch (50);
	  ne.compute (*cloud_normals);
	}

	static int nanCount(CloudPtr p)
	{
	  int count = 0;
	  for (size_t i = 0; i < p->points.size(); ++i)
	    if (pcl::isFinite(p->points[i]))
	      count++;
	  return count;
	}

	static void removeNanPoints(CloudPtr p, CloudPtr copy){
		int new_size = PCLTools<PointT>::nanCount(p);
	    copy->width    = new_size;
		copy->height   = 1;
		copy->is_dense = false;
		copy->points.resize (copy->width * copy->height);

		int idx=0;
		  for (size_t i = 0; i < p->points.size(); ++i)
		    if (pcl::isFinite(p->points[i]))
		    	copy->points[idx++] = p->points[i];
		  ROS_DEBUG_STREAM("New size:" << idx);
	}

	static void mergeOrganizedClouds(CloudPtr a, CloudPtr b)
	{
	  //A,B should be organized clouds of the same size... @ TODO CHECK
	  for (size_t i = 0; i < a->points.size(); ++i)
	    if (!pcl::isFinite(a->points[i]) && pcl::isFinite(b->points[i]))
	      a->points[i] = b->points[i];
	}

	/** Show segmented cloud and plane by coefficients and inliers */
	static void showClouds(CloudPtr c1, CloudPtr c2, pcl::ModelCoefficients::Ptr plane_coeffs, pcl::ModelCoefficients::Ptr cylinder_coeffs)
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
};

#endif
