/*
 * pcl_clustering Description
 *  Created on: 12/05/2015
 *      Author: dfornas
 */
#ifndef PCLCLUSTERING_H_
#define PCLCLUSTERING_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pm_tools/pcl_tools.h>
#include <pm_tools/timing.h>

template <typename PointT>
class CloudClustering
{

  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;

  CloudPtr cloud_;

  /** Store the clusters in a vector of clouds  */
  void extract();

  void getColoredCloud();

  ros::Publisher clusteringStatsPublisher;
  ros::Publisher clusteringSizesPublisher;
  bool publishing_;

public:

  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<CloudPtr> cloud_clusters;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  CloudClustering(CloudPtr in_cloud, bool display_clusters = true) :
      cloud_(in_cloud) {
    publishing_ = false;
  }

  CloudClustering(ros::NodeHandle & nh, CloudPtr in_cloud, bool display_clusters = true) :
          cloud_(in_cloud)
  {
    publishing_ = true;
    clusteringStatsPublisher = nh.advertise<std_msgs::Float32>("stats/clustering", 10);
    clusteringSizesPublisher = nh.advertise<std_msgs::Float32MultiArray>("clustering/sizes", 10);
  }

  /** Apply Euclidian clustering http://pointclouds.org/documentation/tutorials/cluster_extraction.php */
  void applyEuclidianClustering( float tolerance = 0.02, int minSize = 50, int maxSize = 20000 );

  /** Apply Conditional Euclidian clustering http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php */
  void applyConditionalEuclidianClustering();

  /** Apply Region Growing clustering pointclouds.org/documentation/tutorials/region_growing_segmentation.php */
  void applyRegionGrowingClustering( float smoothnessTh = 3.0, float curvTh = 3.0, int minSize = 50, int maxSize = 20000, int neighbours = 30 );

  /** Apply RGB Region Growing clustering pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php */
  void applyRGBRegionGrowingClustering( float distance = 10, float pointColorTh = 6, float regionColorTh = 5, int minSize = 200);

  /** Save clusters to ~/ros_ws/tmp/name+id+.pcd */
  void save( std::string name );

  void display();
  void displayColoured( int ms = 500 );

};

template<typename PointT>
void CloudClustering<PointT>::save( std::string name )
{
	for (int i = 0; i < cloud_clusters.size(); ++i){
		std::stringstream ss;
		ss << "/home/dfornas/ros_ws/tmp/" << name << i << ".pcd" ;
		PCLTools<PointT>::cloudToPCD(cloud_clusters[i], ss.str());
	}
}

template<typename PointT>
void CloudClustering<PointT>::display()
{
	pcl::visualization::PCLVisualizer viewer("Cluster viewer");
	for (int i = 0; i < cloud_clusters.size(); ++i){
		std::stringstream ss;
		ss << "id" << i ;
		viewer.addPointCloud(cloud_clusters[i], ss.str());
        viewer.setBackgroundColor(0.8,0.8,0.8);
	}
	while (!viewer.wasStopped() )
		viewer.spinOnce();
}

template<typename PointT>
void CloudClustering<PointT>::displayColoured( int ms )
{
	pcl::visualization::PCLVisualizer viewer("Cluster viewer coloured");
	if (!cloud_clusters.empty ()) viewer.addPointCloud(colored_cloud, "id");
    viewer.setBackgroundColor(0.8,0.8,0.8);
  int elapsed = 0;
  while (!viewer.wasStopped() && elapsed < ms )
  {
    elapsed += 20;
    viewer.spinOnce(20);
    boost::this_thread::sleep(boost::posix_time::microseconds(20000));
  }
}

template<typename PointT>
void CloudClustering<PointT>::extract()
{
	cloud_clusters.clear();
  std_msgs::Float32MultiArray clusterSizesMsg;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
    CloudPtr cloud_cluster(new Cloud);
	  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		  cloud_cluster->points.push_back (cloud_->points[*pit]); //*
	  cloud_cluster->width = it->indices.size();
	  cloud_cluster->height = 1;
	  cloud_cluster->is_dense = true;
	  cloud_clusters.push_back(cloud_cluster);
    clusterSizesMsg.data.push_back(cloud_cluster->points.size());
	  ROS_DEBUG_STREAM("PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points.");
	}
  if(publishing_) clusteringSizesPublisher.publish(clusterSizesMsg);
  ros::spinOnce();
}

template<typename PointT>
void CloudClustering<PointT>::applyEuclidianClustering( float tolerance, int minSize, int maxSize )
{
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_);

  ProgramTimer tick;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (tolerance); // 2cm
  ec.setMinClusterSize (minSize);
  ec.setMaxClusterSize (maxSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_);
  ec.extract (cluster_indices);
  if(publishing_) clusteringStatsPublisher.publish(tick.getTotalTimeMsg());
  ros::spinOnce();

  extract();
  getColoredCloud();
}


template<typename PointT>
void CloudClustering<PointT>::applyConditionalEuclidianClustering()
{
	// @TODO Copy from tutorial & use condition usefully
}

template<typename PointT>
void CloudClustering<PointT>::applyRegionGrowingClustering( float smoothnessTh, float curvTh, int minSize, int maxSize, int neighbours )
{
  //Normal estimation
  typename pcl::search::Search<PointT>::Ptr tree =
		  boost::shared_ptr<typename pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  typename pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  ProgramTimer tick;
  typename pcl::RegionGrowing<PointT, pcl::Normal> reg;
  reg.setMinClusterSize (minSize);
  reg.setMaxClusterSize (maxSize);

  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (neighbours);
  reg.setInputCloud (cloud_);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (smoothnessTh / 180.0 * M_PI);
  reg.setCurvatureThreshold (curvTh);
  reg.extract (cluster_indices);
  if(publishing_) clusteringStatsPublisher.publish(tick.getTotalTimeMsg());
  ros::spinOnce();

  colored_cloud = reg.getColoredCloud ();
  extract();
}

template<typename PointT>
void CloudClustering<PointT>::applyRGBRegionGrowingClustering( float distance, float pointColorTh, float regionColorTh, int minSize )
{
	typename pcl::search::Search<PointT>::Ptr tree =
			boost::shared_ptr<typename pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
	pcl::RegionGrowingRGB<PointT> reg;

  ProgramTimer tick;
  reg.setInputCloud (cloud_);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (distance);
	reg.setPointColorThreshold (pointColorTh);
	reg.setRegionColorThreshold (regionColorTh);
	reg.setMinClusterSize (minSize);
	reg.extract (cluster_indices);
  clusteringStatsPublisher.publish(tick.getTotalTimeMsg());
  ros::spinOnce();

	colored_cloud = reg.getColoredCloud ();
	extract();
}

template <typename PointT>
void CloudClustering<PointT>::getColoredCloud()
{
  if (!cloud_clusters.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < cloud_clusters.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = cloud_->width;
    colored_cloud->height = cloud_->height;
    colored_cloud->is_dense = cloud_->is_dense;
    for (size_t i_point = 0; i_point < cloud_->points.size (); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = cloud_->points[i_point].x;
      point.y = cloud_->points[i_point].y;
      point.z = cloud_->points[i_point].z;
      point.r = 255;
      point.g = 0;
      point.b = 0;
      colored_cloud->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = cluster_indices.begin (); i_segment != cluster_indices.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
      {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }
}

#endif
