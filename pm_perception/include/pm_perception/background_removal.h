/*
 * background_removal.h
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef BACKGROUNDREMOVAL_H_
#define BACKGROUNDREMOVAL_H_

#include <pcl/io/pcd_io.h>

/** Description
 */
class BackgroundRemoval {


    public:
	/** Constructor.
	 * @param cloud, background_remover, segmentator, hypothesis_generator
	 * */
        BackgroundRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	}

        void setParameters(){}
	void process();

	~BackgroundRemoval(){}
};

#endif /* BACKGROUNDREMOVAL_H_ */

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
