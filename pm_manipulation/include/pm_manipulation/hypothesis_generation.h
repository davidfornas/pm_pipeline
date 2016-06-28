/*
 * hypothesis_generation.h To implement
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef HYPOTHESISGENERATION_H_
#define HYPOTHESISGENERATION_H_

#include <pcl/io/pcd_io.h>
#include <mar_robot_arm5e/ARM5Arm.h>

#include <pm_tools/pcl_clustering.h>
#include <pm_perception/cluster_measure.h>

/** Description
 */
template <typename PointT>
class HypothesisGeneration {

	typedef typename pcl::PointCloud<PointT> Cloud;
	typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

	CloudPtr cloud_;

    public:
	/** Constructor.
	 * @param cloud, background_remover, segmentator, hypothesis_generator
	 * */
      HypothesisGeneration( CloudPtr c ) : cloud_(c){}

	  vpHomogeneousMatrix getGraspCandidate();
};




template<typename PointT>
vpHomogeneousMatrix HypothesisGeneration<PointT>::getGraspCandidate(){

	  //One time clustering...
	  //CloudClustering<PointT> cluster(point_cloud_ptr);

	  /*
	  cluster.applyEuclidianClustering();
	  cluster.displayColoured();
	  cluster.save("euclidian");

	  cluster.applyRegionGrowingClustering();
	  cluster.displayColoured();
	  cluster.save("growing");
	  */

	  CloudClustering<PointT> cluster(cloud_);
	  cluster.applyEuclidianClustering();
	  //SWITCH CASE
	  //cluster.applyRegionGrowingClustering();
	  //cluster.applyRGBRegionGrowingClustering();

	  cluster.save("grasping");
	  cluster.displayColoured();

	  //The selection of the cluster can be done using a image of the environment
	  // and selecting the corresponding 3D cluster (Toni).

	  ClusterMeasure<PointT> cm(cluster.cloud_clusters[0]);
	  cm.getCentroid();
	  cm.getAxis();

	  Eigen::Quaternionf q;
	  Eigen::Vector3f t;
	  float width, height, depth;

	  cm.getOABBox( q, t, width, height, depth );
	  vpHomogeneousMatrix m;
	  return m;

}






#endif /* HYPOTHESISGENERATION_H_ */
