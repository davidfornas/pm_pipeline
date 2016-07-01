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
	bool visualize_;

    public:
	/** Constructor.
	 * @param cloud, background_remover, segmentator, hypothesis_generator
	 * */
      HypothesisGeneration( CloudPtr c, bool visualize = true ) : cloud_(c), visualize_(visualize){}

	  vpHomogeneousMatrix getGraspCandidate();
};




template<typename PointT>
vpHomogeneousMatrix HypothesisGeneration<PointT>::getGraspCandidate(){

	  CloudClustering<PointT> cluster(cloud_);
	  cluster.applyEuclidianClustering();
	  cluster.save("grasping");
	  if (visualize_) cluster.displayColoured();

	  ClusterMeasure<PointT> cm(cluster.cloud_clusters[0], visualize_);
	  Eigen::Vector4f centroid = cm.getCentroid();
	  Eigen::Matrix3f axis = cm.getAxis();

	  Eigen::Quaternionf q;
	  Eigen::Vector3f t;
	  float width, height, depth;

	  cm.getOABBox( q, t, width, height, depth );

	  pcl::visualization::PCLVisualizer viewer("Hypothesis Generation Viewer");
	  viewer.addPointCloud(cloud_);
	  PointT p;
	  p.x = centroid[0];
	  p.y = centroid[1];
	  p.z = centroid[2];
	  viewer.addSphere(p, 0.01, 255, 0, 0, "centroid");
	  Eigen::Affine3f tr;
	  viewer.addCoordinateSystem(1.0, tr);
	  viewer.spin();


	  vpHomogeneousMatrix m;
	  return m;

}

/* SOME EIGEN CODE
 *
 * #include <Eigen/Geometry>

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

int main() {
  Eigen::Affine3d r = create_rotation_matrix(1.0, 1.0, 1.0);
  Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));

  Eigen::Matrix4d m = (t * r).matrix(); // Option 1

  Eigen::Matrix4d m = t.matrix(); // Option 2
  m *= r.matrix();
  return 0;
}

*/


#endif /* HYPOTHESISGENERATION_H_ */
