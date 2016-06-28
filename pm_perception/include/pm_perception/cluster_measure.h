/*
 * cluster_measure This is used to obtain PCA, BBox and other info from clusters
 *  Created on: 19/05/2016
 *      Author: dfornas
 */
#ifndef PCLCLUSTERMEASURE_H_
#define PCLCLUSTERMEASURE_H_

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pm_tools/pcl_tools.h>

template <typename PointT>
class ClusterMeasure
{
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::iterator CloudIt;

  CloudPtr cloud_;
  bool visualize_;

public:

  ClusterMeasure(CloudPtr in_cloud, bool visualize = true) :
      cloud_(in_cloud), visualize_(visualize)
  {
  }

  ClusterMeasure(std::string in_cloud_file, bool visualize = true)
  {
	  PCLTools<PointT>::cloudFromPCD(cloud_, in_cloud_file);
	  visualize_ = visualize;
  }

  /** Return the cluster centroid  */
  Eigen::Vector4f getCentroid();

  /** Visualize PCA axis */
  void getAxis();

  /** Return the object aligned Bounding Box (minimal) */
  void getOABBox(Eigen::Quaternionf & qfinal, Eigen::Vector3f & tfinal, float & width, float & height, float & depth);

  /** Display result  */
  void displayCluster();


};

template<typename PointT>
void ClusterMeasure<PointT>::displayCluster()
{
	pcl::visualization::PCLVisualizer viewer("Cluster measure viewer");
    viewer.addPointCloud(cloud_);
    viewer.spin();
}

template<typename PointT>
Eigen::Vector4f ClusterMeasure<PointT>::getCentroid()
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid<PointT>(*cloud_, centroid);

	if(visualize_){
		pcl::visualization::PCLVisualizer viewer("Cluster centroid viewer");
	    viewer.addPointCloud(cloud_);
	    PointT p;
	    p.x = centroid[0];
	    p.y = centroid[1];
	    p.z = centroid[2];
	    viewer.addSphere(p, 0.01, 255, 0, 0, "centroid");
	    viewer.spin();
	}

	return centroid;
}

template<typename PointT>
void ClusterMeasure<PointT>::getAxis()
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid<PointT>(*cloud_, centroid);

	for(CloudIt it = cloud_->points.begin(); it < cloud_->points.end(); it++){
		it->x -= centroid[0];
		it->y -= centroid[1];
		it->z -= centroid[2];
	}

	pcl::PCA<PointT> pca;
	pca.setInputCloud(cloud_);
    pca.getEigenValues();
	pcl::visualization::PCLVisualizer viewer("Cluster measure viewer");
    viewer.addPointCloud(cloud_);
    PointT p; p.x = centroid[0]; p.y = centroid[1]; p.z = centroid[2];
    viewer.addSphere(p, 0.01, 255, 0, 0, "centroid");
    p.x=0; p.y=0; p.z=0;
    viewer.addSphere(p, 0.01, 0, 255, 0, "center");

    PointT v1, v2, v3;
    Eigen::Matrix3f m = pca.getEigenVectors();
    v1.x=m(0,0);v1.y=m(0,1);v1.z=m(0,2);
    v2.x=m(1,0);v2.y=m(1,1);v2.z=m(1,2);
    v3.x=m(2,0);v3.y=m(2,1);v3.z=m(2,2);

    //Showing UNIT vectors
    viewer.addLine(v1,p,1,0,0,"X");
    viewer.addLine(v2,p,0,1,0,"Y");
    viewer.addLine(v3,p,0,0,1,"Z");

    viewer.spin();
}

//AO BB
template<typename PointT>
void ClusterMeasure<PointT>::getOABBox(Eigen::Quaternionf & qfinal, Eigen::Vector3f & tfinal, float & width, float & height, float & depth)
{
    // compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_, centroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud_, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<PointT> cPoints;
    pcl::transformPointCloud(*cloud_, cPoints, p2w);

    //Obtain mean diag, which is the center of the BB and the object (not the same as centroid)
    PointT min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // final transform
    Eigen::Quaternionf qf(eigDx);
    qfinal = qf;
    tfinal = eigDx*mean_diag + centroid.head<3>();

    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    depth = max_pt.z - min_pt.z;

    if(visualize_){
		// draw the cloud and the box
		pcl::visualization::PCLVisualizer viewer;
		viewer.addPointCloud(cloud_);
		viewer.addCube(tfinal, qfinal, width, height, depth);
		viewer.spin();
    }

}

//1) compute the centroid (c0, c1, c2) and the normalized covariance
//2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
//3) move the points in that RF --- note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
//4) compute the max, the min and the center of the diagonal
//5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z) the transformation you have to apply is Rotation = (e0, e1, e0 X e1) & Translation = Rotation * center_diag + (c0, c1, c2)



#endif
