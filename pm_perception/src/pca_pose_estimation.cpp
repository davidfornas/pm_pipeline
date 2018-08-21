/*
 * sac_pose_estimation.cpp
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */

#include <pm_tools/timing.h>

#include <pm_perception/pca_pose_estimation.h>
#include <pm_perception/cluster_measure.h>
#include <pm_perception/symmetry.h>

#include <pm_superquadrics/fit_superquadric_lm.h>
#include <pm_superquadrics/fit_superquadric_ceres.h>
#include <pm_superquadrics/sample_superquadric.h>
#include <pm_superquadrics/sample_superquadric_uniform.h>

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

bool PCAPoseEstimation::process() {

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  bg_remove->setNewCloud(cloud_);
  bg_remove->initialize(cloud_, cloud_normals);

  cloud_clustering_ = new CloudClustering<PointT>(nh_, cloud_);
  cloud_clustering_->applyEuclidianClustering();
  //Set to -1 because processNext increases the count..
  cluster_index_ = -1;

  if(debug_) {
    ROS_INFO_STREAM("Cluster list: ");
    for (int i = 0; i < cloud_clustering_->cloud_clusters.size(); ++i) {
      ROS_INFO_STREAM("Cluster " << i << "size: " << cloud_clustering_->cloud_clusters[i]->points.size());
    }
    cloud_clustering_->displayColoured();
    //cluster.save("euclidean");
    //PCLView<PointT>::showCloud(cloud_);
    //PCLView<PointT>::showCloud(cloud_clustering_->cloud_clusters[0]);
  }
  return processNext();
}

bool PCAPoseEstimation::processNext() {

  cluster_index_++;
  if (cluster_index_ >= cloud_clustering_->cloud_clusters.size()) return false;
  // @TODO Find best value. 400 for now. Stone is 300.
  if (cloud_clustering_->cloud_clusters[cluster_index_]->points.size() < cluster_thereshold_) return false;

  // ESTIMATON OF THE SYMMETRY PLANE
  ProgramTimer tick;
  CloudPtr full_model(new Cloud);
  if(planar_symmetry_) {
    PlaneSymmetryEstimation pse(cloud_clustering_->cloud_clusters[cluster_index_], bg_remove->cloud_plane,
                                *bg_remove->coefficients_plane);
    if (symmetry_search_) {
      pse.setSearchParams(angle_limit_, angle_step_, distance_ratio_step_);
      pse.searchBest(full_model, fixed_half_height_);
    } else {
      pse.applyCentroid(full_model);
    }

  }else{
    AxisSymmetryEstimation ase(cloud_clustering_->cloud_clusters[cluster_index_], bg_remove->cloud_plane, *bg_remove->coefficients_plane);
    if (symmetry_search_) {
      ase.setSearchParams(angle_limit_, angle_step_, distance_ratio_step_);
      ase.searchBest(full_model, fixed_half_height_);
    } else {
      ase.estimateAndApply(full_model);
    }
  }
  symmetryStatsPublisher.publish(tick.getLapTimeMsg());

  ClusterMeasure<PointT> cm(full_model, debug_);
  Eigen::Quaternionf q;
  Eigen::Vector3f t;
  Eigen::Matrix4f cMo_eigen;
  cMo_eigen = cm.getOABBox( q, t, width_, height_, depth_ );
  cMo = VispTools::EigenMatrix4fToVpHomogeneousMatrix(cMo_eigen) * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0) * vpHomogeneousMatrix(0, 0, 0, 0, 3.1416, 0);
  estimationStatsPublisher.publish(tick.getTotalTimeMsg());

  object_cloud_ = full_model;

  publishResults();
  return true;

}

void PCAPoseEstimation::publishResults() {
  UWSimMarkerPublisher::publishCubeMarker(cMo, height_, depth_, width_);

  vispToTF.resetTransform(cMo, "cMo");
  vispToTF.publish();

  sensor_msgs::PointCloud2 message;
  pcl::PCLPointCloud2 pcl_pc;
  pcl::toPCLPointCloud2(*object_cloud_, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, message);
  message.header.frame_id = "stereo";
  objectCloudPublisher.publish(message);
  objectCloudSizePublisher.publish(ProgramTimer::toFloat32Msgs(object_cloud_->points.size()));

  std_msgs::Float32MultiArray objectParameters;
  objectParameters.data.push_back(width_);
  objectParameters.data.push_back(height_);
  objectParameters.data.push_back(depth_);
  objectParameterPublisher.publish(objectParameters);

  ros::spinOnce();
  objectPosePublisher.publish( VispTools::geometryPoseFromVispHomog(cMo) );
  ros::spinOnce();
  ROS_INFO_STREAM("PCA Object. Width: " << width_ << ". Height: " << height_ << "Depth: " << depth_);
}
