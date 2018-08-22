/*
 * sac_pose_estimation.cpp
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */

#include <pm_tools/timing.h>

#include <pm_perception/sq_pose_estimation.h>
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

bool SQPoseEstimation::process() {

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  bg_remove->setNewCloud(cloud_);
  bg_remove->initialize(cloud_, cloud_normals);

  cloud_clustering_ = new CloudClustering<PointT>(cloud_);
  if(use_region_growing_)
    cloud_clustering_->applyRegionGrowingClustering(region_growing_norm_th_, region_growing_curv_th_, cluster_thereshold_);
  else
    cloud_clustering_->applyEuclidianClustering(euclidian_tolerance_, cluster_thereshold_ );
  //Set to process first cluster only.
  cluster_index_ = -1;

  if (cloud_clustering_->cloud_clusters.size() == 0) return false;

  if(debug_) {
    ROS_INFO_STREAM("Cluster list: ");
    for (int i = 0; i < cloud_clustering_->cloud_clusters.size(); ++i) {
      ROS_INFO_STREAM("Cluster " << i << "size: " << cloud_clustering_->cloud_clusters[i]->points.size());
    }
    cloud_clustering_->displayColoured();
  }

  return processNext();

}

// @TODO Refactor...
bool SQPoseEstimation::processNext() {

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

  // Call to SQ Computing
  double min_fit = std::numeric_limits<double>::max ();
  sq::SuperquadricParameters<double> min_params;
  int i_aux;
  if(use_ceres_){
    sq::SuperquadricFittingCeres<PointT, double> sq_fit;
    sq_fit.setInputCloud (full_model);

    for (int i = 0; i < 3; ++i)
    {
      sq::SuperquadricParameters<double> params;
      sq_fit.setPreAlign (true, i);
      double fit = sq_fit.fit (params);
      ROS_INFO("pre_align axis %d, fit %f\n", i, fit);

      if (fit < min_fit)
      {
        min_fit = fit;
        min_params = params;
        i_aux = i;
      }
    }
  }else{
    sq::SuperquadricFittingLM<PointT, double> sq_fit;
    sq_fit.setInputCloud (full_model);

    for (int i = 0; i < 3; ++i)
    {
      sq::SuperquadricParameters<double> params;
      sq_fit.setPreAlign (true, i);
      double fit = sq_fit.fit (params);
      ROS_INFO("pre_align axis %d, fit %f\n", i, fit);

      if (fit < min_fit)
      {
        min_fit = fit;
        min_params = params;
        i_aux = i;
      }
    }
  }
  sq_params_ = min_params;

  ROS_INFO("Command for sampler:\n-e1 %f -e2 %f -a %f -b %f -c %f -transform %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
           min_params.e1, min_params.e2, min_params.a, min_params.b, min_params.c,
           min_params.transform (0, 0), min_params.transform (0, 1), min_params.transform (0, 2), min_params.transform (0, 3),
           min_params.transform (1, 0), min_params.transform (1, 1), min_params.transform (1, 2), min_params.transform (1, 3),
           min_params.transform (2, 0), min_params.transform (2, 1), min_params.transform (2, 2), min_params.transform (2, 3),
           min_params.transform (3, 0), min_params.transform (3, 1), min_params.transform (3, 2), min_params.transform (3, 3));

  vpHomogeneousMatrix transform = VispTools::EigenMatrixDouble44ToVpHomogeneousMatrix(min_params.transform).inverse();

  estimationStatsPublisher.publish(tick.getTotalTimeMsg());

  if(min_params.e1 == 0 && min_params.e2 == 0 && min_params.a == 1 && min_params.b == 1 && min_params.c ==1)
    return false;

  if(min_params.e1 < 0 || min_params.e2 < 0)
    return false;

  // Sampling: Uniform does not have Mesh export
  sq::SuperquadricSampling<PointT, double> sampling;
  //sq::SuperquadricSamplingUniform<PointT, double> sampling;
  sampling.setParameters (min_params);
  //Only for Uniform
  //sampling.setSpatialSampling(0.01);

  sq_cloud_ = boost::shared_ptr<Cloud>(new Cloud());
  sampling.generatePointCloud (*sq_cloud_);

  for (int j = 0; j < sq_cloud_->points.size(); ++j) {
    sq_cloud_->points[j].r = 128;
    sq_cloud_->points[j].g = 0;
    sq_cloud_->points[j].b = 0;
  }

  //Fix rotation ISSUE where SQ rotation matrix sometimes is weird.
  ClusterMeasure<PointT> cm(sq_cloud_, false);
  Eigen::Quaternionf q; Eigen::Vector3f t;
  Eigen::Matrix4f cMo_eigen; float width, height, depth;
  cMo_eigen = cm.getOABBox( q, t, width, height, depth );
  cMo = VispTools::EigenMatrix4fToVpHomogeneousMatrix(cMo_eigen) * vpHomogeneousMatrix(0, 0, 0, -1.57, 0, 0);
  cMo[0][3] = transform [0][3];
  cMo[1][3] = transform [1][3];
  cMo[2][3] = transform [2][3];

  object_cloud_ = full_model;
  publishResults();
  display();

  //Generate Object Mesh for UWSim
  sampling.generateMesh (sq_mesh_);
  pcl::io::saveOBJFile ("/home/dfornas/ros_ws/src/pm_pipeline/pm_perception/data/sq.obj", sq_mesh_);

  //Generate centered ObjectMesh for UWSim
  min_params.transform.setIdentity();
  sampling.setParameters (min_params);
  sampling.generateMesh (sq_mesh_);
  pcl::io::saveOBJFile ("/home/dfornas/ros_ws/src/pm_pipeline/pm_perception/data/sq_centered.obj", sq_mesh_);
  pcl::io::saveOBJFile ("/home/dfornas/Code/openrave_scripts/objects/sq_centered.obj", sq_mesh_);

  //"package://pm_perception/data/sq.obj"
  vpHomogeneousMatrix r(0,0,0,-1.57,0,0);
  UWSimMarkerPublisher::publishMeshMarker(r ,1, 1, 1, std::string("package://pm_perception/data/sq.obj"), cluster_index_ );
  return true;
}

void SQPoseEstimation::display( int ms ){
  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer("SQ Fitting");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> ob_h(object_cloud_, 0, 0, 120);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> sq_h(sq_cloud_, 160, 0, 0);
  p->setBackgroundColor(0.8,0.8,0.8);
  p->addPointCloud(object_cloud_, ob_h, "c1");
  p->addPointCloud(sq_cloud_, sq_h, "c2");
  int elapsed = 0;
  while (!p->wasStopped() && elapsed < ms )
  {
    elapsed += 20;
    p->spinOnce(20);
    boost::this_thread::sleep(boost::posix_time::microseconds(20000));
  }
}

void SQPoseEstimation::publishResults(){
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
  objectParameters.data.push_back(sq_params_.e1);
  objectParameters.data.push_back(sq_params_.e2);
  objectParameters.data.push_back(sq_params_.a);
  objectParameters.data.push_back(sq_params_.b);
  objectParameters.data.push_back(sq_params_.c);
  objectParameterPublisher.publish(objectParameters);

  ros::spinOnce();
  objectPosePublisher.publish( VispTools::geometryPoseFromVispHomog(cMo) );
  ros::spinOnce();
}
