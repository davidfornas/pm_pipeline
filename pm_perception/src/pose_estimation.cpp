/*
 * sac_pose_estimation.cpp
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */
#include <pm_perception/pose_estimation.h>
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

// Compute Symmetry naive && Compute SQ Shape
void PoseEstimation::estimateSQ( CloudPtr & sq_cloud  ){
  //Keep only object cylinder
  CloudClustering<PointT> cluster(object_cloud_);
  cluster.applyEuclidianClustering();

  // ESTIMATON OF THE SYMMETRY PLANE
  CloudPtr full_model(new Cloud);
  PlaneSymmetryEstimation pse(cluster.cloud_clusters[0], bg_remove->cloud_plane, *bg_remove->coefficients_plane);
  pse.applyFurthest(full_model);
  PCLView<PointT>::showCloud(full_model);

  // Call to SQ Computing
  double min_fit = std::numeric_limits<double>::max ();
  sq::SuperquadricParameters<double> min_params;
  sq::SuperquadricFittingCeres<PointT, double> sq_fit;
  sq_fit.setInputCloud (full_model);

  for (int i = 0; i < 3; ++i)
  {
    sq::SuperquadricParameters<double> params;
    sq_fit.setPreAlign (true, i);
    double fit = sq_fit.fit (params);
    ROS_DEBUG("pre_align axis %d, fit %f\n", i, fit);
    if (fit < min_fit)
    {
      min_fit = fit;
      min_params = params;
    }
  }
  sq::SuperquadricSamplingUniform<PointT, double> sampling;
  sampling.setParameters (min_params);
  sampling.setSpatialSampling(0.01);
  sq_cloud = boost::shared_ptr<Cloud>(new Cloud());
  sampling.generatePointCloud (*sq_cloud);

  for (int j = 0; j < sq_cloud->points.size(); ++j) {
    sq_cloud->points[j].r = 128;
    sq_cloud->points[j].g = 0;
    sq_cloud->points[j].b = 0;
  }
}
