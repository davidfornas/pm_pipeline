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

bool BoxPoseEstimation::process() {

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  bg_remove->setNewCloud(cloud_);
  bg_remove->initialize(cloud_, cloud_normals);

  CloudPtr output (new Cloud), cloud_plane (new Cloud);
  pcl::PointCloud<pcl::Normal>::Ptr output_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;
  PCLTools<PointT>::estimateNormals(cloud_, cloud_normals);

  PlaneSegmentation<PointT> plane_seg(cloud_, cloud_normals);
  plane_seg.setDistanceThreshold(0.04);
  //plane_seg.setIterations(plane_iterations_);
  plane_seg.apply(output, output_normals, cloud_plane, coefficients_plane);
  CloudClustering<PointT> cluster(cloud_plane);
  cluster.applyEuclidianClustering();
  cluster.displayColoured();
  // PCA
  if (cluster.cloud_clusters.size() == 0) return false;
  // @TODO Compute position with centroid, orientation with plane directions (similar to cylinder axis...)

  // ESTIMATON OF THE SYMMETRY PLANE
  CloudPtr full_model(new Cloud);
  PlaneSymmetryEstimation pse(cluster.cloud_clusters[0], bg_remove->cloud_plane, *bg_remove->coefficients_plane);
  pse.applyCentroid(full_model);

  ClusterMeasure<PointT> cm(full_model, true);

  Eigen::Quaternionf q;
  Eigen::Vector3f t;
  Eigen::Matrix4f cMo_eigen;
  float width, height, depth;
  cMo_eigen = cm.getOABBox( q, t, width, height, depth );
  cMo = VispTools::EigenMatrix4fToVpHomogeneousMatrix(cMo_eigen) * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0) * vpHomogeneousMatrix(0, 0, 0, 0, 1.57, 0);
  UWSimMarkerPublisher::publishCubeMarker(cMo, height, depth, width);

  //if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  //}
  object_cloud_ = full_model;
  return true;
}

bool PCAPoseEstimation::process() {

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  bg_remove->setNewCloud(cloud_);
  bg_remove->initialize(cloud_, cloud_normals);

  cloud_clustering_ = new CloudClustering<PointT>(cloud_);
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

// @TODO Refactor...
bool PCAPoseEstimation::processNext() {

  cluster_index_++;
  if (cluster_index_ >= cloud_clustering_->cloud_clusters.size()) return false;
  // @TODO Find best value. 400 for now. Stone is 300.
  if (cloud_clustering_->cloud_clusters[cluster_index_]->points.size() < cluster_thereshold_) return false;

  // ESTIMATON OF THE SYMMETRY PLANE
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

  ClusterMeasure<PointT> cm(full_model, debug_);
  Eigen::Quaternionf q;
  Eigen::Vector3f t;
  Eigen::Matrix4f cMo_eigen;
  float width, height, depth;
  cMo_eigen = cm.getOABBox( q, t, width, height, depth );
  cMo = VispTools::EigenMatrix4fToVpHomogeneousMatrix(cMo_eigen) * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0) * vpHomogeneousMatrix(0, 0, 0, 0, 3.1416, 0);
  UWSimMarkerPublisher::publishCubeMarker(cMo, height, depth, width);

  //if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  //}
  ROS_INFO_STREAM("PCA Object. Width: " << width << ". Height: " << height << "Depth: " << depth);
  object_cloud_ = full_model;
  return true;

}

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

  if(min_params.e1 == 0 && min_params.e2 == 0 && min_params.a == 1 && min_params.b == 1 && min_params.c ==1)
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

  //if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  //}
  object_cloud_ = full_model;
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

bool CylinderPoseEstimation::initialize() {

  CloudPtr cloud_filtered (new Cloud), cloud_filtered2 (new Cloud), cloud_cylinder (new Cloud);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>), cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  coefficients_cylinder = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;

  // @ TODO : Add more filters -> downsampling and radial ooutlier removal.
  PCLTools<PointT>::applyZAxisPassthrough(cloud_, cloud_filtered, 0, 3);
  ROS_DEBUG_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");
  bg_remove->setNewCloud(cloud_filtered);
  bg_remove->initialize(cloud_filtered2, cloud_normals2);

  CylinderSegmentation<PointT> cyl_seg(cloud_filtered2, cloud_normals2);
  cyl_seg.setDistanceThreshold(cylinder_distance_threshold_);
  cyl_seg.setIterations(cylinder_iterations_);
  cyl_seg.setRadiousLimit(radious_limit_);
  cyl_seg.apply(cloud_cylinder, coefficients_cylinder);
  //coefficients_cylinder = PCLTools::cylinderSegmentation(cloud_filtered2, cloud_normals2, cloud_cylinder, cylinder_distance_threshold_, cylinder_iterations_, radious_limit_);
  //PCLTools<PointT>::showSegmentationCloudsAndModels(cloud_plane, cloud_cylinder, coefficients_plane, coefficients_cylinder);

  //Grasp points
  PointT mean, max, min;
  //Cylinder origin
  PointT axis_point;
  axis_point.x=coefficients_cylinder->values[0];
  axis_point.y=coefficients_cylinder->values[1];
  axis_point.z=coefficients_cylinder->values[2];

  //Director vectors: cylinder axis and perpendicular vector.
  tf::Vector3 axis_dir(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);
  axis_dir=axis_dir.normalize();

  tf::Vector3 plane_normal(bg_remove->coefficients_plane->values[0], bg_remove->coefficients_plane->values[1], bg_remove->coefficients_plane->values[2]);
  tf::Vector3 perp = plane_normal - ( axis_dir.dot( plane_normal ) * axis_dir );
  perp=perp.normalize();

  tf::Vector3 result=tf::tfCross( perp, axis_dir).normalize();
  result = -result;

  getMinMax3DAlongAxis(cloud_cylinder, &max, &min, axis_point, &axis_dir, 0.1);
  //Mean point taking into account only a 90% of the points (0.1).
  mean.x=(max.x+min.x)/2;mean.y=(max.y+min.y)/2;mean.z=(max.z+min.z)/2;
  coefficients_cylinder->values[0]=mean.x;
  coefficients_cylinder->values[1]=mean.y;
  coefficients_cylinder->values[2]=mean.z;

  //Cylinder properties
  radious=coefficients_cylinder->values[6];
  height=sqrt((max.x-min.x)*(max.x-min.x)+(max.y-min.y)*(max.y-min.y)+(max.z-min.z)*(max.z-min.z));

  // @ NOTE Ahora mismo el end-efector cae dentro del cilindro en vez de en superfície.
  //Esto está relativamente bien pero no tenemos en cuenta la penetración. Sin embargo, la
  //tenemos en cuenta luego al separanos el radio así que no hay problema en realidad.
  cMo[0][0]=result.x(); cMo[0][1]=axis_dir.x(); cMo[0][2]=perp.x();cMo[0][3]=mean.x;
  cMo[1][0]=result.y(); cMo[1][1]=axis_dir.y(); cMo[1][2]=perp.y();cMo[1][3]=mean.y;
  cMo[2][0]=result.z(); cMo[2][1]=axis_dir.z(); cMo[2][2]=perp.z();cMo[2][3]=mean.z;
  cMo[3][0]=0;cMo[3][1]=0;cMo[3][2]=0;cMo[3][3]=1;
  ROS_INFO_STREAM("cMo is...: " << std::endl << cMo );
  //if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  //}
  object_cloud_ = cloud_cylinder;
  return true;

}

bool CylinderPoseEstimation::process() {

  CloudPtr cloud_filtered (new Cloud), cloud_filtered2 (new Cloud);
  CloudPtr cloud_cylinder (new Cloud), cloud_plane (new Cloud);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>), cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

  coefficients_cylinder = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;
  PCLTools<PointT>::applyZAxisPassthrough(cloud_, cloud_filtered, 0, 3);
  ROS_INFO_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");
  bg_remove->setNewCloud(cloud_filtered);
  bg_remove->initialize(cloud_filtered2, cloud_normals2);

  CylinderSegmentation<PointT> cyl_seg(cloud_filtered2, cloud_normals2);
  cyl_seg.setDistanceThreshold(cylinder_distance_threshold_);
  cyl_seg.setIterations(cylinder_iterations_);
  cyl_seg.setRadiousLimit(radious+0.01);

  cyl_seg.apply(cloud_cylinder, coefficients_cylinder);
  ROS_INFO_STREAM("Found cylinder size: " << cloud_cylinder->size());

  // @TODO return boolean and treat it later
  if(cloud_cylinder->size() < 100) return false;

  //Grasp points
  PointT mean, max, min;
  //Cylinder origin
  PointT axis_point;
  axis_point.x=coefficients_cylinder->values[0];
  axis_point.y=coefficients_cylinder->values[1];
  axis_point.z=coefficients_cylinder->values[2];

  //Director vectors: cylinder axis and perpendicular vector.
  tf::Vector3 axis_dir(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);
  axis_dir=axis_dir.normalize();

  //Add consistency to the axis direction.
  tf::Vector3 camera_y_axis(0, 1, 0), axis_opposite_dir;
  axis_opposite_dir = -axis_dir;
  if(axis_dir.angle(camera_y_axis) < axis_opposite_dir.angle(camera_y_axis))
    axis_dir = axis_opposite_dir;

  tf::Vector3 plane_normal(bg_remove->coefficients_plane->values[0], bg_remove->coefficients_plane->values[1], bg_remove->coefficients_plane->values[2]);
  tf::Vector3 perp = plane_normal - ( axis_dir.dot( plane_normal ) * axis_dir );
  perp=perp.normalize();

  tf::Vector3 result=tf::tfCross( perp, axis_dir).normalize();
  result = -result;

  getMinMax3DAlongAxis(cloud_cylinder, &max, &min, axis_point, &axis_dir, 0.1);
  //Mean point taking into account only a 90% of the points (0.1).
  mean.x=(max.x+min.x)/2;mean.y=(max.y+min.y)/2;mean.z=(max.z+min.z)/2;
  coefficients_cylinder->values[0]=mean.x;
  coefficients_cylinder->values[1]=mean.y;
  coefficients_cylinder->values[2]=mean.z;

  //Cylinder properties
  radious=coefficients_cylinder->values[6];
  height=sqrt((max.x-min.x)*(max.x-min.x)+(max.y-min.y)*(max.y-min.y)+(max.z-min.z)*(max.z-min.z));

  cMo[0][0]=result.x(); cMo[0][1]=axis_dir.x(); cMo[0][2]=perp.x();cMo[0][3]=mean.x;
  cMo[1][0]=result.y(); cMo[1][1]=axis_dir.y(); cMo[1][2]=perp.y();cMo[1][3]=mean.y;
  cMo[2][0]=result.z(); cMo[2][1]=axis_dir.z(); cMo[2][2]=perp.z();cMo[2][3]=mean.z;
  cMo[3][0]=0;cMo[3][1]=0;cMo[3][2]=0;cMo[3][3]=1;
  //if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  //}
  vpHomogeneousMatrix cylinder;
  cylinder = cMo * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0);
  UWSimMarkerPublisher::publishCylinderMarker(cylinder ,radious,radious,height);
  object_cloud_ = cloud_cylinder;
  // @TODO Filter may be here or not. To avoid bad  detections.
  return true;
}

//Ordenar en función de la proyección del punto sobre el eje definido
//por axis_point_g y normal_g (globales)
bool CylinderPoseEstimation::sortFunction(const PointT& d1, const PointT& d2)
{
  double t1 = (normal_g.x()*(d1.x-axis_point_g.x) + normal_g.y()*(d1.y-axis_point_g.y) + normal_g.z()*(d1.z-axis_point_g.z))/(pow(normal_g.x(),2) + pow(normal_g.y(),2) + pow(normal_g.z(),2));
  double t2 = (normal_g.x()*(d2.x-axis_point_g.x) + normal_g.y()*(d2.y-axis_point_g.y) + normal_g.z()*(d2.z-axis_point_g.z))/(pow(normal_g.x(),2) + pow(normal_g.y(),2) + pow(normal_g.z(),2));
  return t1 < t2;
}

//Obtiene los máximos y mínimos del cilindro para encontrar la altura del cilindro con un margen
//de descarte de outlier percentage (normalmente 10%).
void CylinderPoseEstimation::getMinMax3DAlongAxis(const pcl::PointCloud<PointT>::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT axis_point, tf::Vector3 * normal, double outlier_percentage)
{
  axis_point_g=axis_point;
  normal_g=*normal;
  outlier_percentage = outlier_percentage / 2.0;

  PointT max_p = axis_point;
  double max_t = 0.0;
  PointT min_p = axis_point;
  double min_t = 0.0;
  std::vector<PointT> list;
  PointT* k;

  //Al tener la lista de todos los puntos podemos descartar los que esten fuera de un
  //determinado porcentaje (percentiles) para eliminar más outliers y ganar robustez.
  BOOST_FOREACH(const PointT& pt, cloud->points)
  {
    k=new PointT();
    k->x=pt.x*1;k->y=pt.y*1;k->z=pt.z*1;
    list.push_back(*k);
  }
  //Ordenamos con respecto al eje de direccion y tomamos P05 y P95
  std::sort(list.begin(), list.end(),  boost::bind(&CylinderPoseEstimation::sortFunction, this, _1, _2));
  PointT max=list[(int)list.size()*outlier_percentage],min=list[(int)list.size()*(1-outlier_percentage)];
  //Proyección de los puntos reales a puntos sobre la normal.
  double t = (normal->x()*(max.x-axis_point.x) + normal->y()*(max.y-axis_point.y) + normal->z()*(max.z-axis_point.z))/(pow(normal->x(),2) + pow(normal->y(),2) + pow(normal->z(),2));
  PointT p;
  p.x = axis_point.x + normal->x() * t;
  p.y = axis_point.y + normal->y() * t;
  p.z = axis_point.z + normal->z() * t;
  *max_pt=p;
  t = (normal->x()*(min.x-axis_point.x) + normal->y()*(min.y-axis_point.y) + normal->z()*(min.z-axis_point.z))/(pow(normal->x(),2) + pow(normal->y(),2) + pow(normal->z(),2));
  p.x = axis_point.x + normal->x() * t;
  p.y = axis_point.y + normal->y() * t;
  p.z = axis_point.z + normal->z() * t;
  *min_pt=p;
}

bool SpherePoseEstimation::initialize(){
  return process();
}

bool SpherePoseEstimation::process() {

  CloudPtr cloud_filtered (new Cloud), cloud_filtered2 (new Cloud), cloud_sphere (new Cloud);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>), cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  coefficients_sphere = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;

  // @ TODO : Add more filters -> downsampling and radial ooutlier removal.
  PCLTools<PointT>::applyZAxisPassthrough(cloud_, cloud_filtered, 0, 3);
  ROS_DEBUG_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");
  bg_remove->setNewCloud(cloud_filtered);
  bg_remove->initialize(cloud_filtered2, cloud_normals2);

  SphereSegmentation<PointT> sph_seg(cloud_filtered2, cloud_normals2);
  sph_seg.setDistanceThreshold(sphere_distance_threshold_);
  sph_seg.setIterations(sphere_iterations_);
  sph_seg.setRadiousLimit(radious_limit_);
  sph_seg.apply(cloud_sphere, coefficients_sphere);

  PCLView<PointT>::showCloud(cloud_sphere);
  PCLView<PointT>::showCloud(bg_remove->cloud_plane);
  ROS_INFO_STREAM("DEBUG" << cloud_sphere->points.size());

  Eigen::Vector3f sphere_centre;
  sphere_centre.x() = coefficients_sphere->values[0];
  sphere_centre.y() = coefficients_sphere->values[1];
  sphere_centre.z() = coefficients_sphere->values[2];
  radious = coefficients_sphere->values[3];

  Eigen::Vector3f plane_origin, ground_plane_normal;

  ground_plane_normal.x() = bg_remove->coefficients_plane->values[0];
  ground_plane_normal.y() = bg_remove->coefficients_plane->values[1];
  ground_plane_normal.z() = bg_remove->coefficients_plane->values[2];

  Eigen::Vector4f plane_centroid;
  pcl::compute3DCentroid<PointT>(*bg_remove->cloud_plane, plane_centroid);
  Eigen::Vector3f plane_centroid_3f(plane_centroid.x(), plane_centroid.y(), plane_centroid.z());

  Eigen::Vector3f sphere_centre_projected;
  pcl::geometry::project(sphere_centre, plane_centroid_3f, ground_plane_normal, sphere_centre_projected);

  Eigen::Vector3f ground_plane_vector(sphere_centre_projected-plane_centroid_3f);
  ground_plane_vector.normalize();
  ground_plane_normal.normalize();

  tf::Vector3 tf_ground_plane_vector(ground_plane_vector.x(), ground_plane_vector.y(), ground_plane_vector.z());
  tf::Vector3 tf_ground_plane_normal(ground_plane_normal.x(), ground_plane_normal.y(), ground_plane_normal.z());
  tf::Vector3 result=tf::tfCross( tf_ground_plane_vector, tf_ground_plane_normal).normalize();
  result = -result;

  cMo[0][0]=result.x(); cMo[0][1]=ground_plane_normal.x(); cMo[0][2]=ground_plane_vector.x();cMo[0][3]=sphere_centre.x();
  cMo[1][0]=result.y(); cMo[1][1]=ground_plane_normal.y(); cMo[1][2]=ground_plane_vector.y();cMo[1][3]=sphere_centre.y();
  cMo[2][0]=result.z(); cMo[2][1]=ground_plane_normal.z(); cMo[2][2]=ground_plane_vector.z();cMo[2][3]=sphere_centre.z();
  cMo[3][0]=0;cMo[3][1]=0;cMo[3][2]=0;cMo[3][3]=1;
  ROS_INFO_STREAM("cMo is...: " << std::endl << cMo );
  //if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  //}
  vpHomogeneousMatrix sphere;
  sphere = cMo ;//* vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0);
  UWSimMarkerPublisher::publishSphereMarker(sphere ,radious,radious,radious);
  object_cloud_ = cloud_sphere;
  return true;
}


