/*
 * sac_pose_estimation.cpp
 *
 *  Created on: 29/09/2017
 *      Author: dfornas
 */
#include <pm_perception/pose_estimation.h>
#include <pm_perception/cluster_measure.h>
#include <pm_perception/symmetry.h>
#include <pm_tools/pcl_clustering.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

void BoxPoseEstimation::process() {

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
  if (cluster.cloud_clusters.size() == 0) return;
  // @TODO Compute position with centroid, orientation with plane directions (similar to cylinder axis...)

  // ESTIMATON OF THE SYMMETRY PLANE
  CloudPtr full_model(new Cloud);
  Eigen::Vector3f plane_origin, ground_plane_normal, box_plane_normal;

  ground_plane_normal.x() = bg_remove->coefficients_plane->values[0];
  ground_plane_normal.y() = bg_remove->coefficients_plane->values[1];
  ground_plane_normal.z() = bg_remove->coefficients_plane->values[2];

  box_plane_normal.x() = coefficients_plane->values[0];
  box_plane_normal.y() = coefficients_plane->values[1];
  box_plane_normal.z() = coefficients_plane->values[2];

  Eigen::Vector4f boxtop_centroid;
  pcl::compute3DCentroid<PointT>(*cluster.cloud_clusters[0], boxtop_centroid);
  Eigen::Vector3f boxtop_centroid_3f(boxtop_centroid.x(), boxtop_centroid.y(), boxtop_centroid.z());

  Eigen::Vector4f plane_centroid;
  pcl::compute3DCentroid<PointT>(*bg_remove->cloud_plane, plane_centroid);
  Eigen::Vector3f plane_centroid_3f(plane_centroid.x(), plane_centroid.y(), plane_centroid.z());

  Eigen::Vector3f boxtop_centroid_projected_into_plane;
  pcl::geometry::project(boxtop_centroid_3f, plane_centroid_3f, ground_plane_normal, boxtop_centroid_projected_into_plane);

  Eigen::Vector3f box_to_plane(boxtop_centroid_3f-boxtop_centroid_projected_into_plane);

  plane_origin.x() = boxtop_centroid_projected_into_plane[0]+ 0.5 * box_to_plane[0];
  plane_origin.y() = boxtop_centroid_projected_into_plane[1]+ 0.5 * box_to_plane[1];
  plane_origin.z() = boxtop_centroid_projected_into_plane[2]+ 0.5 * box_to_plane[2];

  MirrorCloud mc(cluster.cloud_clusters[0], plane_origin, ground_plane_normal);
  mc.apply(full_model);

  ClusterMeasure<PointT> cm(full_model, true);

  Eigen::Quaternionf q;
  Eigen::Vector3f t;
  Eigen::Matrix4f cMo_eigen;
  float width, height, depth;
  cMo_eigen = cm.getOABBox( q, t, width, height, depth );
  cMo = VispTools::EigenMatrix4fToVpHomogeneousMatrix(cMo_eigen) * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0) * vpHomogeneousMatrix(0, 0, 0, 0, 1.57, 0);
  UWSimMarkerPublisher::publishCubeMarker(cMo, height, depth, width);

  if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  }
  object_cloud_ = full_model;
}

void PCAPoseEstimation::process() {

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  bg_remove->setNewCloud(cloud_);
  bg_remove->initialize(cloud_, cloud_normals);

  CloudClustering<PointT> cluster(cloud_);
  cluster.applyEuclidianClustering();

  if(debug_) {
    ROS_INFO_STREAM("Cluster list: ");
    for (int i = 0; i < cluster.cloud_clusters.size(); ++i) {
      ROS_INFO_STREAM("Cluster " << i << "size: " << cluster.cloud_clusters[i]->points.size());
    }
    cluster.displayColoured();
    //cluster.save("euclidean");
    PCLView<PointT>::showCloud(cloud_);
    PCLView<PointT>::showCloud(cluster.cloud_clusters[0]);
  }

  if (cluster.cloud_clusters.size() == 0) return;

  int max_index;
  double max_dist;
  PCLTools<PointT>::findFurthest(cluster.cloud_clusters[0], bg_remove->coefficients_plane->values[0], bg_remove->coefficients_plane->values[1],
                                 bg_remove->coefficients_plane->values[2], bg_remove->coefficients_plane->values[3], max_index, max_dist);



  // ESTIMATON OF THE SYMMETRY PLANE
  CloudPtr full_model(new Cloud);
  Eigen::Vector3f plane_origin, ground_plane_normal, box_plane_normal;

  ground_plane_normal.x() = bg_remove->coefficients_plane->values[0];
  ground_plane_normal.y() = bg_remove->coefficients_plane->values[1];
  ground_plane_normal.z() = bg_remove->coefficients_plane->values[2];

  Eigen::Vector3f point_in_object(cluster.cloud_clusters[0]->points[max_index].x,
                                  cluster.cloud_clusters[0]->points[max_index].y,
                                  cluster.cloud_clusters[0]->points[max_index].z);

  Eigen::Vector4f plane_centroid;
  pcl::compute3DCentroid<PointT>(*bg_remove->cloud_plane, plane_centroid);
  Eigen::Vector3f plane_centroid_3f(plane_centroid.x(), plane_centroid.y(), plane_centroid.z());

  Eigen::Vector3f point_in_object_projected_into_plane;
  pcl::geometry::project(point_in_object, plane_centroid_3f, ground_plane_normal, point_in_object_projected_into_plane);

  Eigen::Vector3f object_to_plane(point_in_object-point_in_object_projected_into_plane);

  plane_origin.x() = point_in_object_projected_into_plane[0]+ 0.5 * object_to_plane[0];
  plane_origin.y() = point_in_object_projected_into_plane[1]+ 0.5 * object_to_plane[1];
  plane_origin.z() = point_in_object_projected_into_plane[2]+ 0.5 * object_to_plane[2];

  MirrorCloud mc(cluster.cloud_clusters[0], plane_origin, ground_plane_normal);
  mc.apply(full_model);

  ClusterMeasure<PointT> cm(full_model, debug_);
  Eigen::Quaternionf q;
  Eigen::Vector3f t;
  Eigen::Matrix4f cMo_eigen;
  float width, height, depth;
  cMo_eigen = cm.getOABBox( q, t, width, height, depth );
  cMo = VispTools::EigenMatrix4fToVpHomogeneousMatrix(cMo_eigen) * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0) * vpHomogeneousMatrix(0, 0, 0, 0, 1.57, 0);
  UWSimMarkerPublisher::publishCubeMarker(cMo, height, depth, width);

  if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  }
  ROS_INFO_STREAM("PCA Object. Width: " << width << ". Height: " << height << "Depth: " << depth);
  object_cloud_ = full_model;
}

void CylinderPoseEstimation::initialize() {

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
  if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  }
  object_cloud_ = cloud_cylinder;

}

void CylinderPoseEstimation::process() {

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
  if(cloud_cylinder->size() < 100) return;

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
  if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  }
  vpHomogeneousMatrix cylinder;
  cylinder = cMo * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0);
  UWSimMarkerPublisher::publishCylinderMarker(cylinder ,radious,radious,height);
  object_cloud_ = cloud_cylinder;
  // @TODO Filter may be here or not. To avoid bad  detections.
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

void SpherePoseEstimation::initialize(){
  process();
}

void SpherePoseEstimation::process() {

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
  if(debug_) {
    vispToTF.resetTransform(cMo, "cMo");
    vispToTF.publish();
  }
  vpHomogeneousMatrix sphere;
  sphere = cMo ;//* vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0);
  UWSimMarkerPublisher::publishSphereMarker(sphere ,radious,radious,radious);
  object_cloud_ = cloud_sphere;

}



