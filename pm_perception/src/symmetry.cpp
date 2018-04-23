/*
 * symmetry
 * 			 
 *  Created on: 28/03/2018
 *      Author: dfornas
 */
#include <pm_perception/symmetry.h>
#include <visp/vpHomogeneousMatrix.h>
#include <pm_tools/visp_tools.h>

void SymmetryEstimation::display() {

  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer("Mirror colored");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h(cloud_, 0, 230, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h(mirror_, 230, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> pjt_h(projection_, 0, 0, 230);
  p->setBackgroundColor(0.8,0.8,0.8);
  p->addPointCloud(cloud_, tgt_h, "c1");
  p->addPointCloud(mirror_, src_h, "c2");
  p->addPointCloud(projection_, pjt_h, "c3");
  p->addCoordinateSystem(0.1, 0, 0, 0);
  p->spin();
}

void SymmetryEstimation::displayMirrored() {

  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer("Result");
  p->setBackgroundColor(0.8,0.8,0.8);
  p->addPointCloud(mirrored_);
  p->addCoordinateSystem(0.1, 0, 0, 0);
  p->spin();
}

double PlaneSymmetryEstimation::apply( CloudPtr & mirrored ) {

  //Cluster with Z=0 for Score computing
  CloudPtr cluster_mask( new Cloud);
  pcl::copyPointCloud(*cloud_, *cluster_mask);
  for(int i=0; i < cluster_mask->points.size(); i++) cluster_mask->points[i].z = 0;
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cluster_mask);

  mirrored_ = boost::shared_ptr<Cloud>(new Cloud());
  mirror_ = boost::shared_ptr<Cloud>(new Cloud());
  projection_ = boost::shared_ptr<Cloud>(new Cloud());
  pcl::copyPointCloud(*cloud_, *mirrored_);

  ClusterMeasure<PointT> cm(cloud_);

  //Lower is better
  double score = 0.;

  for (int i = 0; i < cloud_->points.size(); ++i) {

    Eigen::Vector3f pt;
    Eigen::Vector3f origin_point(cloud_->points[i].x, cloud_->points[i].y, cloud_->points[i].z);

    pcl::geometry::project(origin_point, plane_origin_, plane_normal_, pt);

    PointT projected(cloud_->points[i]), mirrored(cloud_->points[i]);
    projected.x = pt.x();
    projected.y = pt.y();
    projected.z = pt.z();

    mirrored.x = -cloud_->points[i].x + 2 * pt.x();
    mirrored.y = -cloud_->points[i].y + 2 * pt.y();
    mirrored.z = -cloud_->points[i].z + 2 * pt.z();

    mirror_->push_back(mirrored);
    mirrored_->push_back(mirrored);
    projection_->push_back(projected);

    // SCORE COMPUTING
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    PointT mirrored_z0;
    mirrored_z0 = mirrored;
    mirrored_z0.z = 0;
    kdtree.nearestKSearch(mirrored_z0, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    PointT closest_point = cloud_->points[pointIdxNKNSearch[0]];

    //Si no tiene un punto muy cercano, está fuera de la oclusión del objeto por lo que
    // no deberia añadirse, deberia verse ya.
    if(sqrt(pointNKNSquaredDistance[0])>0.01){
      score += 3 * sqrt(pointNKNSquaredDistance[0]);
    }else if(mirrored.z < closest_point.z){//Si está más cerca que su punto correspondiente deberia verse de antes
      score += 2.5 * closest_point.z - mirrored.z;
    }else{//Así sólo añado los puntos que están estrictamente detrás
      score += mirrored.z - closest_point.z;
    }

  }
  score /= cloud_->points.size();

  //if(score > 0.001)
  mirrored = mirrored_;
  return score;

}

double PlaneSymmetryEstimation::searchBest( CloudPtr & mirrored, bool fixed_half_height ) {

  ROS_INFO_STREAM("Starting symmetry plane search.");
  double best_score = 100;
  CloudPtr best_cloud(new Cloud);

  // Compute the furthest point
  estimateFurthest();
  for ( double d = 0; d < 1; d += distance_ratio_step_ ) {
    for (double y = -angle_limit_; y <= angle_limit_; y += angle_step_) {
      for (double z = -angle_limit_; z <= angle_limit_; z += angle_step_) {
        CloudPtr aux_cloud(new Cloud);

        //Copute the new plane at r*(further_point-bkgrond_point)
        if(fixed_half_height)
          estimatePlane(0.5, 0, y, z);
        else
          estimatePlane(d, 0, y, z);

        double score = apply(aux_cloud);
        if (score < best_score) {
          best_score = score;
          pcl::copyPointCloud(*aux_cloud, *best_cloud);
        }
        //display();
      }
    }
    ROS_INFO_STREAM("Search at " << (d+distance_ratio_step_) * 100<< "percent.");
  }
  ROS_INFO_STREAM("Best (min) plane symmetry score: " << best_score );
  pcl::copyPointCloud( *best_cloud, *mirrored );
  return best_score;

}

void PlaneSymmetryEstimation::estimateFurthest(){

  //Obtain point with most distance to plane
  int max_index;
  double max_dist;
  PCLTools<PointT>::findFurthest(cloud_, plane_coeffs_.values[0], plane_coeffs_.values[1],
                                 plane_coeffs_.values[2], plane_coeffs_.values[3], max_index, max_dist);

//  Eigen::Vector3f furthest_point_in_object(cloud_->points[max_index].x, cloud_->points[max_index].y, cloud_->points[max_index].z);
  reference_point_ = Eigen::Vector3f(cloud_->points[max_index].x, cloud_->points[max_index].y, cloud_->points[max_index].z);
  estimatePlane();
}

void PlaneSymmetryEstimation::estimateCentroid(){

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid<PointT>(*cloud_, centroid);
//  Eigen::Vector3f centroid_3f(centroid.x(), centroid.y(), centroid.z());
  reference_point_ = Eigen::Vector3f(centroid.x(), centroid.y(), centroid.z());
  estimatePlane();
}

void PlaneSymmetryEstimation::estimatePlane( double distance_ratio, double x, double y, double z ){
  //Same plane normal as background
  plane_normal_.x() = plane_coeffs_.values[0];
  plane_normal_.y() = plane_coeffs_.values[1];
  plane_normal_.z() = plane_coeffs_.values[2];
  VispTools::rotateVector(plane_normal_, x, y, z);

  Eigen::Vector4f plane_centroid;
  pcl::compute3DCentroid<PointT>(*plane_cloud_, plane_centroid);
  Eigen::Vector3f plane_centroid_3f(plane_centroid.x(), plane_centroid.y(), plane_centroid.z());

  Eigen::Vector3f point_in_object_projected_into_plane;
  pcl::geometry::project(reference_point_, plane_centroid_3f, plane_normal_, point_in_object_projected_into_plane);

  Eigen::Vector3f object_to_plane(reference_point_-point_in_object_projected_into_plane);

  plane_origin_.x() = point_in_object_projected_into_plane[0]+ distance_ratio * object_to_plane[0];
  plane_origin_.y() = point_in_object_projected_into_plane[1]+ distance_ratio * object_to_plane[1];
  plane_origin_.z() = point_in_object_projected_into_plane[2]+ distance_ratio * object_to_plane[2];
}

double AxisSymmetryEstimation::searchBest( CloudPtr & mirrored, bool fixed_half_height ) {

  ROS_INFO_STREAM("Starting symmetry axis search.");
  double best_score = 100;
  CloudPtr best_cloud(new Cloud);

  for ( double d = 0; d < 1; d += distance_ratio_step_ ) {
    for (double y = -angle_limit_; y <= angle_limit_; y += angle_step_) {
      for (double z = -angle_limit_; z <= angle_limit_; z += angle_step_) {
        CloudPtr aux_cloud(new Cloud);

        //Copute the new plane at r*(centroid-bkgrond_point)
        if(fixed_half_height)
          estimateAxis(0.5, 0, y, z);
        else
          estimateAxis(d, 0, y, z);

        double score = apply(aux_cloud);
        if (score < best_score) {
          best_score = score;
          pcl::copyPointCloud(*aux_cloud, *best_cloud);
        }
      }
    }
    ROS_INFO_STREAM("Search at " << (d+distance_ratio_step_) * 100<< "percent.");
  }
  ROS_INFO_STREAM("Best (min) axis symmetry score: " << best_score );
  pcl::copyPointCloud( *best_cloud, *mirrored );

  return best_score;
}

double AxisSymmetryEstimation::apply( CloudPtr & mirrored ) {

  //Cluster with Z=0 for Score computing
  CloudPtr cluster_mask( new Cloud);
  pcl::copyPointCloud(*cloud_, *cluster_mask);
  for(int i=0; i < cluster_mask->points.size(); i++) cluster_mask->points[i].z = 0;
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cluster_mask);

  mirrored_ = boost::shared_ptr<Cloud>(new Cloud());
  mirror_ = boost::shared_ptr<Cloud>(new Cloud());
  projection_ = boost::shared_ptr<Cloud>(new Cloud());
  pcl::copyPointCloud(*cloud_, *mirrored_);

  mirrored = cloud_;
  ClusterMeasure<PointT> cm(cloud_);
  double score = 0.;
  for (int i = 0; i < cloud_->points.size(); ++i) {

    Eigen::Vector3f pt;
    Eigen::Vector3f origin_point(cloud_->points[i].x, cloud_->points[i].y, cloud_->points[i].z);

    pt = PCLTools<PointT>::projectPoint(origin_point, line_origin_, line_direction_);

    PointT projected(cloud_->points[i]), mirrored(cloud_->points[i]);
    projected.x = pt.x();
    projected.y = pt.y();
    projected.z = pt.z();

    mirrored.x = -cloud_->points[i].x + 2 * pt.x();
    mirrored.y = -cloud_->points[i].y + 2 * pt.y();
    mirrored.z = -cloud_->points[i].z + 2 * pt.z();

    mirror_->push_back(mirrored);
    mirrored_->push_back(mirrored);
    projection_->push_back(projected);

    // SCORE COMPUTING
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    PointT mirrored_z0;
    mirrored_z0 = mirrored;
    mirrored_z0.z = 0;
    kdtree.nearestKSearch(mirrored_z0, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    PointT closest_point = cloud_->points[pointIdxNKNSearch[0]];

    //Si no tiene un punto muy cercano, está fuera de la oclusión del objeto por lo que
    // no deberia añadirse, deberia verse ya.
    if(sqrt(pointNKNSquaredDistance[0])>0.01){
      score += 3 * sqrt(pointNKNSquaredDistance[0]);
    }else if(mirrored.z < closest_point.z){//Si está más cerca que su punto correspondiente deberia verse de antes
      score += 2.5 * closest_point.z - mirrored.z;
    }else{//Así sólo añado los puntos que están estrictamente detrás
      score += mirrored.z - closest_point.z;
    }
  }
  score /= cloud_->points.size();

  mirrored = mirrored_;

  ROS_DEBUG_STREAM("Squared distance mean to axis for mirrored points: " << score);
  return score;
}

void AxisSymmetryEstimation::estimateAxis( double distance_ratio, double x, double y, double z ){

  ClusterMeasure<PointT> cm(cloud_, false);
  Eigen::Matrix4f cMo;
  cMo = cm.getOABBox();

  line_origin_.x() = cMo(0,3);
  line_origin_.y() = cMo(1,3);
  line_origin_.z() = cMo(2,3);

  // X axis, 0
  line_direction_.x() = cMo(0,2);
  line_direction_.y() = cMo(1,2);
  line_direction_.z() = cMo(2,2);
  VispTools::rotateVector(line_direction_, x, y, z);

  //Get new Line Origin
  Eigen::Vector3f plane_normal;
  plane_normal.x() = plane_coeffs_.values[0];
  plane_normal.y() = plane_coeffs_.values[1];
  plane_normal.z() = plane_coeffs_.values[2];

  Eigen::Vector4f plane_centroid;
  pcl::compute3DCentroid<PointT>(*plane_cloud_, plane_centroid);
  Eigen::Vector3f plane_centroid_3f(plane_centroid.x(), plane_centroid.y(), plane_centroid.z());

  Eigen::Vector3f point_in_object_projected_into_plane;
  pcl::geometry::project(line_origin_, plane_centroid_3f, plane_normal, point_in_object_projected_into_plane);

  Eigen::Vector3f object_to_plane(line_origin_-point_in_object_projected_into_plane);

  line_origin_.x() = point_in_object_projected_into_plane[0] + distance_ratio * object_to_plane[0];
  line_origin_.y() = point_in_object_projected_into_plane[1] + distance_ratio * object_to_plane[1];
  line_origin_.z() = point_in_object_projected_into_plane[2] + distance_ratio * object_to_plane[2];

}

