/*
 * symmetry
 * 			 
 *  Created on: 28/03/2018
 *      Author: dfornas
 */
#include <pm_perception/symmetry.h>

void MirrorCloud::display() {

  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer("Mirror colored");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h(cloud_, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h(mirror_, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> pjt_h(projection_, 0, 0, 255);
  p->addPointCloud(cloud_, tgt_h, "c1");
  p->addPointCloud(mirror_, src_h, "c2");
  p->addPointCloud(projection_, pjt_h, "c3");
  p->addCoordinateSystem(0.1, 0, 0, 0);
  p->spin();

}

void MirrorCloud::displayMirrored() {

  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer("Result");

  p->addPointCloud(mirrored_);
  p->addCoordinateSystem(0.1, 0, 0, 0);
  p->spin();

}

void PlaneMirrorCloud::apply( CloudPtr & mirrored ) {
  mirrored = cloud_;
  ClusterMeasure<PointT> cm(cloud_);
  double distance = 0.;
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

    Eigen::Vector3f diff ;
    diff.x() = mirrored.x -projected.x;
    diff.y() = mirrored.y -projected.y;
    diff.z() = mirrored.z -projected.z;
    distance += diff.squaredNorm ();
  }
  distance /= cloud_->points.size();
  ROS_INFO_STREAM("Squared distance mean to plane for mirrored points: "<<distance);
  if(distance > 0.001)
    mirrored = mirrored_;

}



void LineMirrorCloud::apply( CloudPtr & mirrored ) {
  mirrored = cloud_;
  ClusterMeasure<PointT> cm(cloud_);
  double distance = 0.;
  for (int i = 0; i < cloud_->points.size(); ++i) {

    Eigen::Vector3f pt;
    Eigen::Vector3f origin_point(cloud_->points[i].x, cloud_->points[i].y, cloud_->points[i].z);

//    pcl::geometry::project(origin_point, plane_origin_, plane_normal_, pt);

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

    Eigen::Vector3f diff ;
    diff.x() = mirrored.x -projected.x;
    diff.y() = mirrored.y -projected.y;
    diff.z() = mirrored.z -projected.z;
    distance += diff.squaredNorm ();
  }
  distance /= cloud_->points.size();
  ROS_INFO_STREAM("Squared distance mean to plane for mirrored points: "<<distance);
  if(distance > 0.001)
    mirrored = mirrored_;

}

