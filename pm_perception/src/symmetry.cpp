/*
 * symmetry
 * 			 
 *  Created on: 28/03/2018
 *      Author: dfornas
 */
#include <pm_perception/symmetry.h>

void MirrorCloud::apply( CloudPtr & mirrored ) {
  ClusterMeasure<PointT> cm(cloud_);
//  CloudPtr centered(cloud_);
//  cloud_centered_ = centered;
//  Eigen::Vector4f centroid = cm.getCentroid();
//  for (int i = 0; i < cloud_centered_->points.size(); ++i) {
//    cloud_centered_->points[i].x -= centroid.x();
//    cloud_centered_->points[i].y -= centroid.y();
//    cloud_centered_->points[i].z -= centroid.z();
//  }
//  ROS_INFO_STREAM("Size"<< cloud_centered_->points.size());
//  PCLView<PointT>::showCloud(cloud_centered_);

//  CloudPtr mirror(new Cloud), projection(new Cloud);
//  mirror->points.resize( cloud_centered_->points.size() );
//  projection->points.resize( cloud_centered_->points.size() );

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
  }
//  mirror_->width = cloud_->width;
//  projection_->width = cloud_->width;
//  mirrored_->width = cloud_->width * 2;
//  mirrored_->
//  PCLView<PointT>::showCloud(mirror_);
//  PCLView<PointT>::showCloud(projection_);
//  ROS_INFO_STREAM("Size"<< mirror_->points.size());
//  mirror_ = mirror;
//  projection_ = projection;

//  CloudPtr cloud_copy(cloud_);
//  mirrored_ = cloud_copy;
//  mirrored_->points.resize( cloud_->points.size() + mirror_->points.size() );

//  ROS_INFO_STREAM("Size"<< mirrored_->points.size());
//  mirrored->width *= 2;
//  for (int i = 0; i < mirror_->points.size(); ++i) {
//    PointT p(mirror_->points[i]);
//    p.x += centroid.x();
//    p.y += centroid.y();
//    p.z += centroid.z();
//    mirrored_->points[cloud_->points.size() + i] = p;
//  }

//  PCLView<PointT>::showCloud(mirrored_);
  mirrored = mirrored_;

}

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
  /*pcl::ModelCoefficients coeffs;
  coeffs.values.resize(4);
  coeffs.values[0] = 0.0;
  coeffs.values[1] = 0.0;
  coeffs.values[2] = 1.0;
  coeffs.values[3] = 0.0;
  coeffs.values[4] = 0.0;
  coeffs.values[5] = 0.7;
  p->addPlane(coeffs);*/
  p->addCoordinateSystem(0.1, 0, 0, 0);
  p->spin();

}