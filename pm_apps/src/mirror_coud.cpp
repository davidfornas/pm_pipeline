/** 
 * This program is used to mirror a point cloud.
 * @TODO Move to a separate class and make this a test app.
 *
 *  Created on: 22/01/2018
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>
#include <pm_perception/cluster_measure.h>

#include <pcl/filters/fast_bilateral.h>

#include <pcl/console/parse.h>
#include <pcl/common/geometry.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CPtr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mirroring");
  ros::NodeHandle nh;

  std::string source("");
    CPtr cloud(new Cloud), mirror(new Cloud), projection(new Cloud);

  if (pcl::console::find_argument (argc, argv, "-f") > 0){
	  pcl::console::parse_argument (argc, argv, "-f", source);
	  PCLTools<PointT>::cloudFromPCD(cloud, source + std::string(".pcd"));
  }else
    return 0;

  ClusterMeasure<PointT> cm(cloud);
  Eigen::Vector4f centroid = cm.getCentroid();
  for (int i = 0; i < cloud->size() ; ++i) {
    cloud->points[i].x -= centroid.x();
    cloud->points[i].y -= centroid.y();
    cloud->points[i].z -= centroid.z();
  }

  Eigen::Vector3f pt, plane_origin, plane_normal;
  plane_origin.x() = 0.0;
  plane_origin.y() = 0.0;
  plane_origin.z() = 0.05;

  plane_normal.x() = 0.0;
  plane_normal.y() = 0.0;
  plane_normal.z() = 1.0;

  for (int i = 0; i < cloud->size() ; ++i) {
    Eigen::Vector3f origin_point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    pcl::geometry::project(origin_point, plane_origin, plane_normal, pt);

    PointT projected(pt.x(), pt.y(), pt.z());
    pt.x() = -cloud->points[i].x + 2 * pt.x();
    pt.y() = -cloud->points[i].y + 2 * pt.y();
    pt.z() = -cloud->points[i].z + 2 * pt.z();

    PointT mirrored(pt.x(), pt.y(), pt.z());
    mirror->push_back(mirrored);
    projection->push_back(projected);
  }

  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (mirror, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> pjt_h (projection, 0, 0, 255);
  p->addPointCloud (cloud, tgt_h, "c1");
  p->addPointCloud (mirror, src_h, "c2");
  p->addPointCloud (projection, pjt_h, "c3");
  pcl::ModelCoefficients coeffs;
  coeffs.values.resize(4);
  coeffs.values[0] = 0.0;
  coeffs.values[1] = 0.0;
  coeffs.values[2] = 1.0;
  coeffs.values[3] = 0.0;
  //p->addPlane(coeffs);
  p->addCoordinateSystem(0.1, 0, 0, 0);
  p->spin();

  for (int i = 0; i < mirror->size() ; ++i) {
    cloud->push_back(mirror->points[i]);
  }

  //PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.03);
  PCLTools<PointT>::cloudToPCD(cloud, source + std::string("_mirrored.pcd"));

  return (0);
}



