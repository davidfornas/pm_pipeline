/** 
 * This program is used to filter and modify point clouds easily.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>

#include <pm_tools/pcl_segmentation.h>
#include <ros/ros.h>

typedef pcl::PointXYZRGB PointType;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>), point_cloud_ptr2(new pcl::PointCloud<PointType>) ,
      point_cloud_ptr3(new pcl::PointCloud<PointType>),  plane(new pcl::PointCloud<PointType>);
  PCLTools<PointType>::cloudFromPCD(point_cloud_ptr, std::string(argv[1]) + std::string(".pcd")); //Load from PCDReader or from topic

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>), cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);

  PCLTools<PointType>::applyZAxisPassthrough(point_cloud_ptr, point_cloud_ptr2, atoi(argv[2]), atoi(argv[3]));

  PCLTools<PointType>::estimateNormals(point_cloud_ptr2, cloud_normals);

  PlaneSegmentation<PointType> plane_seg(point_cloud_ptr2, cloud_normals);
  plane_seg.setDistanceThreshold(atof(argv[4]));
  plane_seg.setIterations(100);
  plane_seg.apply(point_cloud_ptr3, cloud_normals2, plane, coefficients_plane);

  PCLTools<PointType>::cloudToPCD(point_cloud_ptr3, std::string(argv[1]) + std::string("_processed.pcd"));
  PCLTools<PointType>::cloudToPCD(plane, std::string(argv[1]) + std::string("_plane.pcd"));

  return (0);
}



