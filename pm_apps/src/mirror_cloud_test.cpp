/** 
 * This program is used to mirror a point cloud.
 * @TODO Move to a separate class and make this a test app.
 *
 *  Created on: 22/01/2018
 *      Author: dfornas
 */

#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>
#include <pm_perception/symmetry.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CPtr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mirror_cloud_test");
  ros::NodeHandle nh;

  std::string source("");
    CPtr cloud(new Cloud), mirrored(new Cloud);

  if (pcl::console::find_argument (argc, argv, "-f") > 0){
	  pcl::console::parse_argument (argc, argv, "-f", source);
	  PCLTools<PointT>::cloudFromPCD(cloud, source + std::string(".pcd"));
  }else{
    // Debug in CLion without params.
    PCLTools<PointT>::cloudFromPCD(cloud, "/home/dfornas/ros_ws/prueba_mirror.pcd");
  }

  Eigen::Vector3f plane_origin, plane_normal;

  plane_origin.x() = 0.0;
  plane_origin.y() = 0.0;
  plane_origin.z() = 0.7;

  plane_normal.x() = 0.0;
  plane_normal.y() = 0.0;
  plane_normal.z() = 1.0;

  MirrorCloud mc(cloud, plane_origin, plane_normal);
  mc.apply(mirrored);
  mc.display();
  mc.displayMirrored();
  PCLTools<PointT>::cloudToPCD(mirrored, source + std::string("_mirrored.pcd"));

  return (0);
}



