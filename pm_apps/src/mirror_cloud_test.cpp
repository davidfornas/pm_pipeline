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
    //PCLTools<PointT>::cloudFromPCD(cloud, "/home/dfornas/ros_ws/prueba_mirror.pcd");
    PCLTools<PointT>::cloudFromPCD(cloud, "/home/dfornas/Code/sq/FedeRegions/cluster3.pcd");

  }

  ROS_INFO("Testing Plane Symmetry Estimation");
  Eigen::Vector3f plane_origin, plane_normal;
  plane_origin.x() = 0.0;
  plane_origin.y() = 0.0;
  plane_origin.z() = 0.95;

  plane_normal.x() = 0.0;
  plane_normal.y() = 0.0;
  plane_normal.z() = 1.0;

  CloudPtr full_model(new Cloud);
  PlaneSymmetryEstimation pse(cloud);
  pse.setPlane(plane_origin, plane_normal);
  pse.apply(full_model);
  pse.display();
  pse.displayMirrored();
  PCLTools<PointT>::cloudToPCD(full_model, source + std::string("_mirrored.pcd"));

  ROS_INFO("Testing Axis Symmetry Estimation");
  Eigen::Vector3f line_origin, line_dir;
  line_origin.x() = 0.0;
  line_origin.y() = 0.0;
  line_origin.z() = 1.0;
  //Must be unit vector
  line_dir.x() = 1.0;
  line_dir.y() = 0.0;
  line_dir.z() = 0.0;

  CloudPtr full_model2(new Cloud);
  AxisSymmetryEstimation ase(cloud);
  ase.setAxis(line_origin, line_dir);
  ase.apply(full_model2);
  ase.display();
  ase.displayMirrored();
  PCLTools<PointT>::cloudToPCD(full_model2, source + std::string("line_mirrored.pcd"));

  return (0);
}



