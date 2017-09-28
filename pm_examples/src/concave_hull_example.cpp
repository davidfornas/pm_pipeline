/** 
 * This program test the border detection using segmentation and concave hull.
 *  Created on: 17/03/2014
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_perception/border_detection.h>
#include <pcl/visualization/pcl_visualizer.h>

//Time measures
#include <ctime>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "concave_hull_example");
  ros::NodeHandle nh;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>), cloud_hull(new pcl::PointCloud<PointT>);
  pcl::PCDReader reader;
  reader.read("piramide2.pcd", *cloud);

  //With polymorphism should be:
  //BorderDetection * border_detector = new ConcaveHullBorderDetection(cloud);
  //border_detector->process();
  //border_detector->getTrajectory(cloud_hull);

  ConcaveHullBorderDetection border_detector(cloud);
  border_detector.setSubsamplingDistance(0.02);
  border_detector.process();
  border_detector.getTrajectory(cloud_hull);
  border_detector.generatePath();

  // If world to stereo frame is not published this will get stuck.
  //border_detector.transformPathFrame("/world");

  border_detector.savePathToFile();
  //TrajectoryFollowing trajectory_following(border_detector.getPath(), nh, std::string("/uwsim/joint_state"), std::string("/uwsim/joint_state_command"));

  // ----  VISUALIZATION  ---
  // -----Open 3D viewer and add point cloud-----
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> point_cloud_color_handler(cloud, 0, 0, 0);
  viewer.addPointCloud(cloud);
  // @ WARNING TODO Only using 10 points because the first were wrong.
  for (int i = 10; i < cloud_hull->points.size() - 1; ++i)
  {
    std::ostringstream id;
    id << "name: " << i;
    viewer.addLine<PointT>(cloud_hull->points[i], cloud_hull->points[i + 1], 255, 255, 0, id.str());
  }

  // -----Main loop-----
  long long path_counter = 0;
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
    border_detector.publishPath(nh);
    border_detector.publishTF();
    pcl_sleep(0.01);

    if (path_counter % 100 == 0)
    { // execute at 1hz
      //Optional: trajectory_following.moveToNextWaypoint();
    }
    path_counter++;
  }
  return (0);
}
