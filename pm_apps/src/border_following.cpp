/** 
 * This program test RangeImageBorderDetection http://pointclouds.org/documentation/tutorials/range_image_border_extraction.php
 *  Created on: 17/03/2014
 *      Author: dfornas
 */
#include <pm_perception/border_detection.h>

typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::PointWithRange>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PCDReader reader;

  reader.read ("uwsimFeb15.pcd", *cloud);
  RangeImageBorderDetection border_detector(cloud);
  border_detector.process();
  border_detector.getTrajectoryWithRange(cloud_hull);

    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    //viewer.addCoordinateSystem (1.0f);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> point_cloud_color_handler (cloud_hull, 255, 0, 0);
    viewer.addPointCloud<pcl::PointWithRange> (cloud_hull, point_cloud_color_handler, "points");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "points");

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }
  return (0);
}
