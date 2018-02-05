/** 
 * This program is used to test Octomap
 *  Created on: 05/02/2018
 *      Author: dfornas
 */


#include <pm_perception/octomap.h>

#include <iostream>
#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>
#include <pcl/filters/fast_bilateral.h>

#include <pcl/console/parse.h>
#include <pm_perception/octomap.h>
#include "../../pm_registration/include/pm_registration/marker_registration.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CPtr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  std::string source("");
  CPtr cloud(new Cloud);

  if (pcl::console::find_argument (argc, argv, "-f") > 0){
    pcl::console::parse_argument (argc, argv, "-f", source);
    PCLTools<PointT>::cloudFromPCD(cloud, source + std::string(".pcd"));
    std::cerr << "Cloud loaded from file. " << std::endl;
  }else if (pcl::console::find_argument (argc, argv, "-t") > 0){
    pcl::console::parse_argument (argc, argv, "-t", source);
    PCLTools<PointT>::cloudFromTopic(cloud, source);
    source = "topic";
    std::cerr << "Cloud loaded from topic. " << std::endl;
  }else{
    return 0;
  }

  OctomapManager<PointT> om(cloud);
  om.do_stuff();
  ROS_INFO("Display");
  om.display();
  return (0);
}


