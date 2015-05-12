/** 
 * This program test clustering techniques.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_clustering.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_clustering_example");
  ros::NodeHandle nh;
  CloudClustering cluster;
  cluster.apply();

  ros::Rate r(10);
  while (1)
  {
    //Do nothing but wait.
    cluster.display();
    r.sleep();
  }

  return (0);
}

