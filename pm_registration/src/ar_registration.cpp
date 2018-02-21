/**
 * App: Registration of clouds using ARMarker Poses
 *  Created on: 18/01/2018
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_registration/marker_registration.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "ar_marker_registration");
  ros::NodeHandle nh;

  MarkerRegistration mr(nh, argc, argv);
  mr.run();

  return (0);
}

