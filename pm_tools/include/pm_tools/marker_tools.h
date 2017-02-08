/*
 * VispTools: usage example in http://github.com/davidfornas/pcl_manipulation
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#ifndef MARKERTOOLS_H_
#define MARKERTOOLS_H_

#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visp/vpHomogeneousMatrix.h>
#include <pm_tools/visp_tools.h>

//Class that publishes eef position from marker feedback to UWSim
class EefFollower
{
private:
  ros::Publisher pos_pub;
  bool show_marker, marker_created;

  // create an interactive marker server on the topic namespace uwsim_marker
  interactive_markers::InteractiveMarkerServer server;
  vpHomogeneousMatrix worldToMarkerInitPose;

public:
  EefFollower(std::string topic, ros::NodeHandle &nh )
    : server("uwsim_marker"), show_marker(false), marker_created(false)
  {
    pos_pub = nh.advertise<geometry_msgs::Pose>(topic, 1); //"/gripperPose"
  }

  void setMarkerStatus( bool value ){
    show_marker = value;
  }

  //Interactive marker feedback class (calls the publisher)
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void loop(vpHomogeneousMatrix cMg);


};


#endif
