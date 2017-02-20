/*
 * VispTools: usage example in http://github.com/davidfornas/pcl_manipulation
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#ifndef MARKERTOOLS_H_
#define MARKERTOOLS_H_

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32MultiArray.h>

#include <interactive_markers/interactive_marker_server.h>

#include <visp/vpHomogeneousMatrix.h>
#include <pm_tools/visp_tools.h>

//Class that publishes eef position from marker feedback to UWSim
class EefFollower
{
private:

  ros::Publisher pos_pub, params_pub;
  ros::Subscriber params_sub;

  bool show_marker, marker_created;
  double hand_opening;

  // create an interactive marker server on the topic namespace uwsim_marker
  interactive_markers::InteractiveMarkerServer server;
  vpHomogeneousMatrix marker_creation_pose, marker_current_pose;
  vpHomogeneousMatrix marker_sliding_reference_pose;

public:

  //Guided mode values
  int irad, ialong, iangle;

  EefFollower(std::string topic, ros::NodeHandle &nh )
    : server("uwsim_marker"), show_marker(false), marker_created(false), irad(0), ialong(0), iangle(0)
  {
    pos_pub = nh.advertise<geometry_msgs::Pose>(topic, 1); //"/gripperPose"
    params_pub = nh.advertise<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1);
    params_sub = nh.subscribe("/specification_params_to_uwsim", 1, &EefFollower::paramsCallback, this);
  }

  void setMarkerStatus( bool value ){
    show_marker = value;
  }

  //Interactive marker feedback class (calls the publisher)
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void paramsCallback(const std_msgs::Float32MultiArray &msg);

  void loop(vpHomogeneousMatrix cMg);

  void addMarker(vpHomogeneousMatrix cMg);

  void removeMarker();

  void resetMarker();


};


#endif
