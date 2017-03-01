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
#include <std_msgs/String.h>

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

  geometry_msgs::Pose grasp_pose;

  //Guided mode values
  int irad, ialong, iangle;

  EefFollower(std::string topic, ros::NodeHandle &nh )
    : server("uwsim_marker"), show_marker(false), marker_created(false), irad(30), ialong(20), iangle(45)
  {
    pos_pub = nh.advertise<geometry_msgs::Pose>(topic, 1); //"/gripperPose"
    params_pub = nh.advertise<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1000);
    params_sub = nh.subscribe("/specification_params_to_uwsim", 1, &EefFollower::paramsCallback, this);
    hand_opening = 10/100;
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

//Class that publishes eef position from marker feedback to UWSim
class WaypointServer
{
private:

  ros::Subscriber dredging_status_sub;
  ros::Publisher dredging_pose_pub;

  // create an interactive marker server on the topic namespace uwsim_marker
  interactive_markers::InteractiveMarkerServer server;


  int marker_count;
  std::vector<std::string> name_list;

public:


  std::vector<geometry_msgs::Pose> pose_list;


  WaypointServer(std::string status_topic, std::string pose_topic, ros::NodeHandle &nh )
    : server("uwsim_marker"), marker_count(0)
  {
    dredging_status_sub = nh.subscribe(status_topic, 1, &WaypointServer::statusCallback, this);
    dredging_pose_pub = nh.advertise<geometry_msgs::Pose>(pose_topic, 1);
  }


  //Interactive marker feedback
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void statusCallback(const std_msgs::String &msg);

  void addInteractiveMarker();

  void deleteFirstMarker();

  void clearMarkers();

  void sendWaypoint();


};

class MarkerPublisher
{

  ros::Publisher publisher_;

public:

  //In ths case is safe to make it public. The it's easier to change  marker's appearance.
  visualization_msgs::Marker marker;

  MarkerPublisher(vpHomogeneousMatrix sMs, std::string parent, std::string topic_name, ros::NodeHandle & nh)
  {
    tf::Transform pose = VispTools::tfTransFromVispHomog(sMs);
    setMarker(pose, parent);
    publisher_ = nh.advertise<visualization_msgs::Marker>(topic_name, 1);
  }

  MarkerPublisher(tf::Transform pose, std::string parent, std::string topic_name, ros::NodeHandle & nh)
  {
    setMarker(pose, parent);
    publisher_ = nh.advertise<visualization_msgs::Marker>(topic_name, 1);
  }

  void setMarker(tf::Transform pose, std::string parent, int duration = 1)
  {
    marker.header.frame_id = parent;
    marker.header.stamp = ros::Time::now();
    marker.ns = "pose_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW; //TODO: More marker types if needed.
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.getOrigin().x();
    marker.pose.position.y = pose.getOrigin().y();
    marker.pose.position.z = pose.getOrigin().z();
    marker.pose.orientation.x = pose.getRotation().x();
    marker.pose.orientation.y = pose.getRotation().y();
    marker.pose.orientation.z = pose.getRotation().z();
    marker.pose.orientation.w = pose.getRotation().w();
    marker.lifetime = ros::Duration(duration);
    changeScale(0.1, 0.1, 0.2);
    changeColor(1, 0.2, 0.7, 0.7);
  }

  void setCylinder(vpHomogeneousMatrix sMs, std::string parent, double radious, double height, int duration = 1)
  {
    tf::Transform pose = VispTools::tfTransFromVispHomog(sMs);
    setMarker(pose, parent, duration);
    changeScale(radious, radious, height);
    marker.type = visualization_msgs::Marker::CYLINDER;
  }

  void setCylinder(tf::Transform pose, std::string parent, double radious, double height, int duration = 1)
  {
    setMarker(pose, parent, duration);
    changeScale(radious, radious, height);
    marker.type = visualization_msgs::Marker::CYLINDER;
  }

  void publish()
  {
    marker.header.stamp = ros::Time::now();
    publisher_.publish(marker);
  }

  void changeColor(double r, double g, double b, double a)
  {
    marker.color.a = r;
    marker.color.r = g;
    marker.color.g = b;
    marker.color.b = a;
  }

  void changeScale(double x, double y, double z)
  {
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
  }

  void changePose(tf::Transform pose)
  {
    marker.pose.position.x = pose.getOrigin().x();
    marker.pose.position.y = pose.getOrigin().y();
    marker.pose.position.z = pose.getOrigin().z();
    marker.pose.orientation.x = pose.getOrigin().x();
    marker.pose.orientation.y = pose.getOrigin().y();
    marker.pose.orientation.z = pose.getOrigin().z();
    marker.pose.orientation.w = pose.getOrigin().w();
  }
};

#endif
