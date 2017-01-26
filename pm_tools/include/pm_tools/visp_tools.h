/*
 * VispTools: usage example in http://github.com/davidfornas/pcl_manipulation
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#ifndef VISPTOOLS_H_
#define VISPTOOLS_H_

#include <tf/transform_broadcaster.h>
#include <visp/vpHomogeneousMatrix.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

struct Frame
{
  tf::Transform pose;
  std::string parent, child;
  friend std::ostream& operator<<(std::ostream& out, Frame& x);
};

std::ostream& operator<<(std::ostream&, Frame&);

class VispTools
{

public:
  static geometry_msgs::Pose geometryPoseFromVispHomog(vpHomogeneousMatrix);
  static geometry_msgs::Transform geometryTransFromVispHomog(vpHomogeneousMatrix);
  static tf::Transform tfTransFromVispHomog(vpHomogeneousMatrix);
  static vpHomogeneousMatrix vispHomogFromTfTransform(tf::Transform);
  static vpHomogeneousMatrix vispHomogFromGeometryPose(geometry_msgs::Pose);
  static vpHomogeneousMatrix vispHomogFromXyzrpy(double, double, double, double, double, double);
  static vpHomogeneousMatrix weightedAverage(vpHomogeneousMatrix, int, vpHomogeneousMatrix);

};

/** Tool to publish a vpHomogeneousMatrix on the TF tree */
class VispToTF
{

  tf::TransformBroadcaster *broadcaster_;
  std::map<std::string, Frame> frames_;

public:

  /** Create a new publisher with a frame to add to the TF tree */
  VispToTF(vpHomogeneousMatrix sMs, std::string parent, std::string child);
  VispToTF(tf::Transform sMs, std::string parent, std::string child);
  /** Create a new empty publisher */
  VispToTF();

  /** Add a frame to the publish list */
  void addTransform(vpHomogeneousMatrix sMs, std::string parent, std::string child, std::string id = "0");
  void addTransform(tf::Transform sMs, std::string parent, std::string child, std::string id = "0");

  /** Remove a frame from the publish list */
  void removeTransform(std::string id = "0");

  /** Modify homog matrix of a transform, it's better to keep parent and child unmodified */
  void resetTransform(vpHomogeneousMatrix sMs, std::string id = "0");
  void resetTransform(tf::Transform sMs, std::string id = "0");

  /** Publish current transforms on TF */
  void publish();
  /** Print current transforms on console */
  void print();
};

class MarkerPublisher
{

  ros::Publisher publisher_;

public:

  //In ths case is safe to make it public. The it's easier to change  marker's appearance.
  visualization_msgs::Marker marker;

  MarkerPublisher(vpHomogeneousMatrix sMs, std::string parent, std::string topic_name, ros::NodeHandle nh)
  {
    tf::Transform pose = VispTools::tfTransFromVispHomog(sMs);
    setMarker(pose, parent);
    publisher_ = nh.advertise<visualization_msgs::Marker>(topic_name, 1);
  }

  MarkerPublisher(tf::Transform pose, std::string parent, std::string topic_name, ros::NodeHandle nh)
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
    marker.pose.orientation.x = pose.getOrigin().x();
    marker.pose.orientation.y = pose.getOrigin().y();
    marker.pose.orientation.z = pose.getOrigin().z();
    marker.pose.orientation.w = pose.getOrigin().w();
    marker.lifetime = ros::Duration(duration);
    changeScale(0.1, 0.1, 0.2);
    changeColor(1, 0.2, 0.7, 0.7);
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
