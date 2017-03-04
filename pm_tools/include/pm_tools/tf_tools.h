/*
 * VispTools: usage example in http://github.com/davidfornas/pcl_manipulation
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#ifndef TFTOOLS_H_
#define TFTOOLS_H_

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visp/vpHomogeneousMatrix.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <pm_tools/visp_tools.h>

class TFTools
{

public:
  static bool transformPose(const geometry_msgs::PoseStamped& pose_in,
                     geometry_msgs::PoseStamped &pose_out, const std::string& target_frame_id);

};

struct Frame
{
  tf::Transform pose;
  std::string parent, child;
  friend std::ostream& operator<<(std::ostream& out, Frame& x);
};

std::ostream& operator<<(std::ostream&, Frame&);

/** Tool to publish a vpHomogeneousMatrix on the TF tree */
class FrameToTF
{

  tf::TransformBroadcaster *broadcaster_;
  std::map<std::string, Frame> frames_;

public:

  /** Create a new publisher with a frame to add to the TF tree */
  FrameToTF(vpHomogeneousMatrix sMs, std::string parent, std::string child);
  FrameToTF(tf::Transform sMs, std::string parent, std::string child);
  /** Create a new empty publisher */
  FrameToTF();

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



#endif
