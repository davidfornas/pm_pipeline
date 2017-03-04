/*
 *  VispTools
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */

#include <pm_tools/tf_tools.h>

bool TFTools::transformPose(const geometry_msgs::PoseStamped& pose_in, geometry_msgs::PoseStamped &pose_out, const std::string& target_frame_id)
{
  tf::TransformListener *listener_;
  try{
    listener_->transformPose(target_frame_id, ros::Time(0), pose_in, "/sense3d", pose_out);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  pose_out.header.frame_id = target_frame_id;
  return true;
}


std::ostream& operator<<(std::ostream& out, Frame& x)
{
  out << "Pose origin  [x: " << x.pose.getOrigin().x() << " y: " << x.pose.getOrigin().y() << " z: "
      << x.pose.getOrigin().z() << "] " << "Pose rotation [x:" << x.pose.getRotation().x() << " y:"
      << x.pose.getRotation().y() << " z:" << x.pose.getRotation().z() << " w:" << x.pose.getRotation().w() << "] "
      << std::endl << "Parent: " << x.parent << " Child: " << x.child << std::endl;
  return out;
}

FrameToTF::FrameToTF(vpHomogeneousMatrix sMs, std::string parent, std::string child)
{
  broadcaster_ = new tf::TransformBroadcaster();
  addTransform(sMs, parent, child, "0");
}

FrameToTF::FrameToTF(tf::Transform sMs, std::string parent, std::string child)
{
  broadcaster_ = new tf::TransformBroadcaster();
  addTransform(sMs, parent, child, "0");
}

FrameToTF::FrameToTF()
{
  broadcaster_ = new tf::TransformBroadcaster();
}

void FrameToTF::addTransform(vpHomogeneousMatrix sMs, std::string parent, std::string child, std::string id)
{
  tf::Transform pose = VispTools::tfTransFromVispHomog(sMs);
  addTransform(pose, parent, child, id);
}

void FrameToTF::addTransform(tf::Transform sMs, std::string parent, std::string child, std::string id)
{
  //TODO: It's possible to do further checks.
  if (frames_.count(id) > 0)
  {
    ROS_DEBUG_STREAM("ID [" << id << "] already in use. It won't be added.");
    return;
  }
  // Check for transform duplicates, traverse through the map so keep it small.
  bool duplicate = false;
  std::string old_id;
  for (std::map<std::string, Frame>::iterator ii = frames_.begin(); ii != frames_.end(); ++ii)
  {
    if ((*ii).second.parent == parent && (*ii).second.child == child)
    {
      duplicate = true;
      old_id = (*ii).first;
    }
  }
  //Valid tree structure it's not checked. TF does this check, though.
  if (duplicate)
  {
    ROS_DEBUG_STREAM("Frame from " << child << " to " << parent << " already specified. Reset it with resetTransform(sMs, "<< old_id << " ).");
    return;
  }

  Frame f;
  f.pose = sMs;
  f.parent = parent;
  f.child = child;

  frames_[id] = f;
}

void FrameToTF::resetTransform(vpHomogeneousMatrix sMs, std::string id)
{
  tf::Transform pose = VispTools::tfTransFromVispHomog(vpHomogeneousMatrix(sMs));
  resetTransform(pose, id);
}

void FrameToTF::resetTransform(tf::Transform sMs, std::string id)
{
  if (frames_.count(id) < 1)
  {
    ROS_DEBUG_STREAM("Can't reset this item. ID [" << id << "] not found.");
    return;
  }
  frames_[id].pose = sMs;
}

void FrameToTF::removeTransform(std::string id)
{
  if (frames_.count(id) < 1)
  {
    ROS_DEBUG_STREAM("Can't delete this item. ID [" << id << "] not found.");
    return;
  }
  frames_.erase(id);
}

void FrameToTF::publish()
{
  for (std::map<std::string, Frame>::iterator ii = frames_.begin(); ii != frames_.end(); ++ii)
  {
    tf::StampedTransform transform((*ii).second.pose, ros::Time::now(), (*ii).second.parent, (*ii).second.child);
    broadcaster_->sendTransform(transform);
  }
}

void FrameToTF::print()
{
  for (std::map<std::string, Frame>::iterator ii = frames_.begin(); ii != frames_.end(); ++ii)
  {
    ROS_INFO_STREAM("ID string: " << (*ii).first << std::endl << "Frame -> " << (*ii).second);
  }
}
