/*
 *  VispTools
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */

#include <pm_tools/visp_tools.h>
#include <tf/transform_broadcaster.h>
#include <visp/vpHomogeneousMatrix.h>
#include <stdlib.h>

std::ostream& operator<<(std::ostream& out, Frame& x)
{
  out << "Pose origin  [x: " << x.pose.getOrigin().x() << " y: " << x.pose.getOrigin().y() << " z: "
      << x.pose.getOrigin().z() << "] " << "Pose rotation [x:" << x.pose.getRotation().x() << " y:"
      << x.pose.getRotation().y() << " z:" << x.pose.getRotation().z() << " w:" << x.pose.getRotation().w() << "] "
      << std::endl << "Parent: " << x.parent << " Child: " << x.child << std::endl;
  return out;
}

geometry_msgs::Transform VispTools::geometryTransFromVispHomog(vpHomogeneousMatrix sMs)
{
  vpQuaternionVector q;
  sMs.extract(q);

  geometry_msgs::Transform t;
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();
  t.translation.x = sMs[0][3];
  t.translation.y = sMs[1][3];
  t.translation.z = sMs[2][3];

  return t;
}

tf::Transform VispTools::tfTransFromVispHomog(vpHomogeneousMatrix sMs)
{

  tf::Vector3 translation(sMs[0][3], sMs[1][3], sMs[2][3]);

  vpQuaternionVector q;
  sMs.extract(q);
  tf::Quaternion rotation(q.x(), q.y(), q.z(), q.w());

  tf::Transform pose(rotation, translation);
  return pose;
}

vpHomogeneousMatrix VispTools::vispHomogFromTfTransform(tf::Transform transform)
{
  vpTranslationVector t(transform.getOrigin()[0], transform.getOrigin()[1], transform.getOrigin()[2]);
  vpQuaternionVector q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(),
                       transform.getRotation().w());
  return vpHomogeneousMatrix(t, q);
}

vpHomogeneousMatrix VispTools::vispHomogFromXyzrpy(double x, double y, double z, double r, double p, double yaw)
{
  return vpHomogeneousMatrix(vpTranslationVector(x, y, z), vpRotationMatrix(vpRxyzVector(r, p, yaw)));

}

vpHomogeneousMatrix VispTools::weightedAverage(vpHomogeneousMatrix old_avg, int old_weight, vpHomogeneousMatrix accum)
{

  double mean_x = (old_avg[0][3] * old_weight + accum[0][3]) / (old_weight + 1);
  double mean_y = (old_avg[1][3] * old_weight + accum[1][3]) / (old_weight + 1);
  double mean_z = (old_avg[2][3] * old_weight + accum[2][3]) / (old_weight + 1);

  vpQuaternionVector rot;
  accum.extract(rot);
  tf::Quaternion rotation(rot.x(), rot.y(), rot.z(), rot.w());
  tf::Matrix3x3 m(rotation);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  old_avg.extract(rot);
  tf::Quaternion rotation2(rot.x(), rot.y(), rot.z(), rot.w());
  tf::Matrix3x3 m2(rotation2);
  double old_roll, old_pitch, old_yaw;
  m2.getRPY(roll, pitch, yaw);

  double mean_c_roll = (cos(old_roll) * old_weight + cos(roll)) / (old_weight + 1);
  double mean_s_roll = (sin(old_roll) * old_weight + sin(roll)) / (old_weight + 1);
  double mean_c_pitch = (cos(old_pitch) * old_weight + cos(pitch)) / (old_weight + 1);
  double mean_s_pitch = (sin(old_pitch) * old_weight + sin(pitch)) / (old_weight + 1);
  double mean_c_yaw = (cos(old_yaw) * old_weight + cos(yaw)) / (old_weight + 1);
  double mean_s_yaw = (sin(old_yaw) * old_weight + sin(yaw)) / (old_weight + 1);

  double mean_roll = atan2(mean_s_roll, mean_c_roll);
  double mean_pitch = atan2(mean_s_pitch, mean_c_pitch);
  double mean_yaw = atan2(mean_s_yaw, mean_c_yaw);

  tf::Quaternion btQ;
  btQ = tf::createQuaternionFromRPY(mean_roll, mean_pitch, mean_yaw);
  vpTranslationVector tt(mean_x, mean_y, mean_z);
  vpQuaternionVector qq(btQ.x(), btQ.y(), btQ.z(), btQ.w());
  vpHomogeneousMatrix new_avg(tt, qq);
  ROS_DEBUG_STREAM("Average bMc " << std::endl << new_avg);

  vpTranslationVector trans;
  new_avg.extract(trans);

  return new_avg;
}

VispToTF::VispToTF(vpHomogeneousMatrix sMs, std::string parent, std::string child)
{
  broadcaster_ = new tf::TransformBroadcaster();
  addTransform(sMs, parent, child, "0");
}

VispToTF::VispToTF(tf::Transform sMs, std::string parent, std::string child)
{
  broadcaster_ = new tf::TransformBroadcaster();
  addTransform(sMs, parent, child, "0");
}

VispToTF::VispToTF()
{
  broadcaster_ = new tf::TransformBroadcaster();
}

void VispToTF::addTransform(vpHomogeneousMatrix sMs, std::string parent, std::string child, std::string id)
{
  tf::Transform pose = VispTools::tfTransFromVispHomog(sMs);
  addTransform(pose, parent, child, id);
}

void VispToTF::addTransform(tf::Transform sMs, std::string parent, std::string child, std::string id)
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

void VispToTF::resetTransform(vpHomogeneousMatrix sMs, std::string id)
{
  tf::Transform pose = VispTools::tfTransFromVispHomog(vpHomogeneousMatrix(sMs));
  resetTransform(pose, id);
}

void VispToTF::resetTransform(tf::Transform sMs, std::string id)
{
  if (frames_.count(id) < 1)
  {
    ROS_DEBUG_STREAM("Can't reset this item. ID [" << id << "] not found.");
    return;
  }
  frames_[id].pose = sMs;
}

void VispToTF::removeTransform(std::string id)
{
  if (frames_.count(id) < 1)
  {
    ROS_DEBUG_STREAM("Can't delete this item. ID [" << id << "] not found.");
    return;
  }
  frames_.erase(id);
}

void VispToTF::publish()
{
  for (std::map<std::string, Frame>::iterator ii = frames_.begin(); ii != frames_.end(); ++ii)
  {
    tf::StampedTransform transform((*ii).second.pose, ros::Time::now(), (*ii).second.parent, (*ii).second.child);
    broadcaster_->sendTransform(transform);
  }
}

void VispToTF::print()
{
  for (std::map<std::string, Frame>::iterator ii = frames_.begin(); ii != frames_.end(); ++ii)
  {
    ROS_INFO_STREAM("ID string: " << (*ii).first << std::endl << "Frame -> " << (*ii).second);
  }
}

