/*
 *  VispTools
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */

#include <pm_tools/visp_tools.h>

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

geometry_msgs::Pose VispTools::geometryPoseFromVispHomog(vpHomogeneousMatrix sMs)
{
  vpQuaternionVector q;
  sMs.extract(q);
  geometry_msgs::Pose t;
  t.orientation.x = q.x();
  t.orientation.y = q.y();
  t.orientation.z = q.z();
  t.orientation.w = q.w();
  t.position.x = sMs[0][3];
  t.position.y = sMs[1][3];
  t.position.z = sMs[2][3];
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

vpHomogeneousMatrix VispTools::vispHomogFromGeometryPose(geometry_msgs::Pose pose)
{
  vpTranslationVector t(pose.position.x,pose.position.y, pose.position.z);
  vpQuaternionVector q(pose.orientation.x, pose.orientation.y ,pose.orientation.z, pose.orientation.w);
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


void VispTools::rpyFromQuaternion(double x, double y, double z, double w, double& r, double& p, double& yaw){
  tf::Quaternion rotation(x, y, z, w);
  tf::Matrix3x3 m(rotation);
  m.getRPY(r, p, yaw);
}

void VispTools::quaternionFromRpy(double r, double p, double yaw, double& x, double& y, double& z, double& w){
  tf::Quaternion q;
  q = tf::createQuaternionFromRPY(r, p, yaw);
  x = q.x();
  y = q.y();
  z = q.z();
  w = q.w();
}

Eigen::Matrix4f VispTools::vpHomogeneousMatrixToEigenMatrix4f(vpHomogeneousMatrix &in) {
  Eigen::Matrix4f out;
  out(0,0)=in[0][0];out(0,1)=in[0][1];out(0,2)=in[0][2];out(0,3)=in[0][3];
  out(1,0)=in[1][0];out(1,1)=in[1][1];out(1,2)=in[1][2];out(1,3)=in[1][3];
  out(2,0)=in[2][0];out(2,1)=in[2][1];out(2,2)=in[2][2];out(2,3)=in[2][3];
  out(3,0)=in[3][0];out(3,1)=in[3][1];out(3,2)=in[3][2];out(3,3)=in[3][3];
  return out;
}

vpHomogeneousMatrix VispTools::EigenMatrix4fToVpHomogeneousMatrix(Eigen::Matrix4f &in) {
  vpHomogeneousMatrix out;
  out[0][0]=in(0,0);out[0][1]=in(0,1);out[0][2]=in(0,2);out[0][3]=in(0,3);
  out[1][0]=in(1,0);out[1][1]=in(1,1);out[1][2]=in(1,2);out[1][3]=in(1,3);
  out[2][0]=in(2,0);out[2][1]=in(2,1);out[2][2]=in(2,2);out[2][3]=in(2,3);
  out[3][0]=in(3,0);out[3][1]=in(3,1);out[3][2]=in(3,2);out[3][3]=in(3,3);
  return out;
}

vpHomogeneousMatrix VispTools::EigenMatrixDouble44ToVpHomogeneousMatrix(Eigen::Matrix<double,4,4> &in) {
  vpHomogeneousMatrix out;
  out[0][0]=in(0,0);out[0][1]=in(0,1);out[0][2]=in(0,2);out[0][3]=in(0,3);
  out[1][0]=in(1,0);out[1][1]=in(1,1);out[1][2]=in(1,2);out[1][3]=in(1,3);
  out[2][0]=in(2,0);out[2][1]=in(2,1);out[2][2]=in(2,2);out[2][3]=in(2,3);
  out[3][0]=in(3,0);out[3][1]=in(3,1);out[3][2]=in(3,2);out[3][3]=in(3,3);
  return out;
}

