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
  static void rpyFromQuaternion(double, double, double, double, double&, double&, double&);
  static void quaternionFromRpy(double, double, double, double&, double&, double&, double&);

};



#endif
