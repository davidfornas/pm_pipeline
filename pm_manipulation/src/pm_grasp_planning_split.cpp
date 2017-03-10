/*
 * PMGraspPlanning.cpp
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#include <pm_manipulation/pm_grasp_planning_split.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>

#include <visp/vpHomogeneousMatrix.h>

#include <boost/bind.hpp>
#include <vector>
#include <algorithm>


void PMGraspPlanningSplit::perceive() {


  ROS_INFO_STREAM("Waiting for pose on topic: " << topic_name);
  geometry_msgs::Pose::ConstPtr message = ros::topic::waitForMessage< geometry_msgs::Pose >(topic_name);
  cMo = VispTools::vispHomogFromGeometryPose(*message);
  ROS_INFO_STREAM("Pose received");

  /*
  //Director vectors: cylinder axis and perpendicular vector.
  tf::Vector3 axis_dir(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);
  axis_dir=axis_dir.normalize();

  tf::Vector3 perp(coefficients_cylinder->values[4], -coefficients_cylinder->values[3], 0);
  perp=perp.normalize();

  tf::Vector3 result=tf::tfCross( perp, axis_dir).normalize();

  //getMinMax3DAlongAxis(cloud_cylinder, &max, &min, axis_point, &axis_dir, 0.1);
  //Mean point taking into account only a 90% of the points (0.1).
  mean.x=(max.x+min.x)/2;mean.y=(max.y+min.y)/2;mean.z=(max.z+min.z)/2;
  coefficients_cylinder->values[0]=mean.x;
  coefficients_cylinder->values[1]=mean.y;
  coefficients_cylinder->values[2]=mean.z;

  //Cylinder properties
  radious=coefficients_cylinder->values[6];
  height=sqrt((max.x-min.x)*(max.x-min.x)+(max.y-min.y)*(max.y-min.y)+(max.z-min.z)*(max.z-min.z));

  // @ NOTE Ahora mismo el end-efector cae dentro del cilindro en vez de en superfície.
  //Esto está relativamente bien pero no tenemos en cuenta la penetración. Sin embargo, la
  //tenemos en cuenta luego al separanos el radio así que no hay problema en realidad.
  cMo[0][0]=perp.x(); cMo[0][1]=axis_dir.x(); cMo[0][2]=result.x();cMo[0][3]=mean.x;
  cMo[1][0]=perp.y(); cMo[1][1]=axis_dir.y(); cMo[1][2]=result.y();cMo[1][3]=mean.y;
  cMo[2][0]=perp.z(); cMo[2][1]=axis_dir.z(); cMo[2][2]=result.z();cMo[2][3]=mean.z;
  cMo[3][0]=0;cMo[3][1]=0;cMo[3][2]=0;cMo[3][3]=1;
  ROS_DEBUG_STREAM("cMo is...: " << std::endl << cMo << "Is homog: " << cMo.isAnHomogeneousMatrix()?"yes":"no");

  vispToTF.addTransform(cMo, camera_frame_name, "object_frame", "cMo");
  //TONI DEBUG vispToTF.addTransform(cMg, camera_frame_name, "grasp_frame", "cMg");
  */

  //DEBUG Print MAX and MIN frames
  /*vpHomogeneousMatrix cMg2(cMo);
  cMg2[0][3]=max.x;
  cMg2[1][3]=max.y;
  cMg2[2][3]=max.z;
  vispToTF.addTransform(cMg2, "/sense3d", "/max", "3");
  cMg2[0][3]=min.x;
  cMg2[1][3]=min.y;
  cMg2[2][3]=min.z;
  vispToTF.addTransform(cMg2, "/sense3d", "/min", "4");*/

  //Compute modified cMg from cMo
  recalculate_cMg();
}

void PMGraspPlanningSplit::computeMatrix( double angle, double rad, double along ){
vpHomogeneousMatrix oMg;

//Apply rotations and traslation to reposition the grasp frame.
vpHomogeneousMatrix grMgt0(0,along,0,0,0,0);
vpHomogeneousMatrix gMgrZ(0,0,0,0,0,1.57);
vpHomogeneousMatrix gMgrX(0,0,0,1.57,0,0);
vpHomogeneousMatrix gMgrY(0,0,0,0,0,angle);
vpHomogeneousMatrix grMgt(rad,0,0,0,0,0);
oMg = grMgt0 * gMgrZ * gMgrX * gMgrY * grMgt;
cMg = cMo * oMg ;

cMg=cMg * vpHomogeneousMatrix(0,0,0,0,1.57,0) ;// TONI Y HACIA EL OTRO LADO 3.14-> * vpHomogeneousMatrix(0,0,0,0,0,1.57);
}

/** Compute cMg from cMo. This function is inherited from MAR where
 * the repositioning is done using an UI
 */
void PMGraspPlanningSplit::recalculate_cMg(){

  //Set grasp config values from interface int values.
  intToConfig();
  computeMatrix( angle_, rad_, along_ );

  if( iangle > 90 )
      cMg = cMg * vpHomogeneousMatrix(0,0,0,0,0,-3.14);

  //TONI DEBUG vispToTF.resetTransform( cMg, "cMg");

  //TONI DEBUG: VISUALIZE CYLINDER DETECTION FRAMES.
  vispToTF.publish();

  // Send detected cylinder marker.
  ros::NodeHandle nh;
  vpHomogeneousMatrix cylinder;
  cylinder = cMo * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0);
  MarkerPublisher markerPub( cylinder, camera_frame_name, "visualization_marker", nh);
  markerPub.setCylinder( cylinder, camera_frame_name, 0.15, 0.5, 15);
  markerPub.publish();

}


void PMGraspPlanningSplit::getBestParams( double & angle, double & rad, double & along){

  computeMatrix( angle, rad, along);
  tf::Vector3 a(cMg[2][0],cMg[2][1],cMg[2][2]), b(0, 0, 1);
  double angle1 = a.angle(b);
  ROS_INFO_STREAM("Angle1"<<angle1);
  computeMatrix( 180 - angle, rad, along);
  tf::Vector3 c(cMg[2][0],cMg[2][1],cMg[2][2]);
  double angle2 = c.angle(b);
  ROS_INFO_STREAM("Angle2"<<angle2);
  if (angle2 < angle1)
    angle = 180 - angle;

}


/// Set config values: from int sliders to float values.
void PMGraspPlanningSplit::intToConfig(){
  angle_=iangle*(2.0*M_PI/360.0);
  rad_=-irad/100.0;
  along_=(ialong-20)/100.0;//to allow 20 cm negative.... should allow a range based on minMax distance
}



