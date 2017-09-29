/*
 * PMGraspPlanning.cpp
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#include <pm_manipulation/slider_grasp_planner.h>
#include <pm_tools/pcl_tools.h>

#include <visp/vpHomogeneousMatrix.h>

typedef pcl::PointXYZ PointT;

void SliderGraspPlanner::perceive() {

  bool initialized;

  if( do_ransac ){
    ROS_INFO_STREAM("Computing pose with RANSAC...");
    sac_pose_estimation->doRansac();
    ROS_INFO_STREAM("RANSAC finished.");
  }else{
    ROS_INFO_STREAM("Waiting for pose on topic: " << topic_name);
    if (!initialized){
      geometry_msgs::Pose::ConstPtr message = ros::topic::waitForMessage< geometry_msgs::Pose >(topic_name);
      cMo = VispTools::vispHomogFromGeometryPose(*message);
      initialized = true;
    }else{
      geometry_msgs::Pose::ConstPtr message = ros::topic::waitForMessage< geometry_msgs::Pose >(topic_name, ros::Duration(2));
      if (message == NULL){
           ROS_INFO("No pose messages received in 2 seconds. cMo unchanged.");
           return;
      }
      cMo = VispTools::vispHomogFromGeometryPose(*message);
    }

    ROS_INFO_STREAM("Pose received");
    change_z_ = false;
    //Director vectors: cylinder axis and perpendicular vector.
    if(change_z_){
      // @TODO fix this!!!
      ROS_INFO_STREAM("Changing Z...");
      tf::Vector3 axis_dir(cMo[0][1], cMo[1][1], cMo[2][1]);
      axis_dir=axis_dir.normalize();
      tf::Vector3 perp = tf::Vector3(0,0,1) - ( axis_dir.dot( tf::Vector3(0,0,1) ) * axis_dir );
      perp=perp.normalize();

      tf::Vector3 result=tf::tfCross( perp, axis_dir).normalize();
      result = -result;

      cMo[0][0]=result.x(); cMo[0][2]=perp.x();
      cMo[1][0]=result.y(); cMo[1][2]=perp.y();
      cMo[2][0]=result.z(); cMo[2][2]=perp.z();
    }
    // @DEBUG vispToTF.addTransform(cMo, "/world", "/amphora_from_pose", "cMo");
  }
  //Compute modified cMg from cMo
  recalculate_cMg();
}

void SliderGraspPlanner::redoRansac() {
  sac_pose_estimation->redoRansac();
  recalculate_cMg();
}

void SliderGraspPlanner::computeMatrix( double angle, double rad, double along ){
vpHomogeneousMatrix oMg;

//Apply rotations and traslation to reposition the grasp frame.
vpHomogeneousMatrix grMgt0(0,along,0,0,0,0);
vpHomogeneousMatrix gMgrZ(0,0,0,0,0,1.57);
vpHomogeneousMatrix gMgrX(0,0,0,1.57,0,0);
vpHomogeneousMatrix gMgrY(0,0,0,0,0,angle);
vpHomogeneousMatrix grMgt(rad,0,0,0,0,0);
oMg = grMgt0 * gMgrZ * gMgrX * gMgrY * grMgt;
cMg = cMo * oMg ;

cMg=cMg * vpHomogeneousMatrix(0,0,0,0,1.57,0) * vpHomogeneousMatrix(0,0,0,0,0,3.14);
}

/** Compute cMg from cMo. This function is inherited from MAR where
 * the repositioning is done using an UI
 */
void SliderGraspPlanner::recalculate_cMg(){

  if( do_ransac ) cMo = sac_pose_estimation->get_cMo();

  //Set grasp config values from interface int values.
  intToConfig();
  computeMatrix( angle_, rad_, along_ );

  if( iangle > 90 )
      cMg = cMg * vpHomogeneousMatrix(0,0,0,0,0,-3.14);

  // @DEBUG vispToTF.resetTransform( cMg, "cMg");
  // @DEBUG: VISUALIZE CYLINDER DETECTION FRAMES.
  vispToTF.publish();

  // Send detected cylinder marker.
  ros::NodeHandle nh;
  vpHomogeneousMatrix cylinder;
  cylinder = cMo * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0);
  MarkerPublisher markerPub( cylinder, camera_frame_name, "visualization_marker", nh);
  markerPub.setCylinder( cylinder, camera_frame_name, 0.15, 0.5, 15);
  markerPub.publish();

}

//Compute the best grasp from to approach directions along the cylinder axis.
void SliderGraspPlanner::getBestParams( double & angle, double & rad, double & along){

  computeMatrix( angle, rad, along);
  //tf::Vector3 grasp1(cMg[2][0],cMg[2][1],cMg[2][2]), zworld(0, 0, 1);
  tf::Vector3 grasp1(cMg[0][1],cMg[1][1],cMg[2][1]), zworld(0, 0, 1);
  double angle1 = grasp1.angle(zworld);

  computeMatrix( 180 - angle, rad, along);
  //tf::Vector3 grasp2(cMg[2][0],cMg[2][1],cMg[2][2]);
  tf::Vector3 grasp2(cMg[0][1],cMg[1][1],cMg[2][1]);
  double angle2 = grasp2.angle(zworld);

  if (angle2 > angle1)
    angle = 180 - angle;
}

/// Set config values: from int sliders to float values.
void SliderGraspPlanner::intToConfig(){
  angle_=iangle*(2.0*M_PI/360.0);
  rad_=-irad/100.0;
  along_=(ialong-20)/100.0;//to allow 20 cm negative.... should allow a range based on minMax distance
}


