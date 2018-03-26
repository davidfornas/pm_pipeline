/*
 * SliderGraspPlanner
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */
#include <pm_manipulation/slider_grasp_planner.h>

typedef pcl::PointXYZRGB PointT;

void SliderGraspPlanner::perceive() {

  bool initialized;

  if( do_ransac ){
    pose_estimation->initialize();
  }else{
    ROS_INFO_STREAM("Waiting for pose on topic: " << topic_name);
    if (!initialized){
      geometry_msgs::Pose::ConstPtr message = ros::topic::waitForMessage< geometry_msgs::Pose >(topic_name);
      cMo = VispTools::vispHomogFromGeometryPose(*message);
      initialized = true;
      ROS_INFO_STREAM("Pose received");
    }else{
      geometry_msgs::Pose::ConstPtr message = ros::topic::waitForMessage< geometry_msgs::Pose >(topic_name, ros::Duration(2));
      if (message == NULL){
        ROS_INFO("No pose messages received in 2 seconds. cMo unchanged.");
        return;
      }
      cMo = VispTools::vispHomogFromGeometryPose(*message);
    }
    // @DEBUG vispToTF.addTransform(cMo, "/world", "/amphora_from_pose", "cMo");
  }
  //Compute modified cMg from cMo
  recalculate_cMg();
}

void SliderGraspPlanner::redoRansac() {
  pose_estimation->process();
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

  if( do_ransac ) cMo = pose_estimation->get_cMo();

  //Set grasp config values from interface int values.
  intToConfig();
  computeMatrix( angle_, rad_, along_ );

  if( iangle > 90 )
    cMg = cMg * vpHomogeneousMatrix(0,0,0,0,0,-3.14);

  // @DEBUG vispToTF.resetTransform( cMg, "cMg");
  // @DEBUG: VISUALIZE CYLINDER DETECTION FRAMES.
  vispToTF.publish();

  // Send detected cylinder marker. @TODO FIX THIS CODE IS NOT DOING ANYTHING
  /*ros::NodeHandle nh;
  vpHomogeneousMatrix cylinder;
  cylinder = cMo * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0);
  MarkerPublisher markerPub( cylinder, camera_frame_name, "visualization_marker", nh);
  markerPub.setCylinder( cylinder, camera_frame_name, 0.15, 0.5, 15);
  markerPub.publish();*/
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


