/*
 *  VispTools
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */

#include <pm_tools/marker_tools.h>

void EefFollower::paramsCallback(const std_msgs::Float32MultiArray &msg){
  if(msg.data.size()==4){
    irad = msg.data[0];
    iangle = msg.data[1];
    ialong = msg.data[2];
    hand_opening = msg.data[3];//Hand opening
  }else{
    vpHomogeneousMatrix relative_movement;
    relative_movement = VispTools::vispHomogFromXyzrpy(msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
    removeMarker();
    vpHomogeneousMatrix new_marker_pose;
    new_marker_pose = marker_sliding_reference_pose * relative_movement;
    addMarker(new_marker_pose);
    hand_opening = msg.data[6];//Hand opening
  }
}

//Interactive marker feedback class (calls the publisher)
void EefFollower::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  vpHomogeneousMatrix marker_displacement;
  marker_displacement = VispTools::vispHomogFromGeometryPose(feedback->pose);
  marker_current_pose = marker_creation_pose * marker_displacement;
  marker_sliding_reference_pose = marker_current_pose;
  geometry_msgs::Pose worldPose = VispTools::geometryPoseFromVispHomog( marker_current_pose );

  pos_pub.publish( worldPose );

  ros::spinOnce();
}

void EefFollower::loop(vpHomogeneousMatrix cMg){

  geometry_msgs::Pose pose = VispTools::geometryPoseFromVispHomog(cMg);
  if (!show_marker){
    if(marker_created){
      marker_created = false;
      removeMarker();
    }
    //Publish Pose on UWSim
    pos_pub.publish(pose);
  }else{
    if(!marker_created){
      marker_sliding_reference_pose = cMg;
      addMarker(cMg);
    }
  }
}

void EefFollower::addMarker(vpHomogeneousMatrix cMg){
  geometry_msgs::Pose pose = VispTools::geometryPoseFromVispHomog(cMg);
marker_created = true;
// DF NEED A MARKER CREATED POSE AND INIT TO BE DIFFERENT.
marker_current_pose = cMg;
marker_creation_pose = cMg;

// create an interactive marker for our server
visualization_msgs::InteractiveMarker int_marker;
int_marker.header.frame_id = "/world";

int_marker.name = "eefMarker";
//text orientation depends on world offset, so it's commented
//int_marker.description = "End Effector marker";
int_marker.scale = 0.2;

pos_pub.publish(pose);
int_marker.pose.position.x = pose.position.x;
int_marker.pose.position.y = pose.position.y;
int_marker.pose.position.z = pose.position.z;
int_marker.pose.orientation.x = pose.orientation.x;
int_marker.pose.orientation.y = pose.orientation.y;
int_marker.pose.orientation.z = pose.orientation.z;
int_marker.pose.orientation.w = pose.orientation.w;

// create a grey box marker
visualization_msgs::Marker box_marker;
box_marker.type = visualization_msgs::Marker::SPHERE;
box_marker.scale.x = 0.01;
box_marker.scale.y = 0.01;
box_marker.scale.z = 0.01;
box_marker.color.r = 0.5;
box_marker.color.g = 0.5;
box_marker.color.b = 0.5;
box_marker.color.a = 1.0;
// create a non-interactive control to hold the box
visualization_msgs::InteractiveMarkerControl box_control;
box_control.always_visible = false;
box_control.markers.push_back( box_marker );
// add the 6DOF control to the interactive marker
int_marker.controls.push_back( box_control );

visualization_msgs::InteractiveMarkerControl control;
control.orientation.w = 1;
control.orientation.x = 1;
control.orientation.y = 0;
control.orientation.z = 0;
control.name = "rotate_x";
control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
int_marker.controls.push_back(control);
control.name = "move_x";
control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
int_marker.controls.push_back(control);
control.orientation.w = 1;
control.orientation.x = 0;
control.orientation.y = 1;
control.orientation.z = 0;
control.name = "rotate_z";
control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
int_marker.controls.push_back(control);
control.name = "move_z";
control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
int_marker.controls.push_back(control);
control.orientation.w = 1;
control.orientation.x = 0;
control.orientation.y = 0;
control.orientation.z = 1;
control.name = "rotate_y";
control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
int_marker.controls.push_back(control);
control.name = "move_y";
control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
int_marker.controls.push_back(control);
// add the interactive marker to our collection &
// tell the server to call processFeedback() when feedback arrives for it
server.insert(int_marker,boost::bind(&EefFollower::processFeedback, this, _1) );
// 'commit' changes and send to all clients
server.applyChanges();

}

void EefFollower::removeMarker(){
  server.erase("eefMarker");
  server.applyChanges();
}


void EefFollower::resetMarker(){
  //RESET?
  marker_sliding_reference_pose = marker_current_pose;
}
