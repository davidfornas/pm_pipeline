/*
   This program takes has two uses (@TODO: split)
    1) Take a maker pose and publish iy with a displacement.
    2) Take poses from RANSAC and save the in a CSV file.
 */
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <pm_tools/visp_tools.h>

class Republisher{

public:

  ros::Subscriber marker_sub;
  ros::Subscriber ransac_sub;
  std::ofstream myfile;
  geometry_msgs::PoseStamped p;

  Republisher(ros::NodeHandle & nh){

    marker_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pose", 100,&Republisher::callback, this);
    ransac_sub = nh.subscribe<geometry_msgs::Pose>("/object_pose", 100,&Republisher::ransacCallback, this);

    myfile.open ("pose_compare.csv");
    myfile << "Stamp,MarkerX,MarkerY,MarkerZ,RANSACx,RANSACy,RANSACz\n";

  }

  void callback(geometry_msgs::PoseStamped msg){

    vpHomogeneousMatrix d(0, 0, -0.03, 0, 0, 0);
    vpHomogeneousMatrix obj;
    obj = VispTools::vispHomogFromGeometryPose(msg.pose);

    obj = obj * d;

    p=msg;
    p.pose = VispTools::geometryPoseFromVispHomog(obj);
  }

  void ransacCallback(geometry_msgs::Pose msg){

    ROS_INFO_STREAM("PoseX"<<msg.position.x<<"MarkerPoseX"<<p.pose.position.x);
    ROS_INFO_STREAM("PoseY"<<msg.position.y<<"MarkerPoseY"<<p.pose.position.y);
    ROS_INFO_STREAM("PoseZ"<<msg.position.z<<"MarkerPoseZ"<<p.pose.position.z);

    myfile << p.header.stamp << ",";
    myfile << p.pose.position.x << ",";
    myfile << p.pose.position.y << ",";
    myfile << p.pose.position.z << ",";
    myfile << msg.position.x << ",";
    myfile << msg.position.y << ",";
    myfile << msg.position.z << "\n";
  }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  Republisher rp(nh);

  ros::Publisher position_pub=nh.advertise<geometry_msgs::PoseStamped>("/object_pose_from_marker",1);
  ros::Rate r(25);

  while (ros::ok()) {
    position_pub.publish(rp.p);
    ros::spinOnce();
    r.sleep();
  }

  rp.myfile.close();
  return 0;
}
