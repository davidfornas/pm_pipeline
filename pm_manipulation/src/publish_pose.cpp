/*
  Publish a Pose to use with Merbots GUI
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "Publish Pose");
  ros::NodeHandle nh;

  if (argc!=9) {
    std::cerr << "USAGE: " << argv[0] << " <topic> <x> <y> <z> <xr> <yr> <zr> <wr>" << std::endl;
    std::cerr << "units are in meters and radians." << std::endl;
    return 0;
  }

  std::string topic(argv[1]);
  double x=atof(argv[2]);
  double y=atof(argv[3]);
  double z=atof(argv[4]);
  double xr=atof(argv[5]);
  double yr=atof(argv[6]);
  double zr=atof(argv[7]);
  double wr=atof(argv[8]);

  ros::Publisher position_pub;
  position_pub=nh.advertise<geometry_msgs::Pose>(topic,1);

  ros::Rate r(25);
  while (ros::ok()) {
    geometry_msgs::Pose pose;
    pose.position.x=x;
    pose.position.y=y;
    pose.position.z=z;
    pose.orientation.x=xr;
    pose.orientation.y=yr;
    pose.orientation.z=zr;
    pose.orientation.w=wr;
    position_pub.publish(pose);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
