#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  if (argc != 7) {
    std::cerr << "Usage: " << argv[0] << "<topic> <qdot1> <qdot2> <qdot3> <qdot4> <qdot5>" << std::endl;
    exit(0);
  }
  std::string topic(argv[1]);

  ros::init(argc, argv, "setJointVelocity");
  ros::NodeHandle nh;
  ros::Publisher arm_pub;
  arm_pub=nh.advertise<sensor_msgs::JointState>(topic,1);
  ros::Rate rate(30);

  double qdot[5];
  for (int i=0; i<5; i++) qdot[i]=atof(argv[i+2]);

  while (ros::ok()) {

    sensor_msgs::JointState js;
    js.name.push_back(std::string("Slew"));
    js.position.push_back(qdot[0]);
    js.name.push_back(std::string("Shoulder"));
    js.position.push_back(qdot[1]);
    js.name.push_back(std::string("Elbow"));
    js.position.push_back(qdot[2]);
    js.name.push_back(std::string("JawRotate"));
    js.position.push_back(qdot[3]);
    js.name.push_back(std::string("JawOpening"));
    js.position.push_back(qdot[4]);

    arm_pub.publish(js);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
