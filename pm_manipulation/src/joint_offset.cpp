#include <pm_manipulation/joint_offset.h>

void JointOffset::readJointsCallback(const sensor_msgs::JointState::ConstPtr& m){
    sensor_msgs::JointState joints_fixed;
    for(uint i=0; i<m->name.size(); i++){
      joints_fixed.name.push_back(m->name[i]);
      joints_fixed.position.push_back(m->position[i]+offset_[i]);
      joints_fixed.effort.push_back(m->effort[i]);
    }
    joints_fixed.header.stamp=ros::Time::now();
    joint_state_pub.publish(joints_fixed);
}

void JointOffset::markerCallback(const geometry_msgs::PoseStamped::ConstPtr &m){

  if(!marker_found) marker_found = true;
  setcMeFromTf();

  if( bMc_init){
    vpHomogeneousMatrix bMe=bMc*cMe;
    vpColVector desired_joints, current_joints;
    ros::spinOnce();
    robot->getJointValues(current_joints);
    desired_joints=robot->armIK(bMe);
    if(desired_joints[0]>-1.57 && desired_joints[0]<2.1195 && desired_joints[1]>0 && desired_joints[1]<1.58665 && desired_joints[2]>0 && desired_joints[2]<2.15294){
      for(uint i=0; i<3; i++){
        offset_[i]=desired_joints[i]-current_joints[i];
      }
    }
    else{
      std::cout<<"inverse Kinematic not solve: "<<desired_joints<<std::endl;
    }
  }
}

JointOffset::JointOffset(ros::NodeHandle& nh, std::string topic_joint_state, std::string topic_command_joint, std::string topic_joint_state_fixed): nh_(nh){

  robot=new ARM5Arm(nh_, topic_joint_state, topic_command_joint);

  cMe_found=false;
  bMc_init=false;
  offset_.resize(5);
  offset_=0;

  joint_state_sub=nh.subscribe<sensor_msgs::JointState>(topic_joint_state, 1, &JointOffset::readJointsCallback, this);
  marker_sub=nh.subscribe<geometry_msgs::PoseStamped>("/marker_filter_node/marker_pose", 1, &JointOffset::markerCallback, this);
  joint_state_pub=nh.advertise<sensor_msgs::JointState>(topic_joint_state_fixed,1);
}

JointOffset::JointOffset(ros::NodeHandle& nh, std::string topic_joint_state, std::string topic_command_joint, std::string topic_joint_state_fixed, float elbow_offset): nh_(nh){

  robot=new ARM5Arm(nh_, topic_joint_state, topic_command_joint);

  cMe_found=false;
  bMc_init=false;
  offset_.resize(5);
  offset_=0;
  offset_[2]=elbow_offset; //Recomended offset: 0.3

  joint_state_sub=nh.subscribe<sensor_msgs::JointState>(topic_joint_state, 1, &JointOffset::readJointsCallback, this);
  marker_found = true;
  joint_state_pub=nh.advertise<sensor_msgs::JointState>(topic_joint_state_fixed,1);
}

//Get new base to camera. Use if considering bad camera position (not the case).
int JointOffset::setbMcWithMarker(vpColVector initial_posture){

  ros::spinOnce();
  setcMeFromTf();
  robot->getPosition(bMe);

  bMc=bMe*cMe.inverse();
  bMc_init=true;
  ROS_INFO_STREAM("New bMc from TF(using marker+kinematics): " << std::endl << bMc);

  return 0;
}

int JointOffset::setcMeFromTf(){

  cMe_found=false;
  ros::Time time;
  time=ros::Time::now();
  while(!cMe_found && (ros::Time::now()-time).toSec()<5){
    try{
      listener.lookupTransform("/stereo", "/eef_marker", ros::Time(0), cMe_tf);
      cMe_found=true;
    }
    catch(tf::TransformException & ex){
      ROS_DEBUG_STREAM("cMm not found.");
    }
    ros::spinOnce();
  }
  if(!cMe_found){
    ROS_ERROR_STREAM("cMm not found in 5 seconds.");
    return -1;
  }
  cMe = VispTools::vispHomogFromTfTransform(cMe_tf);
  ROS_DEBUG_STREAM("New cMe from TF(using marker): " << std::endl << cMe);
}


int JointOffset::setbMcFromTf(){

  bMc_init=false;
  ros::Time time;
  time=ros::Time::now();
  while(!bMc_init && (ros::Time::now()-time).toSec()<5){
    try{
      listener.lookupTransform("/arm5/kinematic_base", "/stereo", ros::Time(0), bMc_tf);
      bMc_init=true;
    }
    catch(tf::TransformException & ex){
      ROS_DEBUG_STREAM("bMc not found.");
    }
    ros::spinOnce();
  }
  if(!bMc_init){
    ROS_ERROR_STREAM("bMc not found in 5 seconds.");
    return -1;
  }
  bMc = VispTools::vispHomogFromTfTransform(bMc_tf);
  ROS_DEBUG_STREAM("New bMc from TF(static): " << std::endl << bMc);
}
