#include <ros/ros.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <pm_manipulation/joint_offset.h>
#include <mar_ros_bridge/mar_params.h>
#include <mar_perception/VirtualImage.h>


class ArmWrapper{

  ros::NodeHandle nh_;
//	std::string action_name_;

  vpColVector initial_posture_;
  vpHomogeneousMatrix cartesian_waypoint_;
  vpColVector joint_waypoint_;
  std::string joint_state_, joint_state_command_, joint_state_fixed_;
  JointOffset* joint_offset_;
  ARM5Arm* robot_;
  vpHomogeneousMatrix cMh_;
  double max_current_;
  double velocity_aperture_;
  double gripper_manipulation_;
  double gripper_closed_;


public:
  ArmWrapper(){

    nh_.getParam("joint_state", joint_state_);
    nh_.getParam("joint_state_command", joint_state_command_);
    nh_.getParam("joint_state_fixed", joint_state_fixed_);
    cartesian_waypoint_=mar_params::paramToVispHomogeneousMatrix(&nh_, "cartesian_waypoint_");
    joint_waypoint_=mar_params::paramToVispColVector(&nh_, "joint_waypoint_");
    initial_posture_=mar_params::paramToVispColVector(&nh_, "initial_posture");
    nh_.getParam("max_current", max_current_);
    nh_.getParam("velocity_aperture", velocity_aperture_);
    nh_.getParam("gripper_manipulation", gripper_manipulation_);
    nh_.getParam("gripper_closed", gripper_closed_);

    joint_offset_=new JointOffset(nh_, joint_state_, joint_state_command_, joint_state_fixed_);
    robot_=new ARM5Arm(nh_, joint_state_fixed_, joint_state_command_);
  }

  vpHomogeneousMatrix tfToVisp(tf::StampedTransform matrix_tf){
    vpHomogeneousMatrix matrix_visp;
    matrix_visp[0][0]=matrix_tf.getBasis()[0][0]; matrix_visp[0][1]=matrix_tf.getBasis()[0][1]; matrix_visp[0][2]=matrix_tf.getBasis()[0][2]; matrix_visp[0][3]=matrix_tf.getOrigin().x();
    matrix_visp[1][0]=matrix_tf.getBasis()[1][0]; matrix_visp[1][1]=matrix_tf.getBasis()[1][1]; matrix_visp[1][2]=matrix_tf.getBasis()[1][2]; matrix_visp[1][3]=matrix_tf.getOrigin().y();
    matrix_visp[2][0]=matrix_tf.getBasis()[2][0]; matrix_visp[2][1]=matrix_tf.getBasis()[2][1]; matrix_visp[2][2]=matrix_tf.getBasis()[2][2]; matrix_visp[2][3]=matrix_tf.getOrigin().z();
    return matrix_visp;
  }

  void detectConnector(bool valve_detected){
    tf::StampedTransform cMh_tf;
    tf::TransformListener listener;
    do{
      try{
        listener.lookupTransform("/stereo_down_optical", "/connector", ros::Time(0), cMh_tf);
        valve_detected=true;
      }
      catch(tf::TransformException &ex){
      }
      ros::spinOnce();
    }while(!valve_detected && ros::ok());
    cMh_=ArmWrapper::tfToVisp(cMh_tf);
    /*cMh_[0][0]=-0.07913152724;  cMh_[0][1]=0.9883773439;  cMh_[0][2]=-0.1298014922; cMh_[0][3]=0.02322718259;
    cMh_[1][0]=-0.160249269;  cMh_[1][1]=-0.1411287379;  cMh_[1][2]=-0.9769354386; cMh_[1][3]=-0.1983788169;
    cMh_[2][0]=-0.9838995747;  cMh_[2][1]=-0.05650579903;  cMh_[2][2]=0.1695544794; cMh_[2][3]=0.579123645;*/
    valve_detected=true;
  }

  void reachPosition(vpHomogeneousMatrix waypoint){
    vpHomogeneousMatrix bMc, bMe, cMe, cMgoal;
    joint_offset_->get_bMc(bMc);

    ArmWrapper::detectConnector(true);
    cMgoal=cMh_*waypoint;

    robot_->getPosition(bMe);
    cMe=bMc.inverse()*bMe;
    while((cMe.getCol(4)-cMgoal.getCol(4)).euclideanNorm()>0.015 && ros::ok()){
      std::cout<<"Error: "<<(cMe.getCol(4)-cMgoal.getCol(4)).euclideanNorm()<<std::endl;
      vpColVector xdot(6);
      xdot=0;
      vpHomogeneousMatrix eMgoal=cMe.inverse()*cMgoal;
      xdot[0]=eMgoal[0][3]*0.6;
      xdot[1]=eMgoal[1][3]*0.6;
      xdot[2]=eMgoal[2][3]*0.6;
      robot_->setCartesianVelocity(xdot);
      ros::spinOnce();

      robot_->getPosition(bMe);
      cMe=bMc.inverse()*bMe;
      ArmWrapper::detectConnector(true);
    }
  }

  void reachPositionStraight(vpHomogeneousMatrix waypoint){
    vpHomogeneousMatrix bMc, bMe, cMe, cMgoal;
    joint_offset_->get_bMc(bMc);

    ArmWrapper::detectConnector(true);
    cMgoal=cMh_*waypoint;

    vpColVector q(5);
    cMgoal[0][3]=cMe[0][3];
    while((cMe.getCol(4)-cMgoal.getCol(4)).euclideanNorm()>0.02 && ros::ok()){
      std::cout<<"Error: "<<(cMe.getCol(4)-cMgoal.getCol(4)).euclideanNorm()<<std::endl;
      vpColVector finalJoints(5), current_joints;
      vpHomogeneousMatrix bMv=bMc*cMgoal;
      robot_->getJointValues(current_joints);
      vpHomogeneousMatrix bMe;
      robot_->getPosition(bMe);
      bMv[1][3]=bMe[1][3];
      finalJoints=robot_->armIK(bMv);
      if(finalJoints[0]>-1.57 && finalJoints[0]<2.1195 && finalJoints[1]>0 && finalJoints[1]<1.58665 && finalJoints[2]>0 && finalJoints[2]<2.15294){
        q=finalJoints-current_joints;
        std::cout<<"q: "<<q<<std::endl;
        q[0]=0;
        q[3]=0;
        q[4]=0;
        robot_->setJointVelocity(q);
      }
      else{
        std::cout<<"Point no reachable final Joints: "<<finalJoints<<std::endl;
      }
      ros::spinOnce();
      robot_->getPosition(bMe);
      cMe=bMc.inverse()*bMe;
      ArmWrapper::detectConnector(true);
      cMgoal[0][3]=cMe[0][3];
    }
  }

  void reachJointPosition(vpColVector desired_joints){
    vpColVector current_joints;
    robot_->getJointValues(current_joints);
    desired_joints[4]=current_joints[4];
    while((desired_joints-current_joints).euclideanNorm()>0.015 && ros::ok()){
      std::cout<<"Error: "<<(desired_joints-current_joints).euclideanNorm()<<std::endl;
      robot_->setJointVelocity(desired_joints-current_joints);
      robot_->getJointValues(current_joints);
      ros::spinOnce();
    }
  }

  void openGripper(double velocity, double aperture, double current){
    vpColVector vel(5), current_joints;
    vel=0;
    vel[4]=velocity;
    robot_->getJointValues(current_joints);
    if(velocity>0){
      while(current_joints[4]<aperture && ros::ok() && robot_->getCurrent()<current ){
        robot_->setJointVelocity(vel);
        ros::spinOnce();
        robot_->getJointValues(current_joints);
      }
    }
    else{
      while(current_joints[4]>aperture && ros::ok() && robot_->getCurrent()<current ){
        robot_->setJointVelocity(vel);
        ros::spinOnce();
        robot_->getJointValues(current_joints);
      }
    }
  }

  void turnWrist(double velocity, double position_max, double  position_return, double current ){
    vpColVector vel(5), current_joints;
    vel=0;
    vel[3]=velocity;
    robot_->getJointValues(current_joints);

    if(velocity>0){
      while(current_joints[3]<position_max && robot_->getCurrent()<current && ros::ok()){
        robot_->setJointVelocity(vel);
        ros::spinOnce();
        robot_->getJointValues(current_joints);
        std::cout<<"Current: "<<robot_->getCurrent()<<std::endl;
      }
      std::cout<<"Current: "<<robot_->getCurrent()<<std::endl;
      vel[3]=-velocity;

      while(current_joints[3]>position_return && ros::ok()){
        robot_->setJointVelocity(vel);
        ros::spinOnce();
        robot_->getJointValues(current_joints);
        std::cout<<"current: "<<current_joints[3]<<" position_return: "<<position_return<<std::endl;
      }
    }
    else{
      std::cout<<"Turn left"<<std::endl;
      while(current_joints[3]>position_max && robot_->getCurrent()<current && ros::ok()){
        robot_->setJointVelocity(vel);
        ros::spinOnce();
        robot_->getJointValues(current_joints);
        std::cout<<"Current: "<<robot_->getCurrent()<<std::endl;
      }
      std::cout<<"Current: "<<robot_->getCurrent()<<std::endl;
      vel[3]=-velocity;
      while(current_joints[3]<position_return && ros::ok()){
        robot_->setJointVelocity(vel);
        ros::spinOnce();
        robot_->getJointValues(current_joints);
        std::cout<<"current: "<<current_joints[3]<<" position_return: "<<position_return<<std::endl;
      }
    }
  }

  void testFunctions(){

    joint_offset_->reset_bMc(initial_posture_);
    ArmWrapper::reachJointPosition(joint_waypoint_);
    ArmWrapper::reachPosition(cartesian_waypoint_);
    ROS_INFO("Open the gripper until manipulation aperture");
    ArmWrapper::openGripper(velocity_aperture_, gripper_manipulation_, max_current_);
    ROS_INFO("Close the gripper");
    ArmWrapper::openGripper(-velocity_aperture_, gripper_closed_, max_current_);
  }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_test");
  ArmWrapper robot;
  robot.testFunctions();
  return 0;
}

