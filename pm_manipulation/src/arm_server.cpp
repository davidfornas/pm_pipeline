/* ROBOT Wrapper to ARM5E to execute grasping
 * Author dfornas
 * Date 28/02/2018
 **/

#include <ros/ros.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <mar_ros_bridge/mar_params.h>
#include <mar_perception/VirtualImage.h>
#include <pm_manipulation/joint_offset.h>

class ArmWrapper{

    ros::NodeHandle nh_;

    vpColVector initial_posture_;
    vpHomogeneousMatrix cartesian_waypoint_;
    vpColVector joint_waypoint_;
    std::string joint_state_, joint_state_command_;
    ARM5Arm* robot_;
    vpHomogeneousMatrix cMg_, bMc_;
    double max_current_, max_velocity_;
    double velocity_aperture_;
    double gripper_manipulation_;
    double gripper_closed_;

public:
    ArmWrapper(){

      nh_.getParam("joint_state", joint_state_);
      nh_.getParam("joint_state_command", joint_state_command_);
      cartesian_waypoint_=mar_params::paramToVispHomogeneousMatrix(&nh_, "cartesian_waypoint_");
      joint_waypoint_=mar_params::paramToVispColVector(&nh_, "joint_waypoint_");
      initial_posture_=mar_params::paramToVispColVector(&nh_, "initial_posture");
      nh_.getParam("max_current", max_current_);
      nh_.getParam("max_velocity", max_velocity_);
      nh_.getParam("velocity_aperture", velocity_aperture_);
      nh_.getParam("gripper_manipulation", gripper_manipulation_);
      nh_.getParam("gripper_closed", gripper_closed_);

      robot_=new ARM5Arm(nh_, joint_state_, joint_state_command_);
    }

    void getbMc(){
      //Get bMc from TF...
    };

    vpHomogeneousMatrix tfToVisp(tf::StampedTransform matrix_tf){
      vpHomogeneousMatrix matrix_visp;
      matrix_visp[0][0]=matrix_tf.getBasis()[0][0]; matrix_visp[0][1]=matrix_tf.getBasis()[0][1]; matrix_visp[0][2]=matrix_tf.getBasis()[0][2]; matrix_visp[0][3]=matrix_tf.getOrigin().x();
      matrix_visp[1][0]=matrix_tf.getBasis()[1][0]; matrix_visp[1][1]=matrix_tf.getBasis()[1][1]; matrix_visp[1][2]=matrix_tf.getBasis()[1][2]; matrix_visp[1][3]=matrix_tf.getOrigin().y();
      matrix_visp[2][0]=matrix_tf.getBasis()[2][0]; matrix_visp[2][1]=matrix_tf.getBasis()[2][1]; matrix_visp[2][2]=matrix_tf.getBasis()[2][2]; matrix_visp[2][3]=matrix_tf.getOrigin().z();
      return matrix_visp;
    }

    // Obtain desired cMg_ from TF
    void cMgFromTF(){
      tf::StampedTransform cMg_tf;
      tf::TransformListener listener;
      bool tf_detected = false;
      do{
        try{
          listener.lookupTransform("/stereo", "/cMg", ros::Time(0), cMg_tf);
          tf_detected=true;
        }
        catch(tf::TransformException &ex){
        }
        ros::spinOnce();
      }while(!tf_detected && ros::ok());
      cMg_=ArmWrapper::tfToVisp(cMg_tf);
      tf_detected=true;
    }

    //Set desired cMg
    void setcMg( vpHomogeneousMatrix cMg){
      cMg_ = cMg;
    }

    // Go to goal with respect to camera.
    void reachPositionWrtCamera(vpHomogeneousMatrix cMgoal){
      vpHomogeneousMatrix bMe, cMe;

      robot_->getPosition(bMe);
      cMe=bMc_.inverse()*bMe;

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
        cMe=bMc_.inverse()*bMe;
      }
    }

    // Go to waypoint on goal cMg_*waypoint.
    void reachPositionWithWaypoint(vpHomogeneousMatrix cMgoal, vpHomogeneousMatrix waypoint){
      cMgoal=cMgoal*waypoint;
      reachPositionWrtCamera(cMgoal);
    }

    //Move to a cartesian increment joint by joint.
    void moveCartesianDistance(double x, double y, double z){
      vpHomogeneousMatrix bMe;
      vpColVector js;
      ros::spinOnce();
      robot_->getPosition(bMe);
      bMe[0][3] += x;
      bMe[1][3] += y;
      bMe[2][3] += z;
      js = robot_->armIK(bMe);
      reachJointPositionByJoint(js);
    }

    //Move to a cartesian increment joint by joint.
    void moveCartesianDistance2(double x, double y, double z){
      vpHomogeneousMatrix bMe;
      vpColVector js;
      ros::spinOnce();
      robot_->getPosition(bMe);
      bMe[0][3] += x;
      bMe[1][3] += y;
      bMe[2][3] += z;
      reachCartesianPosition(bMe);
    }


    void reachCartesianPosition(vpHomogeneousMatrix bMg){

      vpHomogeneousMatrix bMe;
      robot_->getPosition(bMe);
      // 0.25, beacuse with 0.15 it stops moving
      while((bMg.getCol(3)-bMe.getCol(3)).euclideanNorm()>0.025 && ros::ok()){
        std::cout<<"Error: "<<(bMg.getCol(3)-bMe.getCol(3)).euclideanNorm()<<std::endl;
        vpColVector xdot(6);
        xdot=0;
        vpHomogeneousMatrix eMg=bMe.inverse()*bMg;
        xdot[0]=eMg[0][3]*0.6;
        xdot[1]=eMg[1][3]*0.6;
        xdot[2]=eMg[2][3]*0.6;
        robot_->setCartesianVelocity(xdot);
        ros::spinOnce();
        robot_->getPosition(bMe);
      }
    }

    //Reach position joint by joint to avoid overcurrent.
    void reachJointPositionByJoint(vpColVector desired_joints){
      vpColVector current_joints;
      robot_->getJointValues(current_joints);
      desired_joints[4]=current_joints[4];
      ros::Rate r(20);

      int joint = 0;
      while(joint<5) {
        while ( std::abs(desired_joints[joint] - current_joints[joint]) > 0.015 && ros::ok()) {
          //std::cout<<"Error: "<<(desired_joints-current_joints).euclideanNorm()<<"X"<<desired_joints<<"X"<<current_joints<<std::endl;
          vpColVector cmd_vel(5, 0.0);
          cmd_vel[joint] = desired_joints[joint] - current_joints[joint];
          while (cmd_vel[joint] > max_velocity_ || cmd_vel[joint] < -max_velocity_) {
            cmd_vel[joint] /= 1.2;
          }
          robot_->setJointVelocity(cmd_vel);
          ros::spinOnce();
          r.sleep();

          robot_->getJointValues(current_joints);
          desired_joints[4] = current_joints[4];
        }
        joint++;
      }
    }

    //Reach joint position all at once. @TODO Limit each joint to avoid overcurrent.
    void reachJointPosition(vpColVector desired_joints){
      vpColVector current_joints;
      robot_->getJointValues(current_joints);
      desired_joints[4]=current_joints[4];
      ros::Rate r(20);

      vpColVector cmd_vel(5, 0.0);
      cmd_vel[0]=0.02;
      robot_->setJointVelocity(cmd_vel);

      while((desired_joints-current_joints).euclideanNorm()>0.015 && ros::ok()){
        //std::cout<<"Error: "<<(desired_joints-current_joints).euclideanNorm()<<"X"<<desired_joints<<"X"<<current_joints<<std::endl;
        vpColVector cmd_vel = desired_joints-current_joints;
        while(cmd_vel.euclideanNorm()>max_velocity_){
          cmd_vel/=1.2;
          //std::cout<<"CmdVel: "<<cmd_vel<<"Norm:"<< cmd_vel.euclideanNorm()<<std::endl;
        }
        //std::cout<<"CmdVel: "<<cmd_vel<<std::endl;
        robot_->setJointVelocity(cmd_vel);
        ros::spinOnce();
        r.sleep();

        robot_->getJointValues(current_joints);
        desired_joints[4]=current_joints[4];
      }
    }

    void openGripper(double velocity, double aperture, double current){
      vpColVector vel(5), current_joints;
      vel=0;
      vel[4]=velocity;
      robot_->getJointValues(current_joints);
      ros::Rate r(20);
      if(velocity>0){
        while(current_joints[4]<aperture && ros::ok() && robot_->getCurrent()<current ){
          robot_->setJointVelocity(vel);
          ros::spinOnce();
          r.sleep();
          robot_->getJointValues(current_joints);
        }
      }
      else{
        while(current_joints[4]>aperture && ros::ok() && robot_->getCurrent()<current ){
          robot_->setJointVelocity(vel);
          ros::spinOnce();
          r.sleep();
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

      //joint_offset_->reset_bMc(initial_posture_);
      //ArmWrapper::reachJointPositionByJoint(joint_waypoint_);
      ROS_INFO("Joint position reached. Trying cartesian...");

      //Needed to udate the postion...
      ros::Duration(0.3).sleep();
      ros::spinOnce();

      //DF: Movimiento cartesiano utilizando incrementos y el movimientos joint by joint.
      ROS_INFO("Moving 10cm. joint by joint.");
      moveCartesianDistance(0.0, 0.0, 0.1);

      ROS_INFO("Moving 10cm. with setCartesianVelocity.");
      moveCartesianDistance2(0.0, 0.0, -0.1);

      //SHould test with obstacles
      ROS_INFO("Open the gripper until manipulation aperture");
      ArmWrapper::openGripper(velocity_aperture_, gripper_manipulation_, max_current_);
      ROS_INFO("Close the gripper");
      ArmWrapper::openGripper(-velocity_aperture_, gripper_closed_, max_current_);

      ROS_INFO("Finished testing...");
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_test");
  ArmWrapper robot;
  robot.testFunctions();
  return 1;
}

