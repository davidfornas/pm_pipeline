/* ROBOT Wrapper to ARM5E to execute grasping
 * Author dfornas
 * Date 28/02/2018
 **/
#include <vector>
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

      //@TODO Make this optional. WrtCamera functions only if there is camera information.
      getbMc();
    }

    vpHomogeneousMatrix tfToVisp(tf::StampedTransform matrix_tf){
      vpHomogeneousMatrix matrix_visp;
      matrix_visp[0][0]=matrix_tf.getBasis()[0][0]; matrix_visp[0][1]=matrix_tf.getBasis()[0][1]; matrix_visp[0][2]=matrix_tf.getBasis()[0][2]; matrix_visp[0][3]=matrix_tf.getOrigin().x();
      matrix_visp[1][0]=matrix_tf.getBasis()[1][0]; matrix_visp[1][1]=matrix_tf.getBasis()[1][1]; matrix_visp[1][2]=matrix_tf.getBasis()[1][2]; matrix_visp[1][3]=matrix_tf.getOrigin().y();
      matrix_visp[2][0]=matrix_tf.getBasis()[2][0]; matrix_visp[2][1]=matrix_tf.getBasis()[2][1]; matrix_visp[2][2]=matrix_tf.getBasis()[2][2]; matrix_visp[2][3]=matrix_tf.getOrigin().z();
      return matrix_visp;
    }

    vpColVector initColVector( double a, double b, double c){
      vpColVector j;
      j.resize(3);
      j[0] = a;
      j[1] = b;
      j[2] = c;
      return j;
    }

    vpColVector initColVector( double a, double b, double c, double d){
      vpColVector j;
      j.resize(4);
      j[0] = a;
      j[1] = b;
      j[2] = c;
      j[3] = d;
      return j;
    }

    vpColVector initColVector( double a, double b, double c, double d, double e){
      vpColVector j;
      j.resize(5);
      j[0] = a;
      j[1] = b;
      j[2] = c;
      j[3] = d;
      j[4] = e;
      return j;
    }

    void getbMc(){
      //Get bMc from TF...
      tf::StampedTransform bMc_tf;
      tf::TransformListener listener;
      bool tf_detected = false;
      do{
        try{
          listener.lookupTransform("arm5/kinematic_base", "stereo", ros::Time(0), bMc_tf);
          tf_detected=true;
        }
        catch(tf::TransformException &ex){
        }
        ros::spinOnce();
      }while(!tf_detected && ros::ok());
      bMc_=ArmWrapper::tfToVisp(bMc_tf);
      tf_detected=true;
      ROS_INFO_STREAM("Got new bMc: " << bMc_);
    };

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
      ROS_INFO_STREAM("Got new cMg: " << cMg_);
    }

    vpHomogeneousMatrix getcMg(){
      return cMg_;
    }

    //Set desired cMg
    void setcMg( vpHomogeneousMatrix cMg){
      cMg_ = cMg;
    }

    //Move to a cartesian increment joint by joint.
    void moveCartesianDistanceJointByJoint(double x, double y, double z){
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
    void moveCartesianDistance(double x, double y, double z){
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
      while((bMg.getCol(3)-bMe.getCol(3)).euclideanNorm()>0.025 && ros::ok()){
        ROS_INFO_STREAM("Position error norm: "<<(bMg.getCol(3)-bMe.getCol(3)).euclideanNorm());
        vpColVector xdot(6);
        xdot=0;
        vpHomogeneousMatrix eMg=bMe.inverse()*bMg;
        xdot[0]=eMg[0][3]*0.6;
        xdot[1]=eMg[1][3]*0.6;
        xdot[2]=eMg[2][3]*0.6;

        while(xdot.euclideanNorm() > 0.4) xdot /= 1.5;
        while(xdot.euclideanNorm() < 0.25) xdot *= 1.5;

        ROS_DEBUG_STREAM("XDOT:" << xdot);
        robot_->setCartesianVelocity(xdot);
        ros::spinOnce();
        robot_->getPosition(bMe);
      }
    }

    // Go to goal with respect to camera.
    void reachPositionWrtCamera(vpHomogeneousMatrix cMgoal){
      vpHomogeneousMatrix bMe, cMe;

      robot_->getPosition(bMe);
      cMe=bMc_.inverse()*bMe;

      while((cMe.getCol(3)-cMgoal.getCol(3)).euclideanNorm()>0.025 && ros::ok()){
        ROS_INFO_STREAM("Position error norm: "<<(cMe.getCol(3)-cMgoal.getCol(3)).euclideanNorm());
        vpColVector xdot(6);
        xdot=0;
        vpHomogeneousMatrix eMgoal=cMe.inverse()*cMgoal;
        xdot[0]=eMgoal[0][3]*0.8;
        xdot[1]=eMgoal[1][3]*0.8;
        xdot[2]=eMgoal[2][3]*0.8;

        while(xdot.euclideanNorm() > 0.4) xdot /= 1.5;
        while(xdot.euclideanNorm() < 0.25) xdot *= 1.5;

        ROS_DEBUG_STREAM("XDOT:" << xdot);
        robot_->setCartesianVelocity(xdot);
        ros::spinOnce();

        robot_->getPosition(bMe);
        cMe=bMc_.inverse()*bMe;
      }
    }

    // Go to goal with respect to camera.
    void reachPositionWrtCameraJointByJoint(vpHomogeneousMatrix cMgoal){
      vpHomogeneousMatrix bMgoal;
      vpColVector js;

      bMgoal = bMc_ * cMgoal;
      js = robot_->armIK(bMgoal);
      reachJointPositionByJoint(js);
    }

    // Go to waypoint on goal cMg_*waypoint.
    void reachPositionWithWaypoint(vpHomogeneousMatrix cMgoal, vpHomogeneousMatrix waypoint){
      cMgoal=cMgoal*waypoint;
      reachPositionWrtCamera(cMgoal);
    }

    //Reach position joint by joint to avoid overcurrent.
    void reachJointPositionByJoint(vpColVector desired_joints){
      vpColVector current_joints;
      robot_->getJointValues(current_joints);
      desired_joints[4]=current_joints[4];
      ros::Rate r(30);
      int joint = 0;
      while(joint<5) {
        while ( std::abs(desired_joints[joint] - current_joints[joint]) > 0.015 && ros::ok()) {
          //std::cout<<"Error: "<<(desired_joints-current_joints).euclideanNorm()<<"X"<<desired_joints<<"X"<<current_joints<<std::endl;
          vpColVector cmd_vel(5, 0.0);
          cmd_vel[joint] = desired_joints[joint] - current_joints[joint];
          while (cmd_vel[joint] > 0.5 || cmd_vel[joint] < -0.5) {
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

      while((desired_joints-current_joints).euclideanNorm()>0.015 && ros::ok()){
        //std::cout<<"Error: "<<(desired_joints-current_joints).euclideanNorm()<<"X"<<desired_joints<<"X"<<current_joints<<std::endl;
        vpColVector cmd_vel = desired_joints-current_joints;
        while(cmd_vel.euclideanNorm()>0.45) cmd_vel/=1.2;
        ROS_DEBUG_STREAM("Joint velocities sent: "<<cmd_vel);
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

      ROS_INFO("Probando joint by joint.");
      vpColVector js = initColVector(-0.8508754461803698, 0.3820938630929661, 1.42683938097373, -0.9393362034233482, 0.3839724354387525);
      reachJointPositionByJoint(js);
      reachJointPosition(js);

      //DF: Movimiento cartesiano utilizando incrementos y el movimientos joint by joint.
      ROS_INFO("Moving 10cm. joint by joint.");
      moveCartesianDistanceJointByJoint(0.0, 0.1, 0.15);

      ROS_INFO("Moving 10cm. with setCartesianVelocity.");
      moveCartesianDistance(0.0, 0.2, 0.45);

      //Should test grasping object
      ROS_INFO("Open the gripper until manipulation aperture");
      openGripper(velocity_aperture_, gripper_manipulation_, max_current_);
      ROS_INFO("Close the gripper");
      openGripper(-velocity_aperture_, gripper_closed_, max_current_);

    }

    void testClose(){
      openGripper(-velocity_aperture_, gripper_closed_, max_current_);
    }

    void testCartesianWrtCamera(){
      vpHomogeneousMatrix cMg, I;
      cMg[0][3] = 0.195;
      cMg[1][3] = 0.174;
      cMg[2][3] = 0.738;
      reachPositionWrtCamera(cMg);
      ROS_INFO("With waypoint");
      reachPositionWithWaypoint(cMg, I);
    }

    void testCartesian(){
      vpHomogeneousMatrix bMg;
      bMg[0][3] = -0.0495;
      bMg[1][3] = 0.031;
      bMg[2][3] = 0.565;
      reachCartesianPosition(bMg);
    }

    void testCartesianAbsMoves(){

      testCartesianWrtCamera();
      testCartesian();

      //TEST WITH VISION
      cMgFromTF();
      reachPositionWrtCameraJointByJoint(cMg_);
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_test");
  ArmWrapper robot;
  //Needed to udate the postion...
  ros::Duration(0.3).sleep();
  ros::spinOnce();

  //Test simple moving functions.
  //robot.testFunctions();

  //Test grasping with processing
  //robot.testCartesianAbsMoves();

  //robot.cMgFromTF();
  //robot.reachPositionWrtCamera(robot.getcMg());

  robot.testClose();



  return 1;
}

