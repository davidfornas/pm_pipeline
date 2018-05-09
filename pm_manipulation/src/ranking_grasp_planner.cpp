/*
 * RankingGraspPlanner
 *
 *  Created on: 05/02/2018
 *      Author: dfornas
 */
#include <pm_manipulation/ranking_grasp_planner.h>
#include <std_msgs/Float32MultiArray.h>

bool RankingGraspPlanner::generateGraspList()
{
  pose_estimation->initialize();
  cMo = pose_estimation->get_cMo();
  getbMc();
  ROS_INFO("Generating Hypothesis...");
  for (double d = -0.2; d <= 0.2; d += 0.04)
  {
    for (double a = 40; a <= 140; a += 8)
    {
      for (double r = -0.5; r <= -0.2; r += 0.05)
      {
        double angle = a * 2 * 3.1416 / 360; // Deg -> Rad

        vpHomogeneousMatrix grMgt0(0, d, 0, 0, 0, 0);
        vpHomogeneousMatrix gMgrZ(0, 0, 0, 0, 0, 1.57);
        vpHomogeneousMatrix gMgrX(0, 0, 0, 1.57, 0, 0);
        vpHomogeneousMatrix gMgrY(0, 0, 0, 0, 0, angle);
        vpHomogeneousMatrix grMgt(r, 0, 0, 0, 0, 0);
        vpHomogeneousMatrix oMg = grMgt0 * gMgrZ * gMgrX * gMgrY * grMgt;
        vpHomogeneousMatrix cMg = cMo * oMg;
        cMg=cMg * vpHomogeneousMatrix(0,0,0,0,1.57,0) * vpHomogeneousMatrix(0,0,0,0,0,3.14);
        GraspHypothesis g = generateGraspHypothesis( cMg );
        grasps.push_back(g);
        }
    }
  }
  ROS_INFO("Hypothesis generated.");
  //std::sort(grasps.begin(), grasps.end(), sortByScore);
  return true;
}

//Generate Hypotheiss list @TODO Fix and Improve
GraspHypothesis RankingGraspPlanner::generateGraspHypothesis( vpHomogeneousMatrix cMg) {

  GraspHypothesis g;
  g.cMg = cMg;
  vpHomogeneousMatrix bMg = bMc * g.cMg;
  vpColVector final_joints(5), final_joints2(5);
  final_joints = robot.armIK(bMg);
  vpHomogeneousMatrix bMg_fk;
  final_joints2[0] = final_joints[0];
  final_joints2[1] = final_joints[1];
  final_joints2[2] = final_joints[2];
  final_joints2[3] = 1.57;
  final_joints2[4] = 0;
  bMg_fk = robot.directKinematics(final_joints2);
  g.cMg_ik = bMc.inverse()*bMg_fk;
  //Compute score
  g.distance_ik_score = (g.cMg.getCol(3) - g.cMg_ik.getCol(3)).euclideanNorm();
  g.angle_ik_score = abs(angle(g.cMg.getCol(2), g.cMg_ik.getCol(2)));
  g.angle_axis_score = abs(abs(angle(cMo.getCol(1), g.cMg_ik.getCol(2)))-1);//Angle between cylinder axis and grasp axis.1 rad is preferred
  g.distance_score = abs((cMo.getCol(3) - g.cMg_ik.getCol(3)).euclideanNorm()-0.35);//35cm is preferred
  g.overall_score = g.distance_ik_score * 100 + g.angle_ik_score *10+g.angle_axis_score+g.distance_score*2;//Should be argued. Now is only a matter of priority.
  return g;
}

void RankingGraspPlanner::getbMc(){
  //Get bMc from TF...
  ROS_INFO("Waiting for base to camera (arm5/kinematic_base to stereo)");
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
  bMc=tfToVisp(bMc_tf);
};

vpHomogeneousMatrix RankingGraspPlanner::tfToVisp(tf::StampedTransform matrix_tf){
  vpHomogeneousMatrix matrix_visp;
  matrix_visp[0][0]=matrix_tf.getBasis()[0][0]; matrix_visp[0][1]=matrix_tf.getBasis()[0][1]; matrix_visp[0][2]=matrix_tf.getBasis()[0][2]; matrix_visp[0][3]=matrix_tf.getOrigin().x();
  matrix_visp[1][0]=matrix_tf.getBasis()[1][0]; matrix_visp[1][1]=matrix_tf.getBasis()[1][1]; matrix_visp[1][2]=matrix_tf.getBasis()[1][2]; matrix_visp[1][3]=matrix_tf.getOrigin().y();
  matrix_visp[2][0]=matrix_tf.getBasis()[2][0]; matrix_visp[2][1]=matrix_tf.getBasis()[2][1]; matrix_visp[2][2]=matrix_tf.getBasis()[2][2]; matrix_visp[2][3]=matrix_tf.getOrigin().z();
  return matrix_visp;
}

//As a kinematic filter will be applied, I prefered to do it externally to avoid adding more deps.
void RankingGraspPlanner::filterGraspList(){

//  std::list<vpHomogeneousMatrix>::iterator i = grasp_list.begin();
//  while (i != grasp_list.end())
//  {
//    if (!false)//Replace with desired condition
//    {
//      grasp_list.erase(i++);  // alternatively, i = items.erase(i);
//    }
//  }
}

bool SQRankingGraspPlanner::generateGraspList() {

  ROS_INFO("SQ Pose estimation...");
  bool success = pose_estimation->process();
  if(!success) return false;
  cMo = pose_estimation->get_cMo();

  ROS_INFO("Grasp planning...");
  std_msgs::Float32MultiArray params;
  params.data.clear();
  //Data is an array with num_grasps, anglerange, deltaspace...
  params.data.push_back(num_grasps);
  params.data.push_back(anglerange);
  params.data.push_back(deltaspace);
  // ... Rolls arange
  params.data.push_back(roll_arange_init);
  params.data.push_back(roll_arange_end);
  params.data.push_back(roll_arange_step);
  // ... Standoffs (variable size)
  params.data.push_back(standoffs[0]);
  params.data.push_back(standoffs[1]);
  params.data.push_back(standoffs[2]);
  params.data.push_back(standoffs[3]);
  ros::Duration(0.5).sleep();
  params_pub.publish(params);
  ros::spinOnce();

  grasps_read = 0;
  while(grasps_read < num_grasps){
    ros::spinOnce();
  }
  ROS_DEBUG_STREAM(num_grasps << "Grasps read");
  return true;
}

void SQRankingGraspPlanner::graspCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){

  grasps_read++;
  ORGraspHypothesis g;
  //GENERATE IK AND SCORES LIKE IN g0 = generateGraspHypothesis( cMg );
  g.cMo = cMo;

  g.oMg[0][0] = msg->data[2];  g.oMg[0][1] = msg->data[3];  g.oMg[0][2] = msg->data[4];  g.oMg[0][3] = msg->data[5];
  g.oMg[1][0] = msg->data[6];  g.oMg[1][1] = msg->data[7];  g.oMg[1][2] = msg->data[8];  g.oMg[1][3] = msg->data[9];
  g.oMg[2][0] = msg->data[10]; g.oMg[2][1] = msg->data[11]; g.oMg[2][2] = msg->data[12]; g.oMg[2][3] = msg->data[13];
  g.oMg[3][0] = msg->data[14]; g.oMg[3][1] = msg->data[15]; g.oMg[3][2] = msg->data[16]; g.oMg[3][3] = msg->data[17];

  g.cMg = g.cMo * g.oMg;

  g.preshapes[0] = msg->data[0];
  g.preshapes[1] = msg->data[1];

  g.measures[0] = msg->data[18];
  g.measures[1] = msg->data[19];
  g.measures[2] = msg->data[20];
  g.measures[3] = msg->data[21];
  g.measures[4] = msg->data[22];
  g.measures[5] = msg->data[23];

  grasps.push_back(g);
}
