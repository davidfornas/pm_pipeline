/*
 * RankingGraspPlanner
 *
 *  Created on: 05/02/2018
 *      Author: dfornas
 */
#include <pm_manipulation/ranking_grasp_planner.h>
#include <std_msgs/Float32MultiArray.h>

/** Publish grasp list sequentially in TF. **/
void RankingGraspPlanner::publishGraspList(){
  while (ros::ok()){
    for (std::list<GraspHypothesis>::iterator it=grasps.begin(); it!=grasps.end(); ++it){
      ROS_INFO_STREAM("Displaying grasp in TF...");
      vispToTF_.resetTransform( (*it).cMg, "cMg");
      vispToTF_.resetTransform( (*it).cMg_ik, "cMg_ik");
      vispToTF_.publish();
      ros::spinOnce();
      ros::Duration(0.3).sleep();
    }
  }
}

/** Publish grasp scores and metrics */
void RankingGraspPlanner::publishGraspData( int grasp_id ){

  std::list<GraspHypothesis>::iterator it = grasps.begin();

  if (grasps.size() > grasp_id)
    std::advance(it, grasp_id);
  else
    return;

  std_msgs::Float32 msg;
  msg.data = (*it).overall_score;
  score_pub_.publish( msg );


  std_msgs::Int32 smsg;
  smsg.data = (int) grasps.size();
  list_size_pub_.publish( smsg );

  std_msgs::String text;
  std::ostringstream stringStream2;
  stringStream2 << "Distance to desired cMg score: " << (*it).distance_ik_score << std::endl <<
                "Angle to desired cMg score: " <<  (*it).angle_ik_score << std::endl <<
                "Distance to the object centroid score: " <<  (*it).distance_score << std::endl <<
                "Angle with the object axis score: " <<  (*it).angle_axis_score << std::endl;
  text.data = stringStream2.str();
  score_description_pub_.publish(text);

  if( (*it).measures[0] > 0.00000001 || (*it).measures[1] > 0.00000001 || (*it).measures[2] > 0.00000001 ) {
    std::ostringstream stringStream3;
    stringStream3 << "M0: " << (*it).measures[0] << "   M1: " << (*it).measures[1] << std::endl <<
                     "M2: " << (*it).measures[2] << "   M3: " << (*it).measures[3] << std::endl <<
                     "M4: " << (*it).measures[4] << "   M5: " << (*it).measures[5];
    text.data = stringStream3.str();
    metrics_pub_.publish(text);
  }

  vispToTF_.resetTransform( (*it).cMg, "cMg");
  vispToTF_.resetTransform( (*it).cMg_ik, "cMg_ik");
  vispToTF_.publish();
  ros::spinOnce();

}


vpHomogeneousMatrix RankingGraspPlanner::getGrasp_cMg( int grasp_id ) {
  std::list<GraspHypothesis>::iterator it = grasps.begin();
  if (grasps.size() > grasp_id) {
    std::advance(it, grasp_id);
    return (*it).cMg;
  }else {
    return vpHomogeneousMatrix();
  }
}

vpHomogeneousMatrix RankingGraspPlanner::getGrasp_cMg_ik( int grasp_id ) {
  std::list<GraspHypothesis>::iterator it = grasps.begin();
  if (grasps.size() > grasp_id) {
    std::advance(it, grasp_id);
    return (*it).cMg_ik;
  }else {
    return vpHomogeneousMatrix();
  }
}

bool RankingGraspPlanner::generateGraspList(){
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
  ROS_DEBUG("Hypothesis generated.");
  grasps.sort(sortByScore);
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
  bMg_fk = robot.directKinematics( final_joints2 );
  g.cMg_ik = bMc.inverse() * bMg_fk;
  //Compute score
  g.distance_ik_score = ( g.cMg.getCol(3) - g.cMg_ik.getCol(3) ).euclideanNorm();
  g.angle_ik_score = abs( VispTools::angle( g.cMg.getCol(2), g.cMg_ik.getCol(2) ) );
  g.angle_axis_score = abs( abs( VispTools::angle( cMo.getCol(1), g.cMg_ik.getCol(2) ) ) -1 );//Angle between cylinder axis and grasp axis.1 rad is preferred
  g.distance_score = abs( ( cMo.getCol(3) - g.cMg_ik.getCol(3) ).euclideanNorm() - 0.35 );//35cm is preferred
  g.overall_score = g.distance_ik_score * 100 + g.angle_ik_score *10 + g.angle_axis_score + g.distance_score * 2;//Should be argued. Now is only a matter of priority.
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
  grasps.sort(sortByScore);
  ROS_DEBUG_STREAM(num_grasps << "grasps read and sorted.");
  return true;
}

void SQRankingGraspPlanner::graspCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){

  grasps_read++;
  GraspHypothesis g;
  g.cMo = cMo;

  g.oMg[0][0] = msg->data[2];  g.oMg[0][1] = msg->data[3];  g.oMg[0][2] = msg->data[4];  g.oMg[0][3] = msg->data[5];
  g.oMg[1][0] = msg->data[6];  g.oMg[1][1] = msg->data[7];  g.oMg[1][2] = msg->data[8];  g.oMg[1][3] = msg->data[9];
  g.oMg[2][0] = msg->data[10]; g.oMg[2][1] = msg->data[11]; g.oMg[2][2] = msg->data[12]; g.oMg[2][3] = msg->data[13];
  g.oMg[3][0] = msg->data[14]; g.oMg[3][1] = msg->data[15]; g.oMg[3][2] = msg->data[16]; g.oMg[3][3] = msg->data[17];
  g.oMg = vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0) * g.oMg;

  g.cMg = g.cMo * g.oMg;

  g.preshapes[0] = msg->data[0];
  g.preshapes[1] = msg->data[1];
  g.measures[0] = msg->data[18];
  g.measures[1] = msg->data[19];
  g.measures[2] = msg->data[20];
  g.measures[3] = msg->data[21];
  g.measures[4] = msg->data[22];
  g.measures[5] = msg->data[23];

  generateGraspScores(g);
  grasps.push_back(g);
}

void SQRankingGraspPlanner::generateGraspScores( GraspHypothesis & grasp ) {

  vpHomogeneousMatrix bMg = bMc * grasp.cMg;
  vpColVector final_joints(5), final_joints2(5);
  final_joints = robot.armIK(bMg);
  vpHomogeneousMatrix bMg_fk;
  final_joints2[0] = final_joints[0];
  final_joints2[1] = final_joints[1];
  final_joints2[2] = final_joints[2];
  final_joints2[3] = 1.57;
  final_joints2[4] = 0;
  bMg_fk = robot.directKinematics(final_joints2);
  grasp.cMg_ik = bMc.inverse() * bMg_fk;
  //Compute scores
  grasp.distance_ik_score = ( grasp.cMg.getCol(3) - grasp.cMg_ik.getCol(3)).euclideanNorm();
  grasp.angle_ik_score = abs( VispTools::angle(grasp.cMg.getCol(2), grasp.cMg_ik.getCol(2)));
  grasp.angle_axis_score = abs(abs( VispTools::angle(grasp.cMo.getCol(1), grasp.cMg_ik.getCol(2))) - 1);//Angle between cylinder axis and grasp axis.1 rad is preferred
  grasp.distance_score = abs(( grasp.cMo.getCol(3) - grasp.cMg_ik.getCol(3) ).euclideanNorm() - 0.35);//35cm is preferred
  grasp.overall_score = grasp.distance_ik_score * 100 + grasp.angle_ik_score * 10 + grasp.angle_axis_score +
                        grasp.distance_score * 2;//Should be argued. Now is only a matter of priority.
}

//Filter grasp list using different constraints.
void SQRankingGraspPlanner::filterGraspList(){

  double min_dist_to_floor = 0.10;

  std::list<GraspHypothesis>::iterator it = grasps.begin();
  while (it != grasps.end())
  {
    PointT center;
    center.x = (*it).cMg[0][3];
    center.y = (*it).cMg[1][3];
    center.z = (*it).cMg[2][3];
    if (pose_estimation->bg_remove->signedDistanceToPlane( center ) > -min_dist_to_floor)
    {
      it = grasps.erase(it);
    }else if ((*it).overall_score > 15.) {
      it = grasps.erase(it);
    }else{
      it++;
    }
  }
}

/** Publish grasp list sequentially in TF. **/
void SQRankingGraspPlanner::publishGraspList( double wait_time ){
  if(grasps.size() == 0){
    ROS_INFO_STREAM("Empty grasp hyp. list");
    return;
  }
  for (std::list<GraspHypothesis>::iterator it=grasps.begin(); it!=grasps.end(); ++it) {
    vispToTF_.resetTransform( (*it).cMg, "cMg");
    vispToTF_.resetTransform( (*it).cMo, "cMo");
    vispToTF_.resetTransform( (*it).oMg, "oMg");
    vispToTF_.resetTransform( (*it).cMg_ik, "cMg_ik");
    vispToTF_.publish();
    ROS_INFO_STREAM("Score" << (*it).overall_score );
    PointT center;
    center.x = (*it).cMg[0][3];
    center.y = (*it).cMg[1][3];
    center.z = (*it).cMg[2][3];
    ROS_INFO_STREAM("Distance" << pose_estimation->bg_remove->signedDistanceToPlane( center )) ;
    ros::spinOnce();
    ros::Duration( wait_time ).sleep();
  }
}

/** Get best rasp based on ranking. Bigger score is worse. */
vpHomogeneousMatrix SQRankingGraspPlanner::getBestGrasp(){
  if(grasps.size()>0) {
    grasps.sort(sortByScore);
    ROS_INFO_STREAM("Best grasp: " << grasps.front().overall_score);
    ROS_INFO_STREAM("List size: " << grasps.size() << "Worst grasp: " << grasps.back().overall_score);
    return grasps.front().cMg;
  }else{
    ROS_ERROR("Empty Grasp Hyp. List");
    return vpHomogeneousMatrix();
  }
}
