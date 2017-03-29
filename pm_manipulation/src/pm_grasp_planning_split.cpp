/*
 * PMGraspPlanning.cpp
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#include <pm_manipulation/pm_grasp_planning_split.h>
#include <pm_tools/pcl_tools.h>

#include <visp/vpHomogeneousMatrix.h>

#include <boost/bind.hpp>
#include <vector>
#include <algorithm>

typedef pcl::PointXYZ PointT;

void PMGraspPlanningSplit::perceive() {

  initialized_ = true;

  if( do_ransac ){
    ROS_INFO_STREAM("Computing pose with RANSAC...");
    doRansac();    
    ROS_INFO_STREAM("RANSAC finished.");
  }else{
    ROS_INFO_STREAM("Waiting for pose on topic: " << topic_name);
    //geometry_msgs::Pose::ConstPtr message = ros::topic::waitForMessage< geometry_msgs::Pose >(topic_name);
    if (!initialized_){
      geometry_msgs::Pose::ConstPtr message = ros::topic::waitForMessage< geometry_msgs::Pose >(topic_name);
      cMo = VispTools::vispHomogFromGeometryPose(*message);
    }else{
      geometry_msgs::Pose::ConstPtr message = ros::topic::waitForMessage< geometry_msgs::Pose >(topic_name, ros::Duration(2));
      cMo = VispTools::vispHomogFromGeometryPose(*message);
      if (message == NULL){
           ROS_INFO("No pose messages received in 10 seconds");
           return;
      }
    }
    ROS_INFO_STREAM("Pose received");
    //Director vectors: cylinder axis and perpendicular vector.
    if(change_z_){
      // @TODO fix this!!!
      ROS_INFO_STREAM("Changing Z...");
      tf::Vector3 axis_dir(cMo[0][1], cMo[1][1], cMo[2][1]);
      axis_dir=axis_dir.normalize();
      tf::Vector3 perp = tf::Vector3(0,0,1) - ( axis_dir.dot( tf::Vector3(0,0,1) ) * axis_dir );
      perp=perp.normalize();

      tf::Vector3 result=tf::tfCross( perp, axis_dir).normalize();
      result = -result;

      cMo[0][0]=result.x(); cMo[0][2]=perp.x();
      cMo[1][0]=result.y(); cMo[1][2]=perp.y();
      cMo[2][0]=result.z(); cMo[2][2]=perp.z();
    }
    //vispToTF.addTransform(cMo, "/world", "/amphora_from_pose", "cMo");
  }
  //Compute modified cMg from cMo
  recalculate_cMg();
}

void PMGraspPlanningSplit::doRansac() {

  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>), cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ()), cloud_plane (new pcl::PointCloud<PointT> ());

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>), cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  coefficients_plane = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;
  coefficients_cylinder = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;

  PCLTools<PointT>::applyZAxisPassthrough(cloud_, cloud_filtered, 0, 3);
  ROS_DEBUG_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");

  // @ TODO : Add more filters -> downsampling and radial ooutlier removal.
  PCLTools<PointT>::estimateNormals(cloud_filtered, cloud_normals);

  PlaneSegmentation<PointT> plane_seg(cloud_filtered, cloud_normals);
  plane_seg.setDistanceThreshold(plane_distance_threshold_);
  plane_seg.setIterations(plane_iterations_);
  plane_seg.apply(cloud_filtered2, cloud_normals2, cloud_plane, coefficients_plane);

  CylinderSegmentation<PointT> cyl_seg(cloud_filtered2, cloud_normals2);
  cyl_seg.setDistanceThreshold(cylinder_distance_threshold_);
  cyl_seg.setIterations(cylinder_iterations_);
  cyl_seg.setRadiousLimit(radious_limit_);
  cyl_seg.apply(cloud_cylinder, coefficients_cylinder);
  //coefficients_cylinder = PCLTools::cylinderSegmentation(cloud_filtered2, cloud_normals2, cloud_cylinder, cylinder_distance_threshold_, cylinder_iterations_, radious_limit_);
  // DEBUG
  //PCLTools<PointT>::showClouds(cloud_plane, cloud_cylinder, coefficients_plane, coefficients_cylinder);

  //Grasp points
  PointT mean, max, min;
  //Cylinder origin
  PointT axis_point;
  axis_point.x=coefficients_cylinder->values[0];
  axis_point.y=coefficients_cylinder->values[1];
  axis_point.z=coefficients_cylinder->values[2];

  //Director vectors: cylinder axis and perpendicular vector.
  tf::Vector3 axis_dir(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);
  axis_dir=axis_dir.normalize();

  tf::Vector3 plane_normal(coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2]);
  tf::Vector3 perp = plane_normal - ( axis_dir.dot( plane_normal ) * axis_dir );
  perp=perp.normalize();

  tf::Vector3 result=tf::tfCross( perp, axis_dir).normalize();
  result = -result;

  getMinMax3DAlongAxis(cloud_cylinder, &max, &min, axis_point, &axis_dir, 0.1);
  //Mean point taking into account only a 90% of the points (0.1).
  mean.x=(max.x+min.x)/2;mean.y=(max.y+min.y)/2;mean.z=(max.z+min.z)/2;
  coefficients_cylinder->values[0]=mean.x;
  coefficients_cylinder->values[1]=mean.y;
  coefficients_cylinder->values[2]=mean.z;

  //Cylinder properties
  radious=coefficients_cylinder->values[6];
  height=sqrt((max.x-min.x)*(max.x-min.x)+(max.y-min.y)*(max.y-min.y)+(max.z-min.z)*(max.z-min.z));

  // @ NOTE Ahora mismo el end-efector cae dentro del cilindro en vez de en superfície.
  //Esto está relativamente bien pero no tenemos en cuenta la penetración. Sin embargo, la
  //tenemos en cuenta luego al separanos el radio así que no hay problema en realidad.
  cMo[0][0]=result.x(); cMo[0][1]=axis_dir.x(); cMo[0][2]=perp.x();cMo[0][3]=mean.x;
  cMo[1][0]=result.y(); cMo[1][1]=axis_dir.y(); cMo[1][2]=perp.y();cMo[1][3]=mean.y;
  cMo[2][0]=result.z(); cMo[2][1]=axis_dir.z(); cMo[2][2]=perp.z();cMo[2][3]=mean.z;
  cMo[3][0]=0;cMo[3][1]=0;cMo[3][2]=0;cMo[3][3]=1;
  ROS_DEBUG_STREAM("cMo is...: " << std::endl << cMo << "Is homog: " << cMo.isAnHomogeneousMatrix()?"yes":"no");
  vispToTF.addTransform(cMo, camera_frame_name, "/amphora", "cMo");

  /*
  vpHomogeneousMatrix wMo;
  wMo = wMc_ * cMo;

  tf::Vector3 axis_dir2(wMo[0][1], wMo[1][1], wMo[2][1]);
  axis_dir2=axis_dir2.normalize();
  tf::Vector3 perp2 = tf::Vector3(0,0,1) - ( axis_dir2.dot( tf::Vector3(0,0,1) ) * axis_dir2 );
  perp2=perp2.normalize();

  tf::Vector3 result2=tf::tfCross( perp2, axis_dir2).normalize();
  result2 = -result2;

  wMo[0][0]=result2.x(); wMo[0][2]=perp2.x();
  wMo[1][0]=result2.y(); wMo[1][2]=perp2.y();
  wMo[2][0]=result2.z(); wMo[2][2]=perp2.z();

  vispToTF.addTransform(wMo, "/world", "/amphora", "wMo");
  */

  //TONI DEBUG vispToTF.addTransform(cMg, camera_frame_name, "grasp_frame", "cMg");
  //cMo = wMc_.inverse() * wMo;
  //Compute modified cMg from cMo
  recalculate_cMg();
  previous_cMo_ = cMo;
}

//THIS SHOULD BE IMPROVED
void PMGraspPlanningSplit::redoRansac() {

  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>), cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ()), cloud_plane (new pcl::PointCloud<PointT> ());

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>), cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  coefficients_plane = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;
  coefficients_cylinder = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;
  PCLTools<PointT>::applyZAxisPassthrough(cloud_, cloud_filtered, 0, 3);
  ROS_INFO_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");

  // @ TODO : Add more filters -> downsampling and radial ooutlier removal.
  PCLTools<PointT>::estimateNormals(cloud_filtered, cloud_normals);

  PlaneSegmentation<PointT> plane_seg(cloud_filtered, cloud_normals);
  plane_seg.setDistanceThreshold(plane_distance_threshold_);
  plane_seg.setIterations(plane_iterations_);
  plane_seg.apply(cloud_filtered2, cloud_normals2, cloud_plane, coefficients_plane);
  //// @TODO plane_seg.removeFromCoefficients(cloud_filtered2, cloud_normals2, cloud_plane, coefficients_plane);

  CylinderSegmentation<PointT> cyl_seg(cloud_filtered2, cloud_normals2);
  cyl_seg.setDistanceThreshold(cylinder_distance_threshold_);
  cyl_seg.setIterations(cylinder_iterations_);
  cyl_seg.setRadiousLimit(radious_limit_);
  cyl_seg.apply(cloud_cylinder, coefficients_cylinder);

  //Grasp points
  PointT mean, max, min;
  //Cylinder origin
  PointT axis_point;
  axis_point.x=coefficients_cylinder->values[0];
  axis_point.y=coefficients_cylinder->values[1];
  axis_point.z=coefficients_cylinder->values[2];

  //Director vectors: cylinder axis and perpendicular vector.
  tf::Vector3 axis_dir(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);
  axis_dir=axis_dir.normalize();

  tf::Vector3 plane_normal(coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2]);
  tf::Vector3 perp = plane_normal - ( axis_dir.dot( plane_normal ) * axis_dir );
  perp=perp.normalize();

  tf::Vector3 result=tf::tfCross( perp, axis_dir).normalize();
  result = -result;

  getMinMax3DAlongAxis(cloud_cylinder, &max, &min, axis_point, &axis_dir, 0.1);
  //Mean point taking into account only a 90% of the points (0.1).
  mean.x=(max.x+min.x)/2;mean.y=(max.y+min.y)/2;mean.z=(max.z+min.z)/2;
  coefficients_cylinder->values[0]=mean.x;
  coefficients_cylinder->values[1]=mean.y;
  coefficients_cylinder->values[2]=mean.z;

  //Cylinder properties
  radious=coefficients_cylinder->values[6];
  height=sqrt((max.x-min.x)*(max.x-min.x)+(max.y-min.y)*(max.y-min.y)+(max.z-min.z)*(max.z-min.z));

  // @ NOTE Ahora mismo el end-efector cae dentro del cilindro en vez de en superfície.
  //Esto está relativamente bien pero no tenemos en cuenta la penetración. Sin embargo, la
  //tenemos en cuenta luego al separanos el radio así que no hay problema en realidad.
  cMo[0][0]=result.x(); cMo[0][1]=axis_dir.x(); cMo[0][2]=perp.x();cMo[0][3]=mean.x;
  cMo[1][0]=result.y(); cMo[1][1]=axis_dir.y(); cMo[1][2]=perp.y();cMo[1][3]=mean.y;
  cMo[2][0]=result.z(); cMo[2][1]=axis_dir.z(); cMo[2][2]=perp.z();cMo[2][3]=mean.z;
  cMo[3][0]=0;cMo[3][1]=0;cMo[3][2]=0;cMo[3][3]=1;
  vispToTF.resetTransform(cMo, "cMo");

  ROS_INFO_STREAM("RANSAC REDONE");
  // @TODO Improve with a proper filter.
  double distance = abs(cMo[0][3]-previous_cMo_[0][3]) * abs(cMo[0][3]-previous_cMo_[0][3])
      + abs(cMo[1][3]-previous_cMo_[1][3]) * abs(cMo[1][3]-previous_cMo_[1][3])
      + abs(cMo[2][3]-previous_cMo_[2][3]) * abs(cMo[2][3]-previous_cMo_[2][3]);
  ROS_INFO_STREAM("Distance: "<< distance);
  if(distance < 0.2){
    tf::Vector3 y(cMo[0][1],cMo[1][1],cMo[2][1]);
    tf::Vector3 previous_y(previous_cMo_[0][1],previous_cMo_[1][1],previous_cMo_[2][1]);
    double angle_diff = y.angle(previous_y);
    ROS_INFO_STREAM("Angle: "<< angle_diff);
    if (angle_diff < 3.3 && angle_diff > 2.9)
      cMo = cMo * vpHomogeneousMatrix(0,1.57,0,0,0,0);

    previous_cMo_ = cMo;
  }else{//If bad (big distance), restore...
    cMo = previous_cMo_;
  }


  recalculate_cMg();

}


/** Publish cloud **/
void PMGraspPlanningSplit::publishCloudOnce(){

}

/** Load cloud to recompute RANSAC **/
void PMGraspPlanningSplit::loadCloud(){

}

/** Compute RANSAC taking into account a previous execution **/
void PMGraspPlanningSplit::perceiveOnline(){

}

void PMGraspPlanningSplit::computeMatrix( double angle, double rad, double along ){
vpHomogeneousMatrix oMg;

//Apply rotations and traslation to reposition the grasp frame.
vpHomogeneousMatrix grMgt0(0,along,0,0,0,0);
vpHomogeneousMatrix gMgrZ(0,0,0,0,0,1.57);
vpHomogeneousMatrix gMgrX(0,0,0,1.57,0,0);
vpHomogeneousMatrix gMgrY(0,0,0,0,0,angle);
vpHomogeneousMatrix grMgt(rad,0,0,0,0,0);
oMg = grMgt0 * gMgrZ * gMgrX * gMgrY * grMgt;
cMg = cMo * oMg ;

cMg=cMg * vpHomogeneousMatrix(0,0,0,0,1.57,0) * vpHomogeneousMatrix(0,0,0,0,0,3.14);
}

/** Compute cMg from cMo. This function is inherited from MAR where
 * the repositioning is done using an UI
 */
void PMGraspPlanningSplit::recalculate_cMg(){

  //Set grasp config values from interface int values.
  intToConfig();
  computeMatrix( angle_, rad_, along_ );

  if( iangle > 90 )
      cMg = cMg * vpHomogeneousMatrix(0,0,0,0,0,-3.14);

  //TONI DEBUG vispToTF.resetTransform( cMg, "cMg");

  //TONI DEBUG: VISUALIZE CYLINDER DETECTION FRAMES.
  vispToTF.publish();

  // Send detected cylinder marker.
  ros::NodeHandle nh;
  vpHomogeneousMatrix cylinder;
  cylinder = cMo * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0);
  MarkerPublisher markerPub( cylinder, camera_frame_name, "visualization_marker", nh);
  markerPub.setCylinder( cylinder, camera_frame_name, 0.15, 0.5, 15);
  markerPub.publish();

}

//Compute the best grasp from to approach directions along the cylinder axis.
void PMGraspPlanningSplit::getBestParams( double & angle, double & rad, double & along){

  computeMatrix( angle, rad, along);
  //tf::Vector3 grasp1(cMg[2][0],cMg[2][1],cMg[2][2]), zworld(0, 0, 1);
  tf::Vector3 grasp1(cMg[0][1],cMg[1][1],cMg[2][1]), zworld(0, 0, 1);
  double angle1 = grasp1.angle(zworld);

  computeMatrix( 180 - angle, rad, along);
  //tf::Vector3 grasp2(cMg[2][0],cMg[2][1],cMg[2][2]);
  tf::Vector3 grasp2(cMg[0][1],cMg[1][1],cMg[2][1]);
  double angle2 = grasp2.angle(zworld);

  if (angle2 > angle1)
    angle = 180 - angle;
}


/// Set config values: from int sliders to float values.
void PMGraspPlanningSplit::intToConfig(){
  angle_=iangle*(2.0*M_PI/360.0);
  rad_=-irad/100.0;
  along_=(ialong-20)/100.0;//to allow 20 cm negative.... should allow a range based on minMax distance
}

///Ordenar en función de la proyección del punto sobre el eje definido
///por axis_point_g y normal_g (globales)
bool PMGraspPlanningSplit::sortFunction(const PointT& d1, const PointT& d2)
{
  double t1 = (normal_g.x()*(d1.x-axis_point_g.x) + normal_g.y()*(d1.y-axis_point_g.y) + normal_g.z()*(d1.z-axis_point_g.z))/(pow(normal_g.x(),2) + pow(normal_g.y(),2) + pow(normal_g.z(),2));
  double t2 = (normal_g.x()*(d2.x-axis_point_g.x) + normal_g.y()*(d2.y-axis_point_g.y) + normal_g.z()*(d2.z-axis_point_g.z))/(pow(normal_g.x(),2) + pow(normal_g.y(),2) + pow(normal_g.z(),2));

  return t1 < t2;
}

///Obtiene los máximos y mínimos del cilindro para encontrar la altura del cilindro con un margen
///de descarte de outlier percentage (normalmente 10%).
void PMGraspPlanningSplit::getMinMax3DAlongAxis(const pcl::PointCloud<PointT>::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT axis_point, tf::Vector3 * normal, double outlier_percentage)
{
  axis_point_g=axis_point;
  normal_g=*normal;

  outlier_percentage = outlier_percentage / 2.0;

  PointT max_p = axis_point;
  double max_t = 0.0;
  PointT min_p = axis_point;
  double min_t = 0.0;
  std::vector<PointT> list;
  PointT* k;

  //Al tener la lista de todos los puntos podemos descartar los que esten fuera de un
  //determinado porcentaje (percentiles) para
  //Eliminar más outliers y ganar robustez.
  BOOST_FOREACH(const PointT& pt, cloud->points)
  {
    k=new PointT();
    k->x=pt.x*1;k->y=pt.y*1;k->z=pt.z*1;
    list.push_back(*k);
  }
  //Ordenamos con respecto al eje de direccion y tomamos P05 y P95
  std::sort(list.begin(), list.end(),  boost::bind(&PMGraspPlanningSplit::sortFunction, this, _1, _2));
  PointT max=list[(int)list.size()*outlier_percentage],min=list[(int)list.size()*(1-outlier_percentage)];
  //Proyección de los puntos reales a puntos sobre la normal.
  double t = (normal->x()*(max.x-axis_point.x) + normal->y()*(max.y-axis_point.y) + normal->z()*(max.z-axis_point.z))/(pow(normal->x(),2) + pow(normal->y(),2) + pow(normal->z(),2));
  PointT p;
  p.x = axis_point.x + normal->x() * t;
  p.y = axis_point.y + normal->y() * t;
  p.z = axis_point.z + normal->z() * t;
  *max_pt=p;
  t = (normal->x()*(min.x-axis_point.x) + normal->y()*(min.y-axis_point.y) + normal->z()*(min.z-axis_point.z))/(pow(normal->x(),2) + pow(normal->y(),2) + pow(normal->z(),2));
  p.x = axis_point.x + normal->x() * t;
  p.y = axis_point.y + normal->y() * t;
  p.z = axis_point.z + normal->z() * t;
  *min_pt=p;
}



