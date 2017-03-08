/*
 * PMGraspPlanning.cpp
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#include <pm_manipulation/pm_grasp_planning.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>

#include <visp/vpHomogeneousMatrix.h>

#include <boost/bind.hpp>
#include <vector>
#include <algorithm>

typedef pcl::PointXYZRGB PointT;

void PMGraspPlanning::proccessScene(){

}
void PMGraspPlanning::getGraspHypothesis(){

}

void PMGraspPlanning::perceive() {

  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>), cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ()), cloud_plane (new pcl::PointCloud<PointT> ());

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>), cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);

  PCLTools<PointT>::applyZAxisPassthrough(cloud_, cloud_filtered, 0, 3);
  ROS_INFO_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");

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
  cyl_seg.apply(cloud_cylinder, coefficients_cylinder);//coefficients_cylinder = PCLTools::cylinderSegmentation(cloud_filtered2, cloud_normals2, cloud_cylinder, cylinder_distance_threshold_, cylinder_iterations_, radious_limit_);

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

  tf::Vector3 perp(coefficients_cylinder->values[4], -coefficients_cylinder->values[3], 0);
  perp=perp.normalize();

  tf::Vector3 result=tf::tfCross( perp, axis_dir).normalize();

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
  cMo[0][0]=perp.x(); cMo[0][1]=axis_dir.x(); cMo[0][2]=result.x();cMo[0][3]=mean.x;
  cMo[1][0]=perp.y(); cMo[1][1]=axis_dir.y(); cMo[1][2]=result.y();cMo[1][3]=mean.y;
  cMo[2][0]=perp.z(); cMo[2][1]=axis_dir.z(); cMo[2][2]=result.z();cMo[2][3]=mean.z;
  cMo[3][0]=0;cMo[3][1]=0;cMo[3][2]=0;cMo[3][3]=1;
  ROS_DEBUG_STREAM("cMo is...: " << std::endl << cMo << "Is homog: " << cMo.isAnHomogeneousMatrix()?"yes":"no");

  vispToTF.addTransform(cMo, camera_frame_name, "object_frame", "cMo");
  vispToTF.addTransform(cMg, camera_frame_name, "grasp_frame", "cMg");

  //DEBUG Print MAX and MIN frames
  /*vpHomogeneousMatrix cMg2(cMo);
  cMg2[0][3]=max.x;
  cMg2[1][3]=max.y;
  cMg2[2][3]=max.z;
  vispToTF.addTransform(cMg2, "/sense3d", "/max", "3");
  cMg2[0][3]=min.x;
  cMg2[1][3]=min.y;
  cMg2[2][3]=min.z;
  vispToTF.addTransform(cMg2, "/sense3d", "/min", "4");*/

  //Compute modified cMg from cMo
  recalculate_cMg();
}

/** Compute cMg from cMo. This function is inherited from MAR where
 * the repositioning is done using an UI
 */
void PMGraspPlanning::recalculate_cMg(){

  //Set grasp config values from interface int values.
  intToConfig();
  vpHomogeneousMatrix oMg;

  //Apply rotations and traslation to reposition the grasp frame.
  vpHomogeneousMatrix grMgt0(0,along_,0,0,0,0);
  vpHomogeneousMatrix gMgrZ(0,0,0,0,0,1.57);
  vpHomogeneousMatrix gMgrX(0,0,0,1.57,0,0);
  vpHomogeneousMatrix gMgrY(0,0,0,0,0,angle_);
  vpHomogeneousMatrix grMgt(rad_,0,0,0,0,0);
  oMg = grMgt0 * gMgrZ * gMgrX * gMgrY * grMgt;
  cMg = cMo * oMg ;

  cMg=cMg * vpHomogeneousMatrix(0,0,0,0,1.57,0) * vpHomogeneousMatrix(0,0,0,0,0,1.57);

  //Compute bMg and plan a grasp on bMg
  //vpHomogeneousMatrix bMg=bMc*cMg;
  //std::cerr << "bMg is: " << std::endl << bMg << std::endl;

  vispToTF.resetTransform( cMg, "cMg");

  //TONI DEBUG: VISUALIZE CYLINDER DETECTION FRAMES.
  //vispToTF.publish();

  // Send detected cylinder marker.
  ros::NodeHandle nh;
  vpHomogeneousMatrix cylinder;
  cylinder = cMo * vpHomogeneousMatrix(0, 0, 0, 1.57, 0, 0);
  MarkerPublisher markerPub( cylinder, camera_frame_name, "visualization_marker", nh);
  markerPub.setCylinder( cylinder, cloud_->header.frame_id, radious * 2, height, 15);
  markerPub.publish();
}

/// Set config values: from int sliders to float values.
void PMGraspPlanning::intToConfig(){
  angle_=iangle*(2.0*M_PI/360.0);
  rad_=-irad/100.0;
  along_=(ialong-20)/100.0;//to allow 20 cm negative.... should allow a range based on minMax distance
}

///Ordenar en función de la proyección del punto sobre el eje definido
///por axis_point_g y normal_g (globales)
bool PMGraspPlanning::sortFunction(const PointT& d1, const PointT& d2)
{
  double t1 = (normal_g.x()*(d1.x-axis_point_g.x) + normal_g.y()*(d1.y-axis_point_g.y) + normal_g.z()*(d1.z-axis_point_g.z))/(pow(normal_g.x(),2) + pow(normal_g.y(),2) + pow(normal_g.z(),2));
  double t2 = (normal_g.x()*(d2.x-axis_point_g.x) + normal_g.y()*(d2.y-axis_point_g.y) + normal_g.z()*(d2.z-axis_point_g.z))/(pow(normal_g.x(),2) + pow(normal_g.y(),2) + pow(normal_g.z(),2));

  return t1 < t2;
}

///Obtiene los máximos y mínimos del cilindro para encontrar la altura del cilindro con un margen
///de descarte de outlier percentage (normalmente 10%).
void PMGraspPlanning::getMinMax3DAlongAxis(const pcl::PointCloud<PointT>::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT axis_point, tf::Vector3 * normal, double outlier_percentage)
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
  std::sort(list.begin(), list.end(),  boost::bind(&PMGraspPlanning::sortFunction, this, _1, _2));
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
  ///@ TODO MAke a list with this points and show it.

}

/// @ TODO  Add d,a,r as parameters.
void PMGraspPlanning::generateGraspList()
{
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
        cMg = cMo * oMg;
        vpHomogeneousMatrix rot(0, 0, 0, 0, 1.57, 0);
        cMg=cMg*rot;
        cMg_list.push_back(cMg);
      }
    }
  }
}

//As a kinematic filter will be applied, I prefer to do it externally to avoid adding more deps.
// @ TODO Improve this function
void PMGraspPlanning::filterGraspList(){

  std::list<vpHomogeneousMatrix>::iterator i = cMg_list.begin();
  while (i != cMg_list.end())
  {
    if (!false)//Replace with desired condition
    {
      cMg_list.erase(i++);  // alternatively, i = items.erase(i);
    }
  }
}


