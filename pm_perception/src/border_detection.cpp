/*
 * border_detection.cpp
 *
 *  Created on: 18/03/2014
 *      Author: dfornas
 */
#include<ros/ros.h>
#include <tf/transform_listener.h>

#include <pm_perception/border_detection.h>
#include <pm_tools/pcl_tools.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>

//VISP
#include <visp/vpHomogeneousMatrix.h>

//Time measures
#include <ctime>

void ConcaveHullBorderDetection::process(){
  pcl::PointCloud<PointT>::Ptr  cloud_filtered (new pcl::PointCloud<PointT>), cloud_projected (new pcl::PointCloud<PointT>) ;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ()), cloud_plane (new pcl::PointCloud<PointT> ());

  // Build a filter to remove spurious NaNs

  clock_t begin = clock();

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud_);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-10, 10);
  pass.filter (*cloud_filtered);

  // Create the segmentation object
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  pcl::NormalEstimation<PointT, pcl::Normal> ne;  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  PlaneSegmentation plane_seg(cloud_filtered, cloud_normals);
  plane_seg.setDistanceThreshold(0.12);
  plane_seg.setIterations(100);
  plane_seg.apply(cloud_filtered2, cloud_normals2, cloud_plane, coefficients_plane);

  plane_normal_.resize(3);
  plane_normal_[0] = coefficients_plane->values[0];
  plane_normal_[1] = coefficients_plane->values[1];
  plane_normal_[2] = coefficients_plane->values[2];


  CylinderSegmentation cyl_seg(cloud_filtered2, cloud_normals2);
  cyl_seg.setDistanceThreshold(0.03);
  cyl_seg.setIterations(10000);
  cyl_seg.setRadiousLimit(0.1);
  cyl_seg.apply(cloud_cylinder, coefficients_cylinder);//coefficients_cylinder = PCLTools::cylinderSegmentation(cloud_filtered2, cloud_normals2, cloud_cylinder, cylinder_distance_threshold_, cylinder_iterations_, radious_limit_);
  // @ TODO cleaner solution
  cyl_seg.getInliers(inliers_cylinder);

  clock_t end = clock();
  std::cerr << "Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  begin = clock();

  // Project the CILINDER inliers into the PLANE model
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers_cylinder);
  proj.setInputCloud (cloud_filtered2);
  proj.setModelCoefficients (coefficients_plane);
  proj.filter (*cloud_projected);

  end = clock();
  std::cerr << "Elapsed projection time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
  begin = clock();

  //pcl::PCDWriter writer;
  //writer.write ("projected_cylinder.pcd", *cloud_projected, false);

  // Create a Concave Hull representation of the projected inliers
  pcl::ConcaveHull<PointT> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.1);
  border_cloud_ = boost::shared_ptr< pcl::PointCloud<PointT> >(new pcl::PointCloud<PointT>);
  chull.reconstruct (*border_cloud_);

  end = clock();
  std::cerr << "Elapsed concave hull time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  //writer.write ("chull.pcd", *border_cloud_, false);
  std::cout << border_cloud_->points.size() << std::endl;

  end = clock();
  std::cerr << "Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
}

void ConcaveHullBorderDetection::generatePath(){
  /*ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  visualization_msgs::Marker trajectory;
  trajectory.header.frame_id = "/stereo_down";//CAMERA FRAME
  trajectory.header.stamp = ros::Time::now();
  trajectory.ns = "points_and_lines";
  trajectory.action = visualization_msgs::Marker::ADD;
  trajectory.pose.orientation.w = 1.0;
  trajectory.id = 0;
  trajectory.type = visualization_msgs::Marker::LINE_LIST;
  trajectory.scale.x = 0.3;
  trajectory.color.r = 1.0;
  trajectory.color.a = 1.0;
  // Create the vertices
     geometry_msgs::Point p, p_old;
     p_old.x = border_cloud_->points[0].x;
     p_old.y = border_cloud_->points[0].y;
     p_old.z = border_cloud_->points[0].z;
     for (int i = 0; i < border_cloud_->points.size(); ++i)
     {
       p.x = border_cloud_->points[i].x;
       p.y = border_cloud_->points[i].y;
       p.z = border_cloud_->points[i].z;
       trajectory.points.push_back(p_old);
       trajectory.points.push_back(p);
       p_old.x = border_cloud_->points[i].x;
       p_old.y = border_cloud_->points[i].y;
       p_old.z = border_cloud_->points[i].z;
     }
     marker_pub.publish(trajectory);
   */
  nav_msgs::Path empty_path;
  path_=empty_path;
  path_.header.frame_id="/stereo_down";
  path_.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped mass_center;
  for (int i = 0; i < border_cloud_->points.size(); ++i)
  {
    mass_center.pose.position.x += border_cloud_->points[i].x;
    mass_center.pose.position.y += border_cloud_->points[i].y;
    mass_center.pose.position.z += border_cloud_->points[i].z;
  }
  mass_center.pose.position.x /= border_cloud_->points.size();
  mass_center.pose.position.y /= border_cloud_->points.size();
  mass_center.pose.position.z /= border_cloud_->points.size();
  // @DEBUG path_.poses.push_back(mass_center);

  for (int i = 0; i < border_cloud_->points.size(); ++i)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id="/stereo_down";
    //Postion
    p.pose.position.x = border_cloud_->points[i].x;
    p.pose.position.y = border_cloud_->points[i].y;
    p.pose.position.z = border_cloud_->points[i].z;

    //Orientation
    double xdiff = mass_center.pose.position.x - border_cloud_->points[i].x;
    double ydiff = mass_center.pose.position.y - border_cloud_->points[i].y;
    double zdiff = mass_center.pose.position.z - border_cloud_->points[i].z;

    vpColVector zaxis(3);//, xaxis(3);
    zaxis[0] = xdiff;
    zaxis[1] = ydiff;
    zaxis[2] = zdiff;
    zaxis=zaxis.normalize();

    vpColVector xaxis=vpColVector::crossProd( zaxis, plane_normal_ ).normalize();
    vpColVector yaxis=vpColVector::crossProd( zaxis, xaxis ).normalize();

    vpHomogeneousMatrix frame;
    frame[0][0]=xaxis[0]; frame[0][1]=yaxis[0]; frame[0][2]=zaxis[0];frame[0][3]=border_cloud_->points[i].x;
    frame[1][0]=xaxis[1]; frame[1][1]=yaxis[1]; frame[1][2]=zaxis[1];frame[1][3]=border_cloud_->points[i].y;
    frame[2][0]=xaxis[2]; frame[2][1]=yaxis[2]; frame[2][2]=zaxis[2];frame[2][3]=border_cloud_->points[i].z;
    frame[3][0]=0;        frame[3][1]=0;        frame[3][2]=0;       frame[3][3]=1;

    //Aplicamos una rotación y traslación para posicionar la garra mejor
    vpHomogeneousMatrix trans(0,0,-0.05,0,0,0);
    vpHomogeneousMatrix rot(0,0,0,-0.74,0,0);
    frame =  frame * rot *trans;

    vpQuaternionVector q;
    frame.extract(q);

    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    // @ TODO COGER LOS PUNTOS DESPUES DE LA TRASLACION
    //p.pose.position.x = frame[0][3];
    //p.pose.position.y = frame[1][3];
    //p.pose.position.z = frame[2][3];

    std::ostringstream name, id;
    name << "name: " << i ;
    id << "id: " << i ;
    vispToTF.addTransform(frame, "/stereo_down", name.str(), id.str());

    path_.poses.push_back(p);
  }
}

void BorderDetection::transformPathFrame(std::string new_frame){

  tf::TransformListener listener(ros::Duration(10));
  nav_msgs::Path new_path;
  new_path.header.frame_id=new_frame;
  new_path.header.stamp = ros::Time::now();
  for (int i = 0; i < path_.poses.size(); ++i) {
    geometry_msgs::PoseStamped new_pose(path_.poses[i]);
    bool ok=false;
    while(!ok)
      try{
        ok=true;
        listener.transformPose(new_frame, path_.poses[i], new_pose);
      }
    catch (tf::TransformException  & ex){
      ok=false;
      ROS_ERROR("%s. Retrying... ",ex.what());
    }
    new_pose.header.frame_id=new_frame;
    new_path.poses.push_back(new_pose);
  }
  path_=new_path;

}

void RangeImageBorderDetection::generatePath(){

}

void RangeImageBorderDetection::process(){

  // -----Parameters-----
  float angular_resolution = 0.5f;
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  bool setUnseenToMaxRange = false;

  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (cloud_->sensor_origin_[0],
                                                             cloud_->sensor_origin_[1],
                                                             cloud_->sensor_origin_[2])) *
                                                                 Eigen::Affine3f (cloud_->sensor_orientation_);

  // -----Create RangeImage from the PointCloud-----
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud(*cloud_, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();



  // -----Extract borders-----
  pcl::RangeImageBorderExtractor border_extractor (&range_image);
  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  border_extractor.compute (border_descriptions);

  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
      veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
      shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
      & veil_points = * veil_points_ptr,
      & shadow_points = *shadow_points_ptr;
  for (int y=0; y< (int)range_image.height; ++y)
  {
    for (int x=0; x< (int)range_image.width; ++x)
    {
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
        border_points.points.push_back (range_image.points[y*range_image.width + x]);
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
        veil_points.points.push_back (range_image.points[y*range_image.width + x]);
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
        shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
    }
  }
  border_cloud_with_ranges_=border_points_ptr;

}
