/*
 * border_detection.cpp
 *
 *  Created on: 18/03/2014
 *      Author: dfornas
 */
#include<ros/ros.h>
#include <tf/transform_listener.h>

#include <pm_perception/border_detection.h>

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

#include <visp/vpHomogeneousMatrix.h>

//Time measures
#include <ctime>

void ConcaveHullBorderDetection::process()
{
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>), cloud_filtered2(
      new pcl::PointCloud<PointT>), cloud_filtered3(new pcl::PointCloud<PointT>), cloud_projected(
      new pcl::PointCloud<PointT>);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>), cloud_normals2(
      new pcl::PointCloud<pcl::Normal>), cloud_normals3(new pcl::PointCloud<pcl::Normal>);

  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_plane2(
      new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);

  pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>()), cloud_plane(
      new pcl::PointCloud<PointT>()), cloud_plane2(new pcl::PointCloud<PointT>());

  clock_t begin = clock();
  // Build a filter to remove spurious NaNs
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud_);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-10, 10); // @ TODO Adjust to 0, depth + threshold
  pass.filter(*cloud_filtered);

  // Create the segmentation object
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_filtered);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  PlaneSegmentation<PointT> plane_seg(cloud_filtered, cloud_normals);
  plane_seg.setDistanceThreshold(0.105); // Previous value: 12
  plane_seg.setIterations(200);
  plane_seg.apply(cloud_filtered2, cloud_normals2, cloud_plane, coefficients_plane);

  plane_normal_.resize(3);
  plane_normal_[0] = coefficients_plane->values[0];
  plane_normal_[1] = coefficients_plane->values[1];
  plane_normal_[2] = coefficients_plane->values[2];

  CylinderSegmentation<PointT> cyl_seg(cloud_filtered2, cloud_normals2);
  cyl_seg.setDistanceThreshold(0.03);
  cyl_seg.setIterations(5000);
  cyl_seg.setRadiousLimit(0.1);
  cyl_seg.apply(cloud_cylinder, coefficients_cylinder);
  // @ TODO cleaner solution
  cyl_seg.getInliers(inliers_cylinder);
  PCLTools<PointT>::showClouds(cloud_plane, cloud_cylinder, coefficients_plane, coefficients_cylinder);

  clock_t end = clock();
  ROS_DEBUG_STREAM("Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC);

  begin = clock();
  // Project the CYLINDER inliers into the PLANE model
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setIndices(inliers_cylinder);
  proj.setInputCloud(cloud_filtered2);
  proj.setModelCoefficients(coefficients_plane);
  proj.filter(*cloud_projected);

  end = clock();
  ROS_DEBUG_STREAM("Elapsed projection time: " << double(end - begin) / CLOCKS_PER_SEC);
  begin = clock();

  //pcl::PCDWriter writer;
  //writer.write ("projected_cylinder.pcd", *cloud_projected, false);

  // Create a Concave Hull representation of the projected inliers
  pcl::ConcaveHull<PointT> chull;
  chull.setInputCloud(cloud_projected);
  chull.setAlpha(0.1);
  border_cloud_ = boost::shared_ptr<pcl::PointCloud<PointT> >(new pcl::PointCloud<PointT>);
  chull.reconstruct(*border_cloud_);

  end = clock();
  ROS_DEBUG_STREAM("Elapsed concave hull time: " << double(end - begin) / CLOCKS_PER_SEC);

  //writer.write ("chull.pcd", *border_cloud_, false);
  ROS_INFO_STREAM("Hull size: "<< border_cloud_->points.size());

  end = clock();
  ROS_DEBUG_STREAM("Elapsed seg. time: " << double(end - begin) / CLOCKS_PER_SEC);
}

void ConcaveHullBorderDetection::generatePath()
{
  nav_msgs::Path empty_path;
  path_ = empty_path;
  path_.header.frame_id = "/stereo_down";
  path_.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped mass_center;
  // @ WARNING/TODO The first 10 points are skipped because were wrong. Improve.
  int starting_point = 10;

  for (int i = starting_point; i < border_cloud_->points.size(); ++i)
  {
    //Move path upwards.
    border_cloud_->points[i].x += -plane_normal_[0] * 0.15;
    border_cloud_->points[i].y += -plane_normal_[1] * 0.15;
    border_cloud_->points[i].z += -plane_normal_[2] * 0.15;

    mass_center.pose.position.x += border_cloud_->points[i].x;
    mass_center.pose.position.y += border_cloud_->points[i].y;
    mass_center.pose.position.z += border_cloud_->points[i].z;
  }
  mass_center.pose.position.x /= border_cloud_->points.size() - starting_point;
  mass_center.pose.position.y /= border_cloud_->points.size() - starting_point;
  mass_center.pose.position.z /= border_cloud_->points.size() - starting_point;

  std::vector<vpColVector> point_list;
  // Subsampling
  vpColVector previous_point(3);
  previous_point[0] = border_cloud_->points[starting_point].x;
  previous_point[1] = border_cloud_->points[starting_point].y;
  previous_point[2] = border_cloud_->points[starting_point].z;
  point_list.push_back(previous_point);

  for (int i = starting_point + 1; i < border_cloud_->points.size(); ++i)
  {
    vpColVector p(3), vect(3);
    p[0] = border_cloud_->points[i].x;
    p[1] = border_cloud_->points[i].y;
    p[2] = border_cloud_->points[i].z;

    vect[0] = p[0] - previous_point[0];
    vect[1] = p[1] - previous_point[1];
    vect[2] = p[2] - previous_point[2];

    double distance = sqrt((vect[0]) * (vect[0]) + (vect[1]) * (vect[1]) + (vect[2]) * (vect[2]));
    ROS_INFO_STREAM("Distance: "<<distance);
    if (distance < subsampling_distance_)
      point_list.push_back(p);
    else
    {
      int num_points = (int)floor(distance / subsampling_distance_);
      ROS_INFO_STREAM(i<<"point. Distance: "<< distance << "number of points: " << num_points);
      for (int j = 1; j < num_points + 1; j++)
      {

        p[0] = previous_point[0] + vect[0] * (j * subsampling_distance_ / distance);
        p[1] = previous_point[1] + vect[1] * (j * subsampling_distance_ / distance);
        p[2] = previous_point[2] + vect[2] * (j * subsampling_distance_ / distance);
        point_list.push_back(p);
      }
      p[0] = border_cloud_->points[i].x;
      p[1] = border_cloud_->points[i].y;
      p[2] = border_cloud_->points[i].z;
      point_list.push_back(p);
    }
    previous_point = p;
  }

  for (int i = 0; i < point_list.size(); ++i)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "/stereo_down";

    //Orientation
    double xdiff = mass_center.pose.position.x - point_list[i][0];
    double ydiff = mass_center.pose.position.y - point_list[i][1];
    double zdiff = mass_center.pose.position.z - point_list[i][2];

    vpColVector zaxis(3);
    zaxis[0] = xdiff;
    zaxis[1] = ydiff;
    zaxis[2] = zdiff;
    zaxis = zaxis.normalize();

    vpColVector xaxis = vpColVector::crossProd(zaxis, plane_normal_).normalize();
    vpColVector yaxis = vpColVector::crossProd(zaxis, xaxis).normalize();

    vpHomogeneousMatrix frame;
    frame[0][0] = xaxis[0]; frame[0][1] = yaxis[0]; frame[0][2] = zaxis[0]; frame[0][3] = point_list[i][0];
    frame[1][0] = xaxis[1]; frame[1][1] = yaxis[1]; frame[1][2] = zaxis[1]; frame[1][3] = point_list[i][1];
    frame[2][0] = xaxis[2]; frame[2][1] = yaxis[2]; frame[2][2] = zaxis[2]; frame[2][3] = point_list[i][2];
    frame[3][0] = 0;        frame[3][1] = 0;        frame[3][2] = 0;        frame[3][3] = 1;

    //Apply rotation and traslation to position the grasp points.
    vpHomogeneousMatrix trans0(0, 0.05, 0, 0, 0, 0);
    vpHomogeneousMatrix trans(0, 0, -0.05, 0, 0, 0);
    vpHomogeneousMatrix rot(0, 0, 0, 0.74, 0, 0);
    frame = frame * trans0 * rot * trans;

    vpQuaternionVector q;
    frame.extract(q);

    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    p.pose.position.x = frame[0][3];
    p.pose.position.y = frame[1][3];
    p.pose.position.z = frame[2][3];

    std::ostringstream name, id;
    name << "name: " << i;
    id << "id: " << i;
    vispToTF.addTransform(frame, "/stereo_down", name.str(), id.str());
    path_.poses.push_back(p);
  }
}

void BorderDetection::transformPathFrame(std::string new_frame)
{
  tf::TransformListener listener(ros::Duration(10));
  nav_msgs::Path new_path;
  new_path.header.frame_id = new_frame;
  new_path.header.stamp = ros::Time::now();
  for (int i = 0; i < path_.poses.size(); ++i)
  {
    geometry_msgs::PoseStamped new_pose(path_.poses[i]);
    bool ok = false;
    while (!ok)
      try
      {
        ok = true;
        listener.transformPose(new_frame, path_.poses[i], new_pose);
      }
      catch (tf::TransformException & ex)
      {
        ok = false;
        ROS_DEBUG("%s. Retrying... ", ex.what());
      }
    new_pose.header.frame_id = new_frame;
    new_path.poses.push_back(new_pose);
  }
  path_ = new_path;
}

void BorderDetection::savePathToFile()
{
  //Export data in Quaternions and Euler angles.
  std::stringstream path_string_euler, path_string_quat;

  for (int i = 0; i < path_.poses.size(); ++i)
  {
    geometry_msgs::PoseStamped p(path_.poses[i]);
    path_string_quat << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " "
        << p.pose.orientation.x << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " "
        << p.pose.orientation.w << "\n";
    vpRxyzVector euler_rot(
        vpRotationMatrix(
            vpQuaternionVector(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z,
                               p.pose.orientation.w)));
    path_string_euler << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " "
        << euler_rot[0] << " " << euler_rot[1] << " " << euler_rot[2] << "\n";
  }

  std::ofstream f_quat("path_quat.txt");
  f_quat << path_string_quat.rdbuf();

  std::ofstream f_euler("path_euler.txt");
  f_euler << path_string_euler.rdbuf();
}

void RangeImageBorderDetection::generatePath()
{
  // @ TODO Implement.

}

void RangeImageBorderDetection::process()
{
  // -----Parameters-----
  float angular_resolution = 0.5f;
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  bool setUnseenToMaxRange = false;

  Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
  scene_sensor_pose = Eigen::Affine3f(
      Eigen::Translation3f(cloud_->sensor_origin_[0], cloud_->sensor_origin_[1], cloud_->sensor_origin_[2]))
      * Eigen::Affine3f(cloud_->sensor_orientation_);

  // -----Create RangeImage from the PointCloud-----
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud(*cloud_, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange();

  // -----Extract borders-----
  pcl::RangeImageBorderExtractor border_extractor(&range_image);
  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  border_extractor.compute(border_descriptions);

  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
      veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>), shadow_points_ptr(
          new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr, &veil_points = *veil_points_ptr,
      &shadow_points = *shadow_points_ptr;
  for (int y = 0; y < (int)range_image.height; ++y)
  {
    for (int x = 0; x < (int)range_image.width; ++x)
    {
      if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
        border_points.points.push_back(range_image.points[y * range_image.width + x]);
      if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
        veil_points.points.push_back(range_image.points[y * range_image.width + x]);
      if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
        shadow_points.points.push_back(range_image.points[y * range_image.width + x]);
    }
  }
  border_cloud_with_ranges_ = border_points_ptr;
}
