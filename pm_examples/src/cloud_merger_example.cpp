/** 
 * This program test the cloud merging utility.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>

#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_merger_example");
  ros::NodeHandle nh;
  CloudMerge merger;


  int clouds_number;
  double depth=2, near=0;
  std::string input_basename("cloud");
  nh.getParam("input_basename", input_basename);
  nh.getParam("clouds_number", clouds_number);
  nh.getParam("depth", depth);
  nh.getParam("near", near);
    if (clouds_number < 2)
    return -1;
  //Point Cloud load
  std::string point_cloud_file(input_basename + std::string("1.pcd"));
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>), cloud2(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read(point_cloud_file, *cloud);
  //PCLTools::applyZAxisPassthrough(cloud_in, cloud_out, depth, near);
  //std::cout << "First PointCloud has: " << cloud->points.size () << " data points. Number of NaNs: " << nanCount(cloud) << std::endl;
  std::cout << PCLTools::nanCount(cloud) << std::endl;

  int s = cloud->points.size();
  for (int i = 2; i <= clouds_number; i++)
  {

    std::ostringstream s;
    s << i;
    const std::string i_as_string(s.str());
    std::string point_cloud_file2(input_basename + i_as_string + std::string(".pcd"));
    // Read in the cloud data
    pcl::PCDReader reader2;
    reader2.read(point_cloud_file2, *cloud2);
    //std::cout << "PointCloud to copy has: " << cloud2->points.size () << " data points. Num. of NaNs: " << nanCount(cloud2) << std::endl; //*
    //(*cloud)+=(*cloud2);
    ////ct.prefilter(cloud2, depth, near);
    merger.nanAwareOrganizedConcatenateMean(cloud, cloud2);
    //std::cout << "First PointCloud current NaN number:" << nanCount(cloud) << std::endl;
    std::cout << PCLTools::nanCount(cloud) << std::endl;
  }

  /*  Computing deviation
  double xvar_mean=0, xvar_var=0, yvar_mean=0, yvar_var=0, zvar_mean=0, zvar_var=0;
  double count=0;
  for (size_t i = 0; i < s; ++i)
  {
    if (ct.coeffs[i] != 0)
    {
      ct.xvar[i] = sqrt(abs(ct.xvar[i] / ct.coeffs[i] - cloud->points[i].x));
      ct.yvar[i] = sqrt(abs(ct.yvar[i] / ct.coeffs[i] - cloud->points[i].y));
      ct.zvar[i] = sqrt(abs(ct.zvar[i] / ct.coeffs[i] - cloud->points[i].z));
      std::cout << "VAR: " << ct.xvar[i] << " " << ct.yvar[i] << " " << ct.zvar[i] << " " << ct.coeffs[i] << std::endl;
      std::cout << "P: " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << std::endl;

      xvar_mean+=ct.xvar[i];
      yvar_mean+=ct.yvar[i];
      zvar_mean+=ct.zvar[i];

      xvar_var+=ct.xvar[i]*ct.xvar[i];
      yvar_var+=ct.yvar[i]*ct.yvar[i];
      zvar_var+=ct.zvar[i]*ct.zvar[i];

      count++;
    }
  }

  xvar_mean/=count;
  yvar_mean/=count;
  zvar_mean/=count;

  xvar_var=sqrt(abs(xvar_var/count-xvar_mean));
  yvar_var=sqrt(abs(yvar_var/count-yvar_mean));
  zvar_var=sqrt(abs(zvar_var/count-zvar_mean));
  std::cout << "Mean deviation-> x: " << std::setprecision(10) << xvar_var << " y: " << std::setprecision(10) << yvar_var << " z: " << std::setprecision(10) << zvar_var << std::endl;
  std::cout << "Mean mean-> x: " << std::setprecision(10) << xvar_mean << " y: " << std::setprecision(10) << yvar_mean << " z: " << std::setprecision(10) << zvar_mean << std::endl;
  */

  //std::cerr << "Cloud sum: " << coeffs << std::endl;
  // Cloud viewer object. You can set the window title.
  pcl::visualization::CloudViewer viewer("Accumulated cloud");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped())
  {
    // Do nothing but wait.
  }

  pcl::PCDWriter writer;
  writer.write(input_basename + std::string("_out.pcd"), *cloud, false);

  return (0);
}

