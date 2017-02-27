/**
 * This program is used to filter and modify point clouds easily.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <iostream>
#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>

#include <pcl/console/parse.h>


ros::Publisher cloud_pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  ros::NodeHandle nh;


  pcl::PointCloud<pcl::PointXYZ>::Ptr original(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colour(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*msg, pcl_pc);
  //PCL Generic cloud to PointT strong type.
  pcl::fromPCLPointCloud2(pcl_pc, *original);

  pcl::copyPointCloud (*original, *colour);

  //Filter and colors for 0-2m. range...
  PCLTools<pcl::PointXYZRGB>::applyZAxisPassthrough(colour, 0, 1.9);

  for (size_t i=0;i<colour->points.size();i++)
  {
    float_t rr = 0, gg = 0, bb = 0;

    //MULTICOLOR
    if(colour->points[i].z<=0.7){
      rr = colour->points[i].z / 0.7 * 255;
    }else if(colour->points[i].z>0.7 && colour->points[i].z<=1.4){
      gg = (colour->points[i].z - 0.7) / 0.7 * 255;
    }else{
      bb = (colour->points[i].z - 1.4) / 0.6 * 255;
    }

    //SINGLE COLOR
    /*
       rr = colour->points[i].z / 2 * 255;
       gg=0;bb=0;
      */

    uint8_t r = (uint8_t) floor(rr), g = (uint8_t) floor(gg), b = (uint8_t) floor(bb);
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    colour->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }

  sensor_msgs::PointCloud2 message;
  //PointT strong type to PCL Generic cloud.
  pcl::toPCLPointCloud2(*colour, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, message);
  cloud_pub.publish(message);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "colour_cloud");
  ros::NodeHandle nh;

  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/sense3d/scan/color", 1000);
  ros::Subscriber original_sub	= nh.subscribe<sensor_msgs::PointCloud2>("/sense3d/scan", 1, cloudCallback);



  ros::spin();
  return (0);
}



