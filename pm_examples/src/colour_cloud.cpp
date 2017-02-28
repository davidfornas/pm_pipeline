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

/*
  Interpolate interpolates a value
  */
double interpolate(double val, double y0, double x0, double y1, double x1){
    return (val - x0)*(y1-y0)/(x1-x0) + y0;
}


double base(double val){
    if (val <= -0.75) return 0;
    else if (val <= -0.25) return interpolate(val,0,-0.75,1,-0.25);
    else if (val <= 0.25) return 1;
    else if (val <= 0.75) return interpolate(val,1.0,0.25,0.0,0.75);
    else return 0;
}

/*
  ColorCloudDepth returns an RGB cloud coloured with depth with matlab's jet color scale
  */

void colourCloudDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double min_z,double max_z){

    // Get Max and Min
    double mx(-10000000),mn(10000000000),mean;
    bool look_for_min_z(true);
    bool look_for_max_z(true);

    if (min_z != max_z){
        if (min_z != 0){
            mn = min_z;
            look_for_min_z = false;
        }
        if (max_z != 0){
            mx = max_z;
            look_for_max_z = false;
        }
    }

    for (int k=0;k<cloudIn->size();k++){
        if (mx<cloudIn->points[k].z && look_for_max_z) mx = cloudIn->points[k].z;
        if (mn>cloudIn->points[k].z && look_for_min_z) mn = cloudIn->points[k].z;
    }
    mean = (mx+mn)/2;

    // Compute Color
    uint8_t r, g, b;
    for (int k=0;k<cloudIn->size();k++){
        pcl::PointXYZRGB point;
        point.x = cloudIn->points[k].x;
        point.y = cloudIn->points[k].y;
        point.z = cloudIn->points[k].z;
        double z = (point.z - mn)/(mx-mn) * 2 - 1;
        r = (int) (base(z - 0.5) * 255);
        g = (int) (base(z) * 255);
        b = (int) (base(z + 0.5) * 255);

        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                      static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

        point.rgb = *reinterpret_cast<float*>(&rgb);
        cloud->push_back(point);
    }
}

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
  //PCLTools<pcl::PointXYZRGB>::applyZAxisPassthrough(colour, 0, 1.95);

  colourCloudDepth( original, colour, 0, 0 );

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

  if( argc != 3 ){
    ROS_ERROR_STREAM("rosrun pm_examples colour_cloud /sense3d/scan /sense3d/scan/color");
  }
  std::string input_cloud_topic(argv[1]), output_cloud_topic(argv[2]);

  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>( output_cloud_topic, 1000); //"/sense3d/scan/color"
  ros::Subscriber original_sub	= nh.subscribe<sensor_msgs::PointCloud2>( input_cloud_topic, 1, cloudCallback); //"/sense3d/scan"

  ros::spin();
  return (0);
}



