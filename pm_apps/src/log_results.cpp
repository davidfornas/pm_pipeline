/** 

 *  Created on: 14/08/2018
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_tools/logger.h>
#include <pcl/console/parse.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "log_results");
  ros::NodeHandle nh;

  if(argc < 2){
	  std::cerr << "rosrun pm_apps log_results -i objectId -f fileName" << std::endl;
      std::cerr << "Options: -c To create file astarting with header." << std::endl << std::endl;
      std::cerr << "Options: -k Continuous recording." << std::endl << std::endl;
	  return 0;
  }

  std::string objectId;
  if (pcl::console::find_argument (argc, argv, "-i") > 0){
    pcl::console::parse_argument (argc, argv, "-i", objectId);
  }else{
    std::cerr << "rosrun pm_apps log_results -i ObjectId !!" << std::endl;
  }

  std::string fileName;
  if (pcl::console::find_argument (argc, argv, "-f") > 0){
    pcl::console::parse_argument (argc, argv, "-f", fileName);
  }else{
    fileName = "estimationResults";
  }

  if(pcl::console::find_argument (argc, argv, "-k") > 0){
    AllDataMultiLogger logger(fileName, objectId, nh, 1); //1 = cylinder
    ros::spin();
    return 0;
  }

  bool append = false;
  if(pcl::console::find_argument (argc, argv, "-c") > 0){
    AllDataSingleLogger logger(fileName, objectId, nh, false); //append = false
    logger.writeRANSACCylinderHeader();
    logger.writeRow();
  }else{
    AllDataSingleLogger logger(fileName, objectId, nh, true); //append = false
    logger.writeRow();
  }

  return (0);
}


