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

  bool append = false;
  if(pcl::console::find_argument (argc, argv, "-c") > 0){
    AllDataLogger logger(fileName, objectId, nh, false, true); //append = false, use average = true;
    logger.writeRANSACCylinderHeader();
    logger.writeRow();
  }else{
    AllDataLogger logger(fileName, objectId, nh, true, true); //append = false, use average = true;
    logger.writeRow();
  }

  return (0);
}


