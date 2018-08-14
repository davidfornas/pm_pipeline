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
	  std::cerr << "rosrun pm_apps log_results -i ObjectId" << std::endl;
      std::cerr << "Options: -a To append without adding header." << std::endl << std::endl;
	  return 0;
  }

  std::string objectId;
  if (pcl::console::find_argument (argc, argv, "-i") > 0){
    pcl::console::parse_argument (argc, argv, "-i", objectId);
  }else{
    std::cerr << "rosrun pm_apps log_results -i ObjectId !!" << std::endl;
  }

  bool append = false;
  if(pcl::console::find_argument (argc, argv, "-a") > 0) append = true;

  AllDataLogger logger("ransacEstimationData", objectId, nh, true);

  logger.writeRANSACCylinderHeader();
  logger.writeRow();

  return (0);
}


