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

  if(argc < 6) {
    std::cerr << "rosrun pm_apps log_results -i objectId  -t modelType [-f fileName]" << std::endl;
    std::cerr << "One of this options is explicitelly needed:" << std::endl;
    std::cerr << "Options: -c To create a new file starting with header (single mode, uses averages)." << std::endl;
    std::cerr << "Options: -a To append to the existing file (single mode, uses averages)." << std::endl;
    std::cerr << "Options: -k Continuous recording." << std::endl << std::endl;
    return 0;
  }

  std::string objectId;
  if (pcl::console::find_argument (argc, argv, "-i") > 0){
    pcl::console::parse_argument (argc, argv, "-i", objectId);
  }else{
    std::cerr << "-i ObjectId is compulsory" << std::endl;
    return 0;
  }

  int mode;
  if (pcl::console::find_argument (argc, argv, "-t") > 0){
    pcl::console::parse_argument (argc, argv, "-t", mode);
  }else{
    std::cerr << "-t modelType (cylinder=1,ransacBox=2,sphere=3,sq=4,pca=5) is compulsory" << std::endl;
    return 0;
  }

  std::string fileName;
  if (pcl::console::find_argument (argc, argv, "-f") > 0){
    pcl::console::parse_argument (argc, argv, "-f", fileName);
  }else{
    fileName = "estimationResults";
  }

  if(pcl::console::find_argument (argc, argv, "-k") > 0){
    AllDataMultiLogger logger(fileName, objectId, nh, mode);
    ros::spin();
    return 0;
  }

  if(pcl::console::find_argument (argc, argv, "-c") > 0){
    AllDataSingleLogger logger(fileName, objectId, nh, false); //append = false
    logger.writeRANSACCylinderHeader();
    logger.writeRow();
    return 0;
  }

  if(pcl::console::find_argument (argc, argv, "-a") > 0){
    AllDataSingleLogger logger(fileName, objectId, nh, true); //append = false
    logger.writeRow();
    return 0;
  }

  std::cerr << "-t modelType (cylinder=1,ransacBox=2,sphere=3,sq=4,pca=5) is compulsory" << std::endl;
  return (0);
}


