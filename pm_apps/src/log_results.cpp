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

  std::ostringstream ssAll, ssAvg, ssStdDev;
  switch(mode) {
    case 1:
      ssAll << objectId << "_ransacCylinder" ;
      ssAvg << "avg" << "_ransacCylinder" ;
      ssStdDev << "stdDev" << "_ransacCylinder" ;
      break;
    case 2:
      ssAll << objectId << "_ransacBox" ;
      ssAvg << "avg" << "_ransacBox" ;
      ssStdDev << "stdDev" << "_ransacBox" ;
      break;
    case 3:
      ssAll << objectId << "_ransacSphere" ;
      ssAvg << "avg" << "_ransacSphere" ;
      ssStdDev << "stdDev" << "_ransacSphere" ;
      break;
    case 4:
      ssAll << objectId << "_sq" ;
      ssAvg << "avg" << "_sq" ;
      ssStdDev << "stdDev" << "_sq" ;
      break;
    case 5:
      ssAll << objectId << "_pca" ;
      ssAvg << "avg" << "_pca" ;
      ssStdDev << "stdDev" << "_pca" ;
      break;
    default:
      break;
  }

  std::string fileName, avgName, stdDevName;
  if (pcl::console::find_argument (argc, argv, "-f") > 0){
    pcl::console::parse_argument (argc, argv, "-f", fileName);
  }else{
    fileName = ssAll.str();
    avgName = ssAvg.str();
    stdDevName = ssStdDev.str();
  }

  boost::scoped_ptr<AllDataSingleLogger> avgLoader, stdDevLoader;
  if(pcl::console::find_argument (argc, argv, "-c") > 0){
    avgLoader.reset(new AllDataSingleLogger(avgName, objectId, nh, false)); //append = false
    stdDevLoader.reset(new AllDataSingleLogger(stdDevName, objectId, nh, false, true)); //append = false
    switch(mode) {
      case 1:
        avgLoader->writeRANSACCylinderHeader();
        stdDevLoader->writeRANSACCylinderHeader();
        break;
      case 2:
        avgLoader->writeRANSACBoxHeader();
        stdDevLoader->writeRANSACBoxHeader();
        break;
      case 3:
        avgLoader->writeRANSACSphereHeader();
        stdDevLoader->writeRANSACSphereHeader();
        break;
      case 4:
        avgLoader->writeSQHeader();
        stdDevLoader->writeSQHeader();
        break;
      case 5:
        avgLoader->writePCAHeader();
        stdDevLoader->writePCAHeader();
        break;
      default:
        break;
    }
  }else{
    avgLoader.reset(new AllDataSingleLogger(avgName, objectId, nh, true)); //append = true
    stdDevLoader.reset(new AllDataSingleLogger(avgName, objectId, nh, true, true)); //append = true
  }
  AllDataMultiLogger logger(fileName, objectId, nh, mode);
  ros::spin();
  avgLoader->writeRow();
  stdDevLoader->writeRowStdDev();
  avgLoader.reset();
  stdDevLoader.reset();
  std::cerr << "Averages also written" << std::endl;

  return (0);
}


