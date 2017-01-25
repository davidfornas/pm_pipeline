#include <mar_perception/PCAutonomousGraspPlanning.h>
#include <unistd.h>

#include <pm_tools/pcl_tools.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

int main(int argc, char **argv){


  ROS_INFO("Object detector called. Starting detection.");
  //subscribe to pointcloud...
  Cloud::Ptr point_cloud_ptr (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromPCD(point_cloud_ptr, std::string(argv[1]) + std::string(".pcd")); //Load from PCDReader or from topic

  PCAutonomousGraspPlanning planner(0, 0, 0, true, point_cloud_ptr);
  planner.setPlaneSegmentationParams(0.09, 200);
  planner.setCylinderSegmentationParams(0.06, 10000, 0.1);
  planner.perceive();
  ROS_INFO("Perception finished.");
  vpHomogeneousMatrix cMo = planner.get_cMg();
  //Result
  /*
  geometry_msgs::Pose p;
  p.position.x=cMo[0][3];
  p.position.y=cMo[1][3];
  p.position.z=cMo[2][3];
  vpQuaternionVector q; cMo.extract(q);
  p.orientation.x=q.x();
  p.orientation.y=q.y();
  p.orientation.z=q.z();
  p.orientation.w=q.w();
  res.cMo=p;
  res.success=true;
  */
return 0;


}
