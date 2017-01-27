/**
 * 3D interface to plan a grasp using a point cloud.
 *
 *  Created on: 24/01/2016
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_manipulation/pm_grasp_planning.h>
#include <pm_tools/pcl_tools.h>

#include <pm_tools/visp_tools.h>

//Quick GUI form OpoenCV, should change it to Qt.
#include <vector>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

vpHomogeneousMatrix worldToMarkerInitPose;

//Class that publishes eef position from marker feedback to UWSim
class EefFollower
{
private:
  ros::Publisher pos_pub;
public:
  EefFollower(std::string topic, ros::NodeHandle &nh )
  {
    pos_pub = nh.advertise<geometry_msgs::Pose>(topic, 1);
  }
  //Interactive marker feedback class (calls the publisher)
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    /*std::cout<< feedback->marker_name << " is now at "<< feedback->pose.position.x << ", " << feedback->pose.position.y
             << ", " << feedback->pose.position.z << std::endl;
    std::cout<< feedback->marker_name << " orientation is " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y
             << ", " << feedback->pose.orientation.z  << ", " << feedback->pose.orientation.w <<std::endl;*/
    vpHomogeneousMatrix markerInitPoseToCurrentMarkerPose;
    markerInitPoseToCurrentMarkerPose = VispTools::vispHomogFromGeometryPose(feedback->pose);

    pos_pub.publish(VispTools::geometryPoseFromVispHomog( worldToMarkerInitPose * markerInitPoseToCurrentMarkerPose ) );
    ros::spinOnce();
  }
};

/** Plans a grasp on a point cloud and visualizes it using UWSim externally.
 * Expects the following params to be set in the ROS parameter server: @TODO CHANGE
 * input_basename: The full path to the input files without PCD extension, ie. /tmp/scan_2
 * resources_data_path: The path the folder with the models for UWSim
 * eMh: a six-element vector representing the hand frame wrt the end-effector frame. [x y z roll pitch yaw] format; now UNUSED
 */
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "grasp_vis");
  ROS_INFO("Hello world!");

  ros::NodeHandle nh;

  EefFollower follower("/gripperPose", nh);//(std::string)argv[1]);


  ros::Publisher position_pub;
  position_pub=nh.advertise<geometry_msgs::Pose>("/gripperPose",1);


  //Variables de configuración: ángulo de agarre, distancias...
  double angle, rad, along;
  bool alignedGrasp;
  std::string input_basename("output");
  nh.getParam("input_basename", input_basename);
  nh.param("alignedGrasp", alignedGrasp, false);
  nh.param("angle", angle, 0.0);
  nh.param("rad", rad, 0.0);
  nh.param("along", along, 0.0);

  //Point Cloud load
  Cloud::Ptr cloud (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromPCD(cloud, input_basename); //Load from PCDReader or from topic
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

  //Init planner
  PMGraspPlanning planner(cloud);
  planner.perceive();

  std::cout << "Starting visualization in UWSim" << std::endl; //Not anymore
  //PONER EL GRIPPER EN LA POSICIÓN INICIAL  cMg
  vpHomogeneousMatrix cMg = planner.get_cMg();

  bool Marker = false, MarkerCreated = false;


  // create an interactive marker server on the topic namespace uwsim_marker
  interactive_markers::InteractiveMarkerServer server("uwsim_marker");

  while (ros::ok())/// && !view.getViewer()->done())
  {
    //Interface
    cv::namedWindow("Grasp configuration", CV_WINDOW_NORMAL);
    cv::createTrackbar("Radius", "Grasp configuration", &(planner.irad), 100);
    cv::createTrackbar("Angle", "Grasp configuration", &(planner.iangle), 360);
    cv::createTrackbar("Distance", "Grasp configuration", &(planner.ialong), 100);
    cv::createTrackbar("Aligned grasp?", "Grasp configuration", &(planner.ialigned_grasp), 1);

    //Compute new grasp frame with the slides
    planner.recalculate_cMg();
    cMg = planner.get_cMg();

    planner.ialigned_grasp==1 ? Marker = false : Marker = true;

    geometry_msgs::Pose pose = VispTools::geometryPoseFromVispHomog(cMg);

    if (!Marker){
      if(MarkerCreated){
        MarkerCreated = false;
        server.erase("eefMarker");
        server.applyChanges();
      }
      //Publish Pose on UWSim
      position_pub.publish(pose);
    }else{
      if(!MarkerCreated){
        MarkerCreated = true;
        worldToMarkerInitPose = cMg;

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/world";

    int_marker.name = "eefMarker";
    //text orientation depends on world offset, so it's commented
    //int_marker.description = "End Effector marker";
    int_marker.scale = 0.2;

    position_pub.publish(pose);
    int_marker.pose.position.x = pose.position.x;
    int_marker.pose.position.y = pose.position.y;
    int_marker.pose.position.z = pose.position.z;
    int_marker.pose.orientation.x = pose.orientation.x;
    int_marker.pose.orientation.y = pose.orientation.y;
    int_marker.pose.orientation.z = pose.orientation.z;
    int_marker.pose.orientation.w = pose.orientation.w;

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::SPHERE;
    box_marker.scale.x = 0.01;
    box_marker.scale.y = 0.01;
    box_marker.scale.z = 0.01;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;
    // create a non-interactive control to hold the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = false;
    box_control.markers.push_back( box_marker );
    // add the 6DOF control to the interactive marker
    int_marker.controls.push_back( box_control );

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker,boost::bind(&EefFollower::processFeedback, &follower, _1) );
    // 'commit' changes and send to all clients
    server.applyChanges();

      }
    }

    ros::spinOnce();

    cv::waitKey(5);
  }
  return 0;
}
