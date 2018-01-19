/**
 * Registration of clouds using ARMarker Poses
 *  Created on: 21/11/2017
 *      Author: dfornas
 */
#ifndef MARKERREGISTRATION_H_
#define MARKERREGISTRATION_H_

#include <pm_tools/tf_tools.h>
#include <pm_tools/pcl_tools.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

typedef message_filters::Subscriber<geometry_msgs::PoseStamped> PoseSub;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> CloudSub;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> CloudPoseSync;

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

typedef pcl::PointNormal NormalT;
typedef typename pcl::PointCloud<NormalT> CloudWithNormals;
typedef typename pcl::PointCloud<NormalT>::Ptr CloudWithNormalsPtr;

// Define a new point representation for < x, y, z, curvature >
class PointRepXYZCurv : public pcl::PointRepresentation <NormalT>
{
  using pcl::PointRepresentation<NormalT>::nr_dimensions_;

public:

  PointRepXYZCurv ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const NormalT &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


class MarkerRegistration{

  //TF only needed as alternative to Pose input. Not good for TF from BAGFILE.
  tf::TransformListener *listener_;
  tf::TransformBroadcaster *broadcaster_;

  //Subscribers and sync. THE TWO TOPICS ARE SYNCED.
  PoseSub *pose_sub_;
  CloudSub *cloud_sub_;
  message_filters::Synchronizer<CloudPoseSync> *sync;

  geometry_msgs::PoseStamped current_pose_, prev_pose_;

  CloudPtr current_cloud_, prev_cloud_, map_cloud_, full_res_map_cloud_;
  pcl::visualization::PCLVisualizer *p;
  // 4 viewports. 1)Originals. 2)Marker correction 3)Marker+ICP 4)Map
  int vp_1, vp_2, vp_3, vp_4;

  ros::Publisher map_pub_;
  bool first_cloud_;

  Eigen::Matrix4f globalTransform;

public:

  MarkerRegistration(ros::NodeHandle &nh, int & argc, char * argv[]);


  void run();

  // Obtain cloud and pose from marker in sync. Trigger align.
  // @TODO Optimize Cloud conversions and formats
  void detectionCallback(const geometry_msgs::PoseStamped::ConstPtr& pose, const sensor_msgs::PointCloud2::ConstPtr& cloud);

  // Convert transform matrix between VISP and Eigen
  Eigen::Matrix4f vpHomogeneousMatrixToEigen4f( vpHomogeneousMatrix in );

  //Display source and target on the first viewport of the visualizer
  void showOriginalDifference(const CloudPtr cloud_target, const CloudPtr cloud_source);

  //Display source and target on the second viewport of the visualizer
  void showMarkerAlignment(const CloudPtr cloud_target, const CloudPtr cloud_source);

  //Display source and target aligned with ICP
  void showICPRefinement(const CloudPtr cloud_target, const CloudPtr cloud_source, double score);

  //Display map
  void showMap(const CloudPtr map, bool pause = true);

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
  void pairAlign (const CloudPtr cloud_src, const CloudPtr cloud_tgt, CloudPtr output, Eigen::Matrix4f &final_transform, bool downsample = false);
};


#endif
