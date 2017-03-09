/*
 * PCGraspPlanning.h Complete point cloud grasp planning from
 * MAR (https://github.com/penalvea/irs-ros-pkg). It has been
 * divided in modular components but this is the full version.
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef PMGRASPPLANNING_H_
#define PMGRASPPLANNING_H_

#include <pm_tools/tf_tools.h>
#include <pm_tools/marker_tools.h>

#include <pcl/io/pcd_io.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpPoint.h>
#include <visp/vpHomogeneousMatrix.h>

#include <tf/transform_datatypes.h>
#include <list>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PointTPtr;

/** Point cloud grasp planning full */
class PMGraspPlanning {

  PointTPtr cloud_;
  //Grasping params (to allow different grasps and radious (for grasp penetration)).
  double angle_, rad_, along_;
  //Punto central del cilindro y la direccion.
  PointT axis_point_g; tf::Vector3 normal_g;
  FrameToTF vispToTF;
  double plane_distance_threshold_, cylinder_distance_threshold_, radious_limit_;
  int plane_iterations_, cylinder_iterations_;

  std::string camera_frame_name;

public:

  vpHomogeneousMatrix cMg, cMo; ///< Grasp frame with respect to the camera after planning
  double radious, height;

  //With integuers to use trackbars
  int iangle, irad, ialong;

  std::list<vpHomogeneousMatrix> cMg_list;


  /** Constructor.
   * @params: cloud
   * */
  PMGraspPlanning(PointTPtr cloud){
    angle_=0;iangle=0;
    rad_=0;irad=0;
    along_=0;ialong=0;

    setPlaneSegmentationParams();
    setCylinderSegmentationParams();

    cloud_ = cloud;
    camera_frame_name = cloud->header.frame_id;

  }

  /** @ TODO To implement. Place holder. */
  void proccessScene();

  /** @ TODO To implement. Place holder. */
  void getGraspHypothesis();

  /** Generate not only a grasp but a list */
  void generateGraspList();

  /** Evaluate and rank grasp list*/
  void filterGraspList();

  /** Main function where segmentation is done */
  void perceive();

  /** Set plane segmentation parameters: distance to the inliers to the plane
   * and number of iterations.
   */
  void setPlaneSegmentationParams(double distanceThreshold = 0.03, int iterations = 100){
    plane_distance_threshold_=distanceThreshold;
    plane_iterations_=iterations;
  }

  /** Set cylinder segmentation parameters: distance to the inliers to the plane,
   * number of iterations and radious limit.
   */
  void setCylinderSegmentationParams(double distanceThreshold = 0.05,int iterations = 20000, double rlimit = 0.1){
    cylinder_distance_threshold_=distanceThreshold;
    cylinder_iterations_=iterations;
    radious_limit_=rlimit;
  }

  void setCamerFrameName( std::string name){
    camera_frame_name = name;
  }

  /** Get the grasp frame with respect to the camera frame */
  vpHomogeneousMatrix get_cMg() {return cMg;}

  void getBestParams( double & angle, double & rad, double & along );
  void computeMatrix( double angle, double rad, double along );

  /** Recalculate cMg with current parameters */
  void recalculate_cMg();

  /** Get the grasp frame pose with respect to an arbitrary frame 'b', given as input relative to the camera frame
   * @param bMc an homogeneous matrix with the camera frame given wrt the frame 'b'
   * @returns an homogeneous matrix with the grasp frame given wrt the frame 'b'
   */
  vpHomogeneousMatrix get_bMg(vpHomogeneousMatrix bMc) {return bMc*cMg;}

  ~PMGraspPlanning() {}

private:

  /** Comparer used in the sort function */
  bool sortFunction(const PointT& d1, const PointT& d2);

  /** Get the grasp frame with respect to the camera frame */
  void getMinMax3DAlongAxis(const pcl::PointCloud<PointT>::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT axis_point, tf::Vector3 * normal, double outlier_percentage = 0.1);

  /** Configure the camera based in int slider parameters */
  void intToConfig();

};
#endif /* PMGRASPPLANNING_H_ */
