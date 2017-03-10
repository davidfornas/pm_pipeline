/*
 * PCGraspPlanning.h Complete point cloud grasp planning from
 * MAR (https://github.com/penalvea/irs-ros-pkg). It has been
 * divided in modular components but this is the full version.
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef PMGRASPPLANNINGSPLIT_H_
#define PMGRASPPLANNINGSPLIT_H_

#include <pm_tools/tf_tools.h>
#include <pm_tools/marker_tools.h>

#include <pcl/io/pcd_io.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpPoint.h>
#include <visp/vpHomogeneousMatrix.h>

#include <tf/transform_datatypes.h>
#include <list>


/** Grasp planning from object pose */
class PMGraspPlanningSplit {

  //Grasping params (to allow different grasps and radious (for grasp penetration)).
  double angle_, rad_, along_;
  //Punto central del cilindro y la direccion.
  ///// PointT axis_point_g;
  tf::Vector3 normal_g;
  FrameToTF vispToTF;

  std::string camera_frame_name, topic_name;

public:

  vpHomogeneousMatrix cMg, cMo; ///< Grasp frame with respect to the camera after planning
  double radious, height;

  //With integuers to use trackbars
  int iangle, irad, ialong;

  /** Constructor.
   * @params: cloud
   * */
  PMGraspPlanningSplit( std::string object_topic_name, std::string frame_id = "sense3d" ){
    angle_=0;iangle=0;
    rad_=0;irad=0;
    along_=0;ialong=0;

    camera_frame_name = frame_id; //cloud->header.frame_id;
    topic_name = object_topic_name;
  }


  /** Main function where segmentation is done */
  void perceive();


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

  ~PMGraspPlanningSplit() {}

private:

  /** Configure the camera based in int slider parameters */
  void intToConfig();

};
#endif /* PMGRASPPLANNINGSPLIT_H_ */
