/*
 * PCGraspPlanning.h
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef PMGRASPPLANNING_H_
#define PMGRASPPLANNING_H_

#include <pm_perception/background_removal.h>
#include <pm_perception/object_segmentation.h>
#include <pm_manipulation/hypothesis_generation.h>

#include <pcl/io/pcd_io.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpPoint.h>
#include <visp/vpHomogeneousMatrix.h>

#include <tf/transform_datatypes.h>
#include <list>

typedef pcl::PointXYZRGB PointT;

/** Description
 */
class PMGraspPlanning {
  pcl::PointCloud<PointT>::Ptr cloud_;
  //Grasping params (to allow different grasps and radious (for grasp penetration)).
  double angle_, rad_, along_;
  //Punto central del cilindro y la direccion.
  PointT axis_point_g; tf::Vector3 normal_g;
  bool aligned_grasp_;
  //VispToTF vispToTF;
  //MarkerPublisher * cylPub;
  double plane_distance_threshold_, cylinder_distance_threshold_, radious_limit_;
  int plane_iterations_, cylinder_iterations_;

public:

  vpHomogeneousMatrix cMg, cMo; ///< Grasp frame with respect to the camera after planning
  double radious, height;
  //With integuers to use trackbars
  int iangle, irad, ialong, ialigned_grasp;

  BackgroundRemoval * background_remover_;
  ObjectSegmentation * segmentator_;
  HypothesisGeneration * hypothesis_generation_;

  /** Constructor.
   * @param cloud, background_remover, segmentator, hypothesis_generator
   * */
  PMGraspPlanning(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, BackgroundRemoval * background_remover,
                  ObjectSegmentation * segmentator, HypothesisGeneration * hypothesis_generation){
    angle_=0;iangle=0;
    rad_=0;irad=0;
    along_=0;ialong=0;
    setAlignedGrasp(true);ialigned_grasp=1;
    //vispToTF.addTransform(cMg, "/stereo", "/cMo", "1");
    //vispToTF.addTransform(cMg, "/stereo", "/cMg", "2");
    setPlaneSegmentationParams();
    setCylinderSegmentationParams();

    cloud_ = cloud;

    background_remover_ = background_remover;
    segmentator_ = segmentator;
    hypothesis_generation_ = hypothesis_generation;
  }

  void proccessScene();
  std::list<vpHomogeneousMatrix> cMg_list;
  void generateGraspList();
  void filterGraspList();
  /** Description */
  void getGraspHypothesis();


  ~PMGraspPlanning() {}

  /** Main function where segmentation is done */
  void perceive();

  /** Set whether to perform a grasp aligned with the cylinder axis or not **/
  void setAlignedGrasp(bool a) {aligned_grasp_=a;}

  void setPlaneSegmentationParams(double distanceThreshold = 0.03, int iterations = 100){
    plane_distance_threshold_=distanceThreshold;
    plane_iterations_=iterations;
  }
  void setCylinderSegmentationParams(double distanceThreshold = 0.05,int iterations = 20000, double rlimit = 0.1){
    cylinder_distance_threshold_=distanceThreshold;
    cylinder_iterations_=iterations;
    radious_limit_=rlimit;
  }

  /** Get the grasp frame with respect to the camera frame */
  vpHomogeneousMatrix get_cMg() {return cMg;}

  /** Recalculate cMg with current parameters */
  void recalculate_cMg();

  /** Get the grasp frame pose with respect to an arbitrary frame 'b', given as input relative to the camera frame
   * @param bMc an homogeneous matrix with the camera frame given wrt the frame 'b'
   * @returns an homogeneous matrix with the grasp frame given wrt the frame 'b'
   */
  vpHomogeneousMatrix get_bMg(vpHomogeneousMatrix bMc) {return bMc*cMg;}

private:

  /** Comparer used in the sort function */
  bool sortFunction(const PointT& d1, const PointT& d2);

  /** Get the grasp frame with respect to the camera frame */
  void getMinMax3DAlongAxis(const pcl::PointCloud<PointT>::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT axis_point, tf::Vector3 * normal);

  /** Configure the camera based in int slider parameters */
  void intToConfig();


};
#endif /* PMGRASPPLANNING_H_ */
