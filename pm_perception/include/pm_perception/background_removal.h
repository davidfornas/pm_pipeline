/*
 * background_removal.h
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef BACKGROUNDREMOVAL_H_
#define BACKGROUNDREMOVAL_H_

#include <pcl/io/pcd_io.h>
#include <pm_tools/pcl_segmentation.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;

/** Remove background from a PointCloud. Uses RANSAC plane fitting or theresholding.
 */
class BackgroundRemoval {

  CloudPtr cloud_;

public:
  pcl::ModelCoefficients::Ptr coefficients_plane;
  double plane_distance_threshold_;
  int plane_iterations_;
  bool ransac_background_filter_;


  /** Constructor.
   * @param source cloud
   */
  BackgroundRemoval(CloudPtr source,  double distanceThreshold = 0.05){
    setPlaneSegmentationParams(distanceThreshold);
    ransac_background_filter_ = true;
    coefficients_plane = (pcl::ModelCoefficients::Ptr) new pcl::ModelCoefficients;
  }

  /** Set background removal mode */
  void setRansacBackgroundFilter( bool ransac_background_filter ){ ransac_background_filter_ = ransac_background_filter; }

  /** Set plane segmentation parameters: distance to the inliers to the plane
   * and number of iterations.
   */
  void setPlaneSegmentationParams(double distanceThreshold = 0.03, int iterations = 100){
    plane_distance_threshold_=distanceThreshold;
    plane_iterations_=iterations;
  }
  /** Set new input cloud */
  void setNewCloud(CloudPtr cloud){ cloud_ = cloud; }

  //void setParameters(){}

  /* Remove plane and filter source cloud into output */
  void initialize(CloudPtr & output, pcl::PointCloud<pcl::Normal>::Ptr &output_normals);

  /* Remove plane and filter source cloud into output */
  void process(CloudPtr & output, pcl::PointCloud<pcl::Normal>::Ptr &output_normals);

  /* Remove bg plane iteratively */
  void removeIteratively(CloudPtr & output);

  ~BackgroundRemoval(){}

  typedef boost::shared_ptr<BackgroundRemoval> Ptr;
};

#endif /* BACKGROUNDREMOVAL_H_ */
