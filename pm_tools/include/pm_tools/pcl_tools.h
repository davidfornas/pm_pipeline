/*
 * pcl_tools Point cloud processing tools using PCL
 *  Created on: 03/03/2015
 *      Author: dfornas
 */
#ifndef PCLTOOLS_H_
#define PCLTOOLS_H_

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/normal_space.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/kdtree.h>

#include <ros/topic.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

template <typename PointT>
class PCLTools
{
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

public:

  static void cloudFromPCD(CloudPtr cloud, std::string fileName){
    pcl::PCDReader reader;
    reader.read(fileName, *cloud);
    ROS_DEBUG_STREAM("PointCloud loaded: " << cloud->points.size() << " data points." << std::endl);
  }

  static void cloudToPCD(CloudPtr cloud, std::string fileName){
    pcl::PCDWriter writer;
    //Binary format is used to avoid problems with Kinect, switch if necessary.
    writer.writeBinary(fileName, *cloud);
    ROS_DEBUG_STREAM("PointCloud saved." << cloud->points.size() << " data points."  << std::endl);
  }

  static void cloudFromTopic(CloudPtr cloud, std::string topicName){
    sensor_msgs::PointCloud2::ConstPtr message = ros::topic::waitForMessage< sensor_msgs::PointCloud2 >(topicName);
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*message, pcl_pc);
    //PCL Generic cloud to PointT strong type.
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);
    ROS_DEBUG_STREAM("PointCloud loaded: " << cloud->points.size() << " data points." << std::endl);
  }

  static void cloudToTopic(CloudPtr cloud, std::string topicName, ros::NodeHandle & n){
    ROS_ERROR_STREAM("Code not publishing properly");
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>(topicName, 1000);
    sensor_msgs::PointCloud2 message;

    pcl::PCLPointCloud2 pcl_pc;
    //PointT strong type to PCL Generic cloud.
    pcl::toPCLPointCloud2(*cloud, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, message);
    cloud_pub.publish(message);
    ros::spinOnce();
  }

  static void applyZAxisPassthrough(CloudPtr in, CloudPtr out, double min, double max){
    typename pcl::PassThrough<PointT> pass;
    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min, max);
    pass.filter (*out);
  }
  //In place filter version
  static void applyZAxisPassthrough(CloudPtr & in, double min, double max){
    CloudPtr result( new Cloud );
    applyZAxisPassthrough(in, result, min, max);
    in = result;
  }

  /** Voxel Grid filter filter */
  static void applyVoxelGridFilter(CloudPtr in, CloudPtr out,  float size = 0.03){
    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (in);
    vg.setLeafSize (size, size, size);
    vg.filter (*out);
  }
  static void applyVoxelGridFilter(CloudPtr & in,  float size = 0.03){
    CloudPtr result( new Cloud );
    applyVoxelGridFilter(in, result, size);
    in = result;
  }

  /** Statistical Outlier Removal filter */
  static void applyStatisticalOutlierRemoval(CloudPtr in, CloudPtr out, int meanK = 50, float StdThresh = 1.0){
    typename pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (in);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (StdThresh);
    sor.filter(*out);
  }
  static void applyStatisticalOutlierRemoval(CloudPtr & in, int meanK = 50, float StdThresh = 1.0){
    CloudPtr result( new Cloud );
    applyStatisticalOutlierRemoval(in, result, meanK, StdThresh);
    in = result;
  }

  /** Fast Bilateral Filter to smooth clouds filter */
  static void apllyFastBilateralFilter(CloudPtr & in, float bilateralSigmaS, float bilateralSigmaR){
    CloudPtr result( new Cloud );
    pcl::FastBilateralFilter<PointT> filter;
    filter.setInputCloud(in);
    filter.setSigmaS( bilateralSigmaS );
    filter.setSigmaR( bilateralSigmaR );
    filter.applyFilter(*result);
    in = result;
  }

  /** Median filter: The median filter is one of the simplest and wide-spread image processing filters. It is known to
   * perform well with "shot"/impulse noise (some individual pixels having extreme values), it does not reduce contrast
   * across steps in the function (as compared to filters based on averaging), and it is robust to outliers.
   * Furthermore, it is simple to implement and efficient, as it requires a single pass over the image. It consists of
   * a moving window of fixed size that replaces the pixel in the center with the median inside the window.
   * It only precesses Depth in organized non transformed Clouds.*/
  static void apllyMedianFilter(CloudPtr & in, int window_size = 5){
    pcl::MedianFilter<PointT> filter;
    filter.setWindowSize( window_size );
    filter.applyFilter(*in);
  }

  /** NormalSpaceSampling samples the input point cloud in the space of normal directions computed at every point. */
  static void normalSpaceSampling(CloudPtr & in, pcl::PointCloud<pcl::Normal>::Ptr normals, CloudPtr & out, int grid_res = 4, int samples = 1000){
    pcl::NormalSpaceSampling<PointT, pcl::Normal> sampler;
    sampler.setInputCloud(in);
    sampler.setNormals(normals);
    sampler.setBins(grid_res, grid_res, grid_res);
    sampler.setSample(samples);
    sampler.filter(*out);
  }


  /** Compute normals */
  static void estimateNormals(CloudPtr in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT> ());
    typename pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (in);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
  }

  /** Compute normals */
  static void estimateNormals(CloudPtr in, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals){
    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT> ());
    typename pcl::NormalEstimation<PointT, pcl::PointNormal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (in);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
  }

  /** Count not NaN points */
  static int nanAwareCount(CloudPtr p)
  {
    int count = 0;
    for (size_t i = 0; i < p->points.size(); ++i)
      if (pcl::isFinite(p->points[i]))
        count++;
    return count;
  }

  /** Center cloud on origin */
  static int moveToOrigin(CloudPtr p)
  {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid<PointT>(*p, centroid);
    for (size_t i = 0; i < p->points.size(); ++i) {
      p->points[i].x -= centroid.x();
      p->points[i].y -= centroid.y();
      p->points[i].z -= centroid.z();
    }
  }

  /** Remove NaN points, @TODO compare with PCL method */
  static void removeNanPoints(CloudPtr p, CloudPtr copy){
    /* COMPARE WITH ...
     *
     * std::vector<int> indices;
     * pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
     *
    */
    int new_size = PCLTools<PointT>::nanAwareCount(p);
    copy->width    = new_size;
    copy->height   = 1;
    copy->is_dense = false;
    copy->points.resize (copy->width * copy->height);

    int idx=0;
    for (size_t i = 0; i < p->points.size(); ++i)
      if (pcl::isFinite(p->points[i]))
        copy->points[idx++] = p->points[i];
    ROS_DEBUG_STREAM("New size:" << idx);
  }
  // In place NaN remove
  static void removeNanPoints(CloudPtr & p){
    CloudPtr result( new Cloud );
    removeNanPoints(p, result);
    p = result;
  }

  /** Fill gaps in A with points from B */
  static void mergeOrganizedClouds(CloudPtr a, CloudPtr b)
  {
    //A,B should be organized clouds of the same size... @ TODO CHECK
    for (size_t i = 0; i < a->points.size(); ++i)
      if (!pcl::isFinite(a->points[i]) && pcl::isFinite(b->points[i]))
        a->points[i] = b->points[i];
  }


  static int findFurthest(CloudPtr cloud, double a, double b, double c, double d, int & max_index, double & max_distance){
    max_index = -1;
    max_distance = 0;

    for (int i = 0; i < cloud->points.size(); ++i) {
      double distance = fabs( a * cloud->points[i].x + b * cloud->points[i].y + c * cloud->points[i].z + d );
      if(distance > max_distance){
        max_distance = distance;
        max_index = i;
      }
    }
  }

  static double signedDistanceToPlane( PointT p, double a, double b, double c, double d){
    return (a * p.x + b * p.y + c * p.z + d);
  }

  static PointT projectPoint(PointT P, PointT origin, PointT direction_unit_vector){
    PointT A, AP, AB, projection;
    A = origin;
    AB = direction_unit_vector;
    AP.x = P.x - A.x;
    AP.y = P.y - A.y;
    AP.z = P.z - A.z;
    // dot(AB,AP)
    double dotProduct = AP.x * AB.x + AP.y * AB.y + AP.z * AB.z;
    projection.x = A.x + AB.x * dotProduct;
    projection.y = A.y + AB.y * dotProduct;
    projection.z = A.z + AB.z * dotProduct;

    return projection;
  }

  static Eigen::Vector3f projectPoint(Eigen::Vector3f P, Eigen::Vector3f origin, Eigen::Vector3f direction_unit_vector){
    Eigen::Vector3f A, AP, AB, projection;
    A = origin;
    AB = direction_unit_vector;
    AP = P - A;
    // dot(AB,AP)
    double dotProduct = AP.dot(AB);
    projection = A + AB * dotProduct;
    return projection;
  }

  /** Show segmented cloud and plane by coefficients and inliers */
  static void showSegmentationCloudsAndModels(CloudPtr c1, CloudPtr c2, pcl::ModelCoefficients::Ptr plane_coeffs,
                                              pcl::ModelCoefficients::Ptr cylinder_coeffs)
  {

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer("Segmentation Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(c1, 20, 20, 90);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color2(c1, 20, 200, 20);

    viewer->addPointCloud<PointT>(c1, single_color, "Plane cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Plane cloud");
    viewer->addPointCloud<PointT>(c2, single_color2, "Cylinder cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cylinder cloud");

    if (plane_coeffs != 0)
    {
      viewer->addPlane(*plane_coeffs, "Plane");
    }
    if (cylinder_coeffs != 0)
    {
      viewer->addCylinder(*cylinder_coeffs, "Cylinder");
    }
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }
};

template <typename PointT>
class PCLView
{
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

public:
  /** Show cloud */
  static void showCloud(CloudPtr source)
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CViewer"));

    viewer->addPointCloud<PointT>(source, "Plane cloud");
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0.8,0.8,0.8);
    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    viewer->close();
  }


  /** Show cloud during ms miliseconds */
  static void showCloudDuring(CloudPtr source, int ms = 1000)
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CViewer"));

    int elapsed = 0;
    viewer->addPointCloud<PointT>(source, "Plane cloud");
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0.8,0.8,0.8);
    while (!viewer->wasStopped() && elapsed < ms )
    {
      elapsed += 20;
      viewer->spinOnce(20);
      boost::this_thread::sleep(boost::posix_time::microseconds(20000));
    }
    viewer->close();
  }


};

class EigenTools
{
public:

  static Eigen::Quaterniond
  euler2Quaternion( const double roll,
                    const double pitch,
                    const double yaw )
  {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    //Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    //Eigen::Matrix3d rotationMatrix = q.matrix();

    return q;
  }

  static Eigen::Quaterniond
  euler2Matrix( const double roll,
                const double pitch,
                const double yaw )
  {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();

    return q;
  }

};




#endif
