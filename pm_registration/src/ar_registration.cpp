/**
 * Registration of clouds using ARMarker Poses
 *  Created on: 21/11/2017
 *      Author: dfornas
 */


#include <pm_tools/tf_tools.h>
#include <pm_tools/visp_tools.h>
#include <pm_tools/pcl_tools.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

typedef message_filters::Subscriber<geometry_msgs::PoseStamped> PoseSub;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> CloudSub;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> CloudPoseSync;

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;
typedef pcl::PointNormal NormalT;
typedef pcl::PointCloud<NormalT> CloudWithNormals;

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <NormalT>
{
    using pcl::PointRepresentation<NormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
      // Define the number of dimensions
      nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const NormalT &p, float * out) const
    {
      // < x, y, z, curvature >
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

    geometry_msgs::PoseStamped current_pose_, map_pose_;

    CloudPtr current_cloud_, map_cloud_;
    pcl::visualization::PCLVisualizer *p;
    //its left and right viewports
    int vp_1, vp_2, vp_3;

    ros::Publisher cloud_pub_;
    bool first_cloud_;

public:

    MarkerRegistration(ros::NodeHandle &nh, int & argc, char * argv[]){

        broadcaster_ = new tf::TransformBroadcaster();
        listener_ = new tf::TransformListener();

        //Subscribe both to Pose and Cloud to have marker position properly
        // @TODO Static names to parameters
        pose_sub_ = new PoseSub(nh, "/pose", 1);
        cloud_sub_ = new CloudSub(nh, "/stereo/points2", 1); //Prev value: stereo_camera

        sync = new message_filters::Synchronizer<CloudPoseSync>(CloudPoseSync(5), *pose_sub_, *cloud_sub_);
        sync->registerCallback(boost::bind(&MarkerRegistration::detectionCallback, this, _1, _2));

        current_cloud_ = boost::shared_ptr<Cloud>( new Cloud() );
        map_cloud_ = boost::shared_ptr<Cloud>( new Cloud() );

        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/out", 1000);

        first_cloud_ = true;

        // Create a PCLVisualizer object
        p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
        p->createViewPort (0.0,  0, 0.33, 1.0, vp_1);
        p->createViewPort (0.33, 0, 0.66, 1.0, vp_2);
        p->createViewPort (0.66, 0, 1.0,  1.0, vp_3);


    }


    void run(){

        //WIP THAT WILL BE USED TO TRANSFORM USING POSE INSTEAD OF TF. DO NOT DELETE JUST IN CASE TF IS PREFERED (not with bags)
        //The marker is -1m. in Z with respect to World. This is not used for resgitration.
        ros::Rate r(10);
        while(ros::ok()){

            bool gotTransform = false;
            tf::StampedTransform markerMcamera; // World to camera (sense3d or stereo_camera)

            while (!gotTransform)
            {
                try{
                    listener_->lookupTransform( "ar_marker", "stereo", ros::Time(0), markerMcamera); // "sense3d" or any stereo_frame
                    gotTransform = true;
                }
                catch (tf::TransformException & ex){
                    ROS_DEBUG("%s",ex.what());
                }
            }

            //pose_out.header.frame_id = target_frame_id;
            //SUSCRIBE TO cMm e invertir Ó TO mMc
            vpHomogeneousMatrix mMc = VispTools::vispHomogFromTfTransform( markerMcamera );
            vpHomogeneousMatrix wMm(0, 0, -1, 0, 0, 3.1416);
            vpHomogeneousMatrix wMc = wMm * mMc;

            tf::Transform t = VispTools::tfTransFromVispHomog(wMc);
            tf::StampedTransform transform(t, ros::Time::now(), "world", "stereo");//Not now but the same as the listened: markerMcamera.stamp_
            broadcaster_->sendTransform(transform);
            ros::spinOnce();
            r.sleep();
        }
    }

    void detectionCallback(const geometry_msgs::PoseStamped::ConstPtr& pose, const sensor_msgs::PointCloud2::ConstPtr& cloud)
    {
        //Convert input cloud to PCL
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*cloud, pcl_pc);
        //PCL Generic cloud to PointT strong type.
        pcl::fromPCLPointCloud2(pcl_pc, *current_cloud_);
        PCLTools<PointT>::applyZAxisPassthrough(current_cloud_, 0.3, 1.15);
        PCLTools<PointT>::applyVoxelGridFilter(current_cloud_, 0.01);
        ROS_INFO_STREAM("PointCloud loaded: " << current_cloud_->points.size() << " data points." << std::endl);

        current_pose_.header = pose->header;
        current_pose_.pose = pose->pose;

        CloudPtr pre_aligned_cloud;
        pre_aligned_cloud = boost::shared_ptr<Cloud>( new Cloud() );


        if(first_cloud_){
            //Initialize map cloud and pose
            map_pose_ = current_pose_;
            first_cloud_ = false;
            map_cloud_->header = current_cloud_->header;
            for(int i=0; i< current_cloud_->points.size(); i++){
                map_cloud_->points.push_back( current_cloud_->points[i] );
            }

        }else{
            //Get trasform between map and current
            vpHomogeneousMatrix c2Mm2 = VispTools::vispHomogFromGeometryPose(  current_pose_.pose );
            vpHomogeneousMatrix c1Mm1 = VispTools::vispHomogFromGeometryPose(  map_pose_.pose );
            vpHomogeneousMatrix wMm1(0, 0, 1, 0, 0, 0), wMm2(0, 0, 1, 0, 0, 0);
            vpHomogeneousMatrix c1Mc2;
            c1Mc2 = c1Mm1 * wMm1.inverse() * wMm2 * c2Mm2.inverse();
            ROS_INFO_STREAM("Transform. X: "<<c1Mc2[0][3] << "Y: "<<c1Mc2[1][3] << "Z:" << c1Mc2[2][3]);

            //Add cloud points to map
            for(int i=0; i< current_cloud_->points.size(); i++){
                PointT new_point = current_cloud_->points[i];
                vpHomogeneousMatrix point(current_cloud_->points[i].x, current_cloud_->points[i].y, current_cloud_->points[i].z, 0, 0, 0);
                point = c1Mc2 * point ;
                new_point.x = point [0][3];
                new_point.y = point [1][3];
                new_point.z = point [2][3];
                pre_aligned_cloud->points.push_back( new_point );
            }
          ;

        }

      /*ROS_INFO_STREAM("Map size: " << map_cloud_->points.size() << " data points." << std::endl);
      sensor_msgs::PointCloud2 out_message;

      pcl::PCLPointCloud2 out_pcl_pc;
      //PointT strong type to PCL Generic cloud.
      pcl::toPCLPointCloud2(*map_cloud_, out_pcl_pc);
      pcl_conversions::fromPCL(out_pcl_pc, out_message);
      cloud_pub_.publish(out_message);
*/
      float summd = (current_pose_.pose.position.x - map_pose_.pose.position.x) + (current_pose_.pose.position.y - map_pose_.pose.position.y) + (current_pose_.pose.position.z - map_pose_.pose.position.z);
      ROS_INFO_STREAM("Marker difference" << summd);

      showCloudsLeft(current_cloud_, map_cloud_);
      showCloudsCenter(pre_aligned_cloud, map_cloud_);

      Eigen::Matrix4f pairTransform;
      CloudPtr aligned_cloud (new Cloud);
      pairAlign (map_cloud_, pre_aligned_cloud, aligned_cloud, pairTransform, false);

      //showCloudsRight(aligned_cloud, map_cloud_);

      map_pose_ = current_pose_;
      //first_cloud_ = false;
      map_cloud_->header = current_cloud_->header;
      map_cloud_->points.resize(0);
      for(int i=0; i< current_cloud_->points.size(); i++){
        map_cloud_->points.push_back( current_cloud_->points[i] );
      }

    }

    ////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
    void showCloudsLeft(const CloudPtr cloud_target, const CloudPtr cloud_source)
    {
      p->removePointCloud ("vp1_target");
      p->removePointCloud ("vp1_source");

      pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
      p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
      p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

      p-> spinOnce();
    }

    ////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
    void showCloudsCenter(const CloudPtr cloud_target, const CloudPtr cloud_source)
    {
      p->removePointCloud ("vp2_target");
      p->removePointCloud ("vp2_source");

      pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
      p->addPointCloud (cloud_target, tgt_h, "vp2_target", vp_2);
      p->addPointCloud (cloud_source, src_h, "vp2_source", vp_2);

      p-> spinOnce();
    }

    /*void showCloudsRight(const CloudPtr cloud_target, const CloudPtr cloud_source)
    {
      p->removePointCloud ("vp3_target");
      p->removePointCloud ("vp3_source");

      pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
      p->addPointCloud (cloud_target, tgt_h, "vp3_target", vp_3);
      p->addPointCloud (cloud_source, src_h, "vp3_source", vp_3);

      p-> spin();
    }*/

    ////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
    void showCloudsRight(const CloudWithNormals::Ptr cloud_target, const CloudWithNormals::Ptr cloud_source)
    {
      p->removePointCloud ("vp3_source");
      p->removePointCloud ("vp3_target");


      /*pcl::visualization::PointCloudColorHandlerGenericField<NormalT> tgt_color_handler (cloud_target, "curvature");
      if (!tgt_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");

      pcl::visualization::PointCloudColorHandlerGenericField<NormalT> src_color_handler (cloud_source, "curvature");
      if (!src_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");*/
      pcl::visualization::PointCloudColorHandlerCustom<NormalT> tgt_h (cloud_target, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<NormalT> src_h (cloud_source, 255, 0, 0);

      /*p->addPointCloud (cloud_target, tgt_color_handler, "vp3_target", vp_3);
      p->addPointCloud (cloud_source, src_color_handler, "vp3_source", vp_3);*/
      p->addPointCloud (cloud_target, tgt_h, "vp3_target", vp_3);
      p->addPointCloud (cloud_source, src_h, "vp3_source", vp_3);

      p->spinOnce();
    }



////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
    void pairAlign (const CloudPtr cloud_src, const CloudPtr cloud_tgt, CloudPtr output, Eigen::Matrix4f &final_transform, bool downsample = false)
    {

      // Downsample for consistency and speed. Enable this for large datasets
      CloudPtr src (new Cloud), tgt (new Cloud), opt (new Cloud);
      pcl::VoxelGrid<PointT> grid;
      if (downsample)
      {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
      }
      else
      {
        src = cloud_src;
        tgt = cloud_tgt;
      }

      ROS_INFO_STREAM("ALIGN: " << PCLTools<PointT>::nanAwareCount(src) << "X" << PCLTools<PointT>::nanAwareCount(tgt));

      std::vector<int> indices;
      PCLTools<PointT>::removeNanPoints(src);
      PCLTools<PointT>::removeNanPoints(tgt);
      //pcl::removeNaNFromPointCloud(*src,*src, indices);
      //pcl::removeNaNFromPointCloud(*tgt,*tgt, indices);



      // Compute surface normals and curvature
      CloudWithNormals::Ptr points_with_normals_src (new CloudWithNormals);
      CloudWithNormals::Ptr points_with_normals_tgt (new CloudWithNormals);


      pcl::NormalEstimation<PointT, NormalT> norm_est;
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
      norm_est.setSearchMethod (tree);
      norm_est.setKSearch (30);

      norm_est.setInputCloud (src);
      norm_est.compute (*points_with_normals_src);
      pcl::copyPointCloud (*src, *points_with_normals_src);

      norm_est.setInputCloud (tgt);
      norm_est.compute (*points_with_normals_tgt);
      pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

      // Instantiate our custom point representation (defined above) ...
      MyPointRepresentation point_representation;
      // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
      float alpha[4] = {1.0, 1.0, 1.0, 1.0};
      point_representation.setRescaleValues (alpha);

      //
      // Align
      pcl::IterativeClosestPointNonLinear<NormalT, NormalT> reg;
      reg.setTransformationEpsilon (1e-6);
      // Set the maximum distance between two correspondences (src<->tgt) to 10cm
      // Note: adjust this based on the size of your datasets
      reg.setMaxCorrespondenceDistance (0.1);
      // Set the point representation
      reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

      reg.setInputSource (points_with_normals_src);
      reg.setInputTarget (points_with_normals_tgt);

      //
      // Run the same optimization in a loop and visualize the results
      Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
      CloudWithNormals::Ptr reg_result = points_with_normals_src;
      reg.setMaximumIterations (2);
      for (int i = 0; i < 100; ++i)
      {
        PCL_DEBUG ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
          reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();


        // visualize current state
        //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
      }

      //
      // Get the transformation from target to source
      targetToSource = Ti.inverse();

      //
      // Transform target back in source frame
      pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
      pcl::transformPointCloud (*tgt, *opt, targetToSource);

      p->removePointCloud ("vp3_source");
      p->removePointCloud ("vp3_target");

      pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
      //p->addPointCloud (output, cloud_tgt_h, "target", vp_3);
      //p->addPointCloud (cloud_src, cloud_src_h, "source", vp_3);
      p->addPointCloud (opt, cloud_tgt_h, "vp3_target", vp_3);
      p->addPointCloud (src, cloud_src_h, "vp3_source", vp_3);

      if(reg.hasConverged()) {
        float score = reg.getFitnessScore();
        ROS_INFO_STREAM("SCORE: " << score);
      }
      PCL_INFO ("Press q to continue the registration.\n");
      p->spin ();

      p->removePointCloud ("vp3_source");
      p->removePointCloud ("vp3_target");

      //add the source to the transformed target
      *output += *cloud_src;

      final_transform = targetToSource;
    }
};

int main (int argc, char** argv)
{

    // Set up ROS.
    ros::init(argc, argv, "ar_marker_registration");
    ros::NodeHandle nh;

    MarkerRegistration mr(nh, argc, argv);
    mr.run();

    return (0);
}
