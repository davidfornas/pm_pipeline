/**
 * Registration of clouds using ARMarker Poses
 *  Created on: 21/11/2017
 *      Author: dfornas
 */


#include <pm_registration/marker_registration.h>

void MarkerRegistration::run() {

  // @TODO THAT WILL BE USED TO TRANSFORM USING POSE INSTEAD OF TF. DO NOT DELETE JUST IN CASE TF IS PREFERED (not with bags)
  //The marker is -1m. in Z with respect to World. This is not used for regitration.
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

void MarkerRegistration::pairAlign(const CloudPtr cloud_src, const CloudPtr cloud_tgt, CloudPtr output,
                                   Eigen::Matrix4f &final_transform, bool downsample) {

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
  ROS_INFO_STREAM("Cloud sizes: " << PCLTools<PointT>::nanAwareCount(src) << " and " << PCLTools<PointT>::nanAwareCount(tgt));

  std::vector<int> indices;
  PCLTools<PointT>::removeNanPoints(src);
  PCLTools<PointT>::removeNanPoints(tgt);

  // Compute surface normals and curvature
  CloudWithNormalsPtr points_with_normals_src (new CloudWithNormals);
  CloudWithNormalsPtr points_with_normals_tgt (new CloudWithNormals);


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
  PointRepXYZCurv point_representation;
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
  reg.setPointRepresentation (boost::make_shared<const PointRepXYZCurv> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  CloudWithNormalsPtr reg_result = points_with_normals_src;
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

  //ICP Score, the lower, the better.
  double score = 1.0;
  if(reg.hasConverged()) {
    score = reg.getFitnessScore();
    ROS_DEBUG_STREAM("Icp Score: " << score);
  }
  showICPRefinement(opt, src, score);

  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
}

void MarkerRegistration::showMap(const CloudPtr map, bool pause) {
  p->removePointCloud ("vp3_map");
  p->addPointCloud (map, "vp3_map", vp_3);
  if(pause){
    ROS_INFO ("Press q to continue the registration.");
    p->spin ();
  }else{
    p->spinOnce();
  }
}

void MarkerRegistration::showICPRefinement(const CloudPtr cloud_target, const CloudPtr cloud_source, double score) {
  p->removePointCloud ("vp4_target");
  p->removePointCloud ("vp4_source");
  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp4_target", vp_4);
  p->addPointCloud (cloud_source, src_h, "vp4_source", vp_4);
  ROS_INFO_STREAM ("ICP Score:" << score);
  p->spinOnce ();
}

void MarkerRegistration::showMarkerAlignment(const CloudPtr cloud_target, const CloudPtr cloud_source) {
  p->removePointCloud ("vp2_source");
  p->removePointCloud ("vp2_target");
  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp2_target", vp_2);
  p->addPointCloud (cloud_source, src_h, "vp2_source", vp_2);
  p-> spinOnce();
}

void MarkerRegistration::showOriginalDifference(const CloudPtr cloud_target, const CloudPtr cloud_source) {
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");
  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
  p-> spinOnce();
}

void MarkerRegistration::detectionCallback(const geometry_msgs::PoseStamped::ConstPtr &pose,
                                           const sensor_msgs::PointCloud2::ConstPtr &cloud) {
  //Convert input cloud to PCL
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*cloud, pcl_pc);
  //PCL Generic cloud to PointT strong type.
  pcl::fromPCLPointCloud2(pcl_pc, *current_cloud_);
  //Z Filter to get rid of plane
  // @TODO Remove plane with other method (RANSAC). Other option is doing euclidean clustering.
  PCLTools<PointT>::applyZAxisPassthrough(current_cloud_, 0.3, 1.15);
  PCLTools<PointT>::applyVoxelGridFilter(current_cloud_, 0.02);
  ROS_INFO_STREAM("PointCloud from topic filtered: " << current_cloud_->points.size() << " points.");

  current_pose_.header = pose->header;
  current_pose_.pose = pose->pose;

  CloudPtr pre_aligned_cloud;
  pre_aligned_cloud = boost::shared_ptr<Cloud>( new Cloud() );

  Eigen::Matrix4f prealignTransform = Eigen::Matrix4f::Identity();

  if(first_cloud_){
    //Initialize previus cloud and pose
    prev_pose_ = current_pose_;
    first_cloud_ = false;
    prev_cloud_->header = current_cloud_->header;
    map_cloud_->header = current_cloud_->header;
    for(int i=0; i< current_cloud_->points.size(); i++){
      prev_cloud_->points.push_back( current_cloud_->points[i] );
      map_cloud_->points.push_back( current_cloud_->points[i] );
    }

  }else{
    //Get trasform between map and current. c stands for camera, m stands for maker, 2 is from current pose and 1 is from prev.
    vpHomogeneousMatrix c2Mm2 = VispTools::vispHomogFromGeometryPose(  current_pose_.pose );
    vpHomogeneousMatrix c1Mm1 = VispTools::vispHomogFromGeometryPose(  prev_pose_.pose );
    vpHomogeneousMatrix wMm1(0, 0, 1, 0, 0, 0), wMm2(0, 0, 1, 0, 0, 0);
    vpHomogeneousMatrix c1Mc2;
    c1Mc2 = c1Mm1 * wMm1.inverse() * wMm2 * c2Mm2.inverse();
    ROS_INFO_STREAM("Position transform between clouds (using markers) ->  X: " << c1Mc2[0][3] << "Y: " << c1Mc2[1][3] << "Z:" << c1Mc2[2][3]);

    prealignTransform = VispTools::vpHomogeneousMatrixToEigenMatrix4f(c1Mc2);

    //Create prealigned cloud.
    for(int i=0; i< current_cloud_->points.size(); i++){
      PointT new_point = current_cloud_->points[i];
      vpHomogeneousMatrix point(current_cloud_->points[i].x, current_cloud_->points[i].y, current_cloud_->points[i].z, 0, 0, 0);
      point = c1Mc2 * point ;
      new_point.x = static_cast<float>(point [0][3]);
      new_point.y = static_cast<float>(point [1][3]);
      new_point.z = static_cast<float>(point [2][3]);
      pre_aligned_cloud->points.push_back( new_point );
    }
    ;

  }

  /*    PRINT MAP ONCE ITS BUILT
  ROS_INFO_STREAM("Map size: " << prev_cloud_->points.size() << " data points." << std::endl);
  sensor_msgs::PointCloud2 out_message;

  pcl::PCLPointCloud2 out_pcl_pc;
  //PointT strong type to PCL Generic cloud.
  pcl::toPCLPointCloud2(*prev_cloud_, out_pcl_pc);
  pcl_conversions::fromPCL(out_pcl_pc, out_message);
  map_pub_.publish(out_message);
  */

  //@ TODO filter out markers with big shifts in orientation.
  double summd;
  summd = (current_pose_.pose.position.x - prev_pose_.pose.position.x) + (current_pose_.pose.position.y - prev_pose_.pose.position.y)
          + (current_pose_.pose.position.z - prev_pose_.pose.position.z);
  ROS_INFO_STREAM("Marker difference" << summd);

  showOriginalDifference(current_cloud_, prev_cloud_);
  showMarkerAlignment(pre_aligned_cloud, prev_cloud_);

  Eigen::Matrix4f pairTransform;
  CloudPtr aligned_cloud (new Cloud);
  pairAlign (prev_cloud_, pre_aligned_cloud, aligned_cloud, pairTransform, false);

  ROS_INFO_STREAM(pre_aligned_cloud);
  globalTransform = pairTransform * prealignTransform * globalTransform;
  CloudPtr final_cloud = boost::shared_ptr<Cloud>( new Cloud() );

  pcl::transformPointCloud (*current_cloud_, *final_cloud, globalTransform);

  //Add cloud with different color.
  uint8_t r = static_cast<uint8_t>(std::rand() % 255);
  uint8_t g = static_cast<uint8_t>(std::rand() % 255);
  uint8_t b = static_cast<uint8_t>(std::rand() % 255);
  for(int i=0; i< final_cloud->points.size(); i++){
    final_cloud->points[i].r = r;
    final_cloud->points[i].g = g;
    final_cloud->points[i].b = b;
    map_cloud_->points.push_back( final_cloud->points[i] );
  }

  //VOXELGRID
  showMap(map_cloud_, false);
  ROS_INFO_STREAM("Map size: " << map_cloud_->points.size() << " points.");
  ROS_INFO_STREAM("Final: " << final_cloud->points.size() << " points.");

  //Update prev cloud. Probably not very efficient.
  prev_pose_ = current_pose_;
  prev_cloud_->header = current_cloud_->header;
  prev_cloud_->points.resize(0);
  for(int i=0; i< current_cloud_->points.size(); i++){
    prev_cloud_->points.push_back( current_cloud_->points[i] );
  }

}

MarkerRegistration::MarkerRegistration(ros::NodeHandle &nh, int &argc, char **argv) {

  broadcaster_ = new tf::TransformBroadcaster();
  listener_ = new tf::TransformListener();

  //Subscribe both to Pose and Cloud to have marker position properly
  // @TODO Static names to parameters
  pose_sub_ = new PoseSub(nh, "/pose", 1);
  cloud_sub_ = new CloudSub(nh, "/stereo/points2", 1);

  sync = new message_filters::Synchronizer<CloudPoseSync>(CloudPoseSync(5), *pose_sub_, *cloud_sub_);
  sync->registerCallback(boost::bind(&MarkerRegistration::detectionCallback, this, _1, _2));

  current_cloud_ = boost::shared_ptr<Cloud>( new Cloud() );
  prev_cloud_ = boost::shared_ptr<Cloud>( new Cloud() );
  map_cloud_ = boost::shared_ptr<Cloud>( new Cloud() );
  full_res_map_cloud_ = boost::shared_ptr<Cloud>( new Cloud() );

  map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/out", 1000);

  first_cloud_ = true;
  globalTransform = Eigen::Matrix4f::Identity();


  // Create a PCLVisualizer object.  4 viewports. 1)Originals. 2)Marker correction 3)Marker+ICP 4)Map
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0.5, 0.5, 1.0, vp_1);
  p->createViewPort (0.0, 0.0, 0.5, 0.5, vp_2);
  p->createViewPort (0.5, 0.5, 1.0, 1.0, vp_3);
  p->createViewPort (0.5, 0.0, 1.0, 0.5, vp_4);

  p->addText( "Original difference",    0, 0, "1", vp_1);
  p->addText( "Alignment with markers", 0, 0, "2", vp_2);
  p->addText( "Full map",               0, 0, "3", vp_3);
  p->addText( "ICP Refinement",         0, 0, "4", vp_4);

}


