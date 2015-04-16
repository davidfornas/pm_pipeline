/*
 * pose_estimation.cpp
 *
 *  Created on: 16/04/2014
 *      Author: dfornas
 */

#include <pm_perception/pose_estimation.h>

#include <visp/vpPose.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpDisplayX.h>

#include <pm_tools/virtual_image.h>
#include <geometry_msgs/Transform.h>

double TSIZE_X, TSIZE_Y;

void PoseEstimation::process()
{
  VirtualImage g(*nh_, image_topic_, image_info_topic_); //IMAGE_SUBSAMPLE);
  ros::Rate r(4);

  vpImage<vpRGBa> Ic, Ipat; // Color image and pattern image
  vpImage<unsigned char> I; // Grey level image

  while (!g.ready() && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  if (!ros::ok())
    return;

  std::string object_pose_topic = "/pose";
  //Advertise object pose
  ros::Publisher pose_pub = nh_->advertise<geometry_msgs::Transform>(object_pose_topic, 1);

  //wait for the image to stabilize
  ROS_INFO("Getting images...");
  while (!g.ready() && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  if (!ros::ok())
    return;

  // Open the framegrabber by loading the first image of the sequence
  g.open(Ic);
  g.acquire(Ic);

  bool enable_visualization = false;
  vpDisplayX window;
  if (enable_visualization)
  {
    window.init(Ic);
  }

  //Template model initialization
  vpPoint P[4];
  vpPose pose;

  //Real object
  P[0].setWorldCoordinates(-TSIZE_X / 2, -TSIZE_Y / 2, 0);
  P[1].setWorldCoordinates(TSIZE_X / 2, -TSIZE_Y / 2, 0);
  P[2].setWorldCoordinates(TSIZE_X / 2, TSIZE_Y / 2, 0);
  P[3].setWorldCoordinates(-TSIZE_X / 2, TSIZE_Y / 2, 0);

  while (ros::ok())
  {
    ros::spinOnce();

    // read the image and then increment the image counter so that the next
    // call to acquire(I) will get the next image
    g.acquire(Ic);
    vpImageConvert::convert(Ic, I);

    if (enable_visualization)
      vpDisplay::display(Ic);

    vpHomogeneousMatrix cMo;
    pose.clearPoint();
    // Compute template 3D pose
    //It was computed from template, now it must be done from user input
    double userX[4], userY[4];
    for (int i = 0; i < 4; i++)
    {
      double x = 0, y = 0;
      vpPixelMeterConversion::convertPoint(g.K, userX[i], userY[i], x, y);
      P[i].set_x(x);
      P[i].set_y(y);
      pose.addPoint(P[i]);
    }

    pose.computePose(vpPose::LAGRANGE, cMo);

    //publish the pose and target detected
    geometry_msgs::Transform cMot;
    //KDL::Rotation cRo(cMo[0][0], cMo[0][1], cMo[0][2], cMo[1][0], cMo[1][1], cMo[1][2], cMo[2][0], cMo[2][1],
        //              cMo[2][2]);
    cMot.translation.x = cMo[0][3];
    cMot.translation.y = cMo[1][3];
    cMot.translation.z = cMo[2][3];
    //cRo.GetQuaternion(cMot.rotation.x, cMot.rotation.y, cMot.rotation.z, cMot.rotation.w);
    pose_pub.publish(cMot);

    if (enable_visualization)
    {

      vpDisplay::displayFrame(Ic, cMo, g.K, 0.1, vpColor::none, 3);
      //distance to target
      char dtt[32];
      sprintf(dtt, "DTT: %.3f m", cMo[2][3]);
      //vpDisplay::setFont(Ic,"-adobe-courier-*-*-*-*-12-*-*-*-*-*-*-*");
      vpDisplay::displayRectangle(Ic, 0, 0, 90, 20, vpColor::black, true);
      vpDisplay::displayCharString(Ic, 15, 5, dtt, vpColor::white);

    }
  }

}

