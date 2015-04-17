/*
 * pose_estimation.cpp Box top face pose estimation from user input clicks
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
#include <pm_tools/visp_tools.h>

#include <geometry_msgs/Transform.h>

double TSIZE_X = 0.140, TSIZE_Y = 0.30;
//Box dimensions -> 150,300,160

void PoseEstimation::process()
{
  VirtualImage g(*nh_, image_topic_, image_info_topic_); //IMAGE_SUBSAMPLE);
  ros::Rate r(4), rWait(1);

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

  bool enable_visualization = true;
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

    ROS_INFO("Give me the points");
    vpImagePoint clicks[4], cursor;
    int nclicks = 0;

    while (nclicks < 4)
    {
      //vpImageConvert::convert(*Ic,*I);
      vpDisplay::display(Ic);
      if (nclicks >= 1)
      {
        //draw click point
        vpDisplay::displayCross(Ic, clicks[0], 10, vpColor::red, 3);
      }
      if (nclicks >= 2)
      {
        //draw 2nd point, Y line and bounding box
        vpDisplay::displayCross(Ic, clicks[1], 10, vpColor::red, 3);
        vpDisplay::displayLine(Ic, clicks[0], clicks[1], vpColor::green, 3);
        //get mouse pointer and draw bounding box
        //vpDisplay::getPointerPosition(Ic,cursor);

      }
      if (nclicks >= 3)
      {
        //draw 2nd point, Y line and bounding box
        vpDisplay::displayCross(Ic, clicks[2], 10, vpColor::red, 3);
        vpDisplay::displayLine(Ic, clicks[1], clicks[2], vpColor::green, 3);
        //get mouse pointer and draw bounding box
        //vpDisplay::getPointerPosition(Ic,cursor);

      }

      if (vpDisplay::getClick(Ic, clicks[nclicks], false))
      {
        //click done
        nclicks++;
        if (nclicks == 4)
        {
          //draw 2nd point, Y line and bounding box
          vpDisplay::displayCross(Ic, clicks[3], 10, vpColor::red, 3);
          vpDisplay::displayLine(Ic, clicks[2], clicks[3], vpColor::green, 3);
          //get mouse pointer and draw bounding box
          //vpDisplay::getPointerPosition(Ic,cursor);

        }
      }
      vpDisplay::flush(Ic);
    }
    ROS_INFO_STREAM(clicks[0]);
    ROS_INFO_STREAM(clicks[1]);
    ROS_INFO_STREAM(clicks[2]);
    ROS_INFO_STREAM(clicks[3]);

    //Compute 3D points from image points.
    for (int i = 0; i < 4; i++)
    {
      double x = 0, y = 0;
      vpPixelMeterConversion::convertPoint(g.K, clicks[i], x, y);
      P[i].set_x(x);
      P[i].set_y(y);
      pose.addPoint(P[i]);
    }

    // Compute template 3D pose
    pose.computePose(vpPose::LAGRANGE, cMo);

    //publish the pose and target detected
    geometry_msgs::Transform cMot;
    cMot = VispTools::geometryTransFromVispHomog( cMo );
    pose_pub.publish(cMot);

    ROS_INFO_STREAM("Computed pose (geometry transform): " << cMot);
    ROS_INFO_STREAM("Computed pose (ViSP): " << cMo);

    if (enable_visualization)
    {
      vpDisplay::displayFrame(Ic, cMo, g.K, 0.1, vpColor::none, 3);
      //distance to target
      char dtt[32];
      sprintf(dtt, "DTT: %.3f m", cMo[2][3]);
      //vpDisplay::setFont(Ic,"-adobe-courier-*-*-*-*-12-*-*-*-*-*-*-*");
      vpDisplay::displayRectangle(Ic, 0, 0, 90, 20, vpColor::black, true);
      vpDisplay::displayCharString(Ic, 15, 5, dtt, vpColor::white);
      ros::Duration(3).sleep();

    }
  }

}

