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

#include <pm_tools/visp_tools.h>

#include <geometry_msgs/Transform.h>

const double TSIZE_X = 0.140, TSIZE_Y = 0.30, TSIZE_Z = 0.160;
//Real black box dimensions -> 140, 300, 160 (mm)
const int NUM_POINTS = 6; // @ TODO Use parameter server or init at constructor.

vpHomogeneousMatrix PoseEstimation::process()
{
  VirtualImage g(*nh_, image_topic_, image_info_topic_);
  ros::Rate r(4);

  vpImage<vpRGBa> Ic, Ipat; // Color image and pattern image
  vpImage<unsigned char> I; // Grey level image

  while (!g.ready() && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  if (!ros::ok())
    return vpHomogeneousMatrix(0,0,0,0,0,0);

  std::string object_pose_topic = "/pose"; // @ TODO Get from parameter server
  //Advertise object pose
  ros::Publisher pose_pub = nh_->advertise<geometry_msgs::Transform>(object_pose_topic, 1);

  //Wait for the image to stabilize
  ROS_DEBUG("Getting images...");
  while (!g.ready() && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  if (!ros::ok())
    return vpHomogeneousMatrix(0,0,0,0,0,0);

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
  vpPoint P[NUM_POINTS];
  vpPose pose;

  //Possible models based on the number of corner used.
  P[0].setWorldCoordinates(-TSIZE_X / 2, -TSIZE_Y / 2, 0);
  P[1].setWorldCoordinates(TSIZE_X / 2, -TSIZE_Y / 2, 0);
  P[2].setWorldCoordinates(TSIZE_X / 2, TSIZE_Y / 2, 0);
  P[3].setWorldCoordinates(-TSIZE_X / 2, TSIZE_Y / 2, 0);

  if (NUM_POINTS == 5)
  {
    P[2].setWorldCoordinates(TSIZE_X / 2, -TSIZE_Y / 2, -TSIZE_Z);
    P[3].setWorldCoordinates(TSIZE_X / 2, TSIZE_Y / 2, 0);
    P[4].setWorldCoordinates(-TSIZE_X / 2, TSIZE_Y / 2, 0);
  }

  if (NUM_POINTS == 6)
  {
    P[2].setWorldCoordinates(TSIZE_X / 2, -TSIZE_Y / 2, -TSIZE_Z);
    P[3].setWorldCoordinates(TSIZE_X / 2, TSIZE_Y / 2, 0);
    P[4].setWorldCoordinates(TSIZE_X / 2, TSIZE_Y / 2, -TSIZE_Z);
    P[5].setWorldCoordinates(-TSIZE_X / 2, TSIZE_Y / 2, 0);
  }

  ros::spinOnce();

  g.acquire(Ic);
  vpImageConvert::convert(Ic, I);

  if (enable_visualization)
    vpDisplay::display(Ic);

  pose.clearPoint();

  //Get user clicks
  vpImagePoint cursor;
  int nclicks = 0;
  while (nclicks < NUM_POINTS)
  {
    if (vpDisplay::getClick(Ic, clicks_[nclicks], false))
    {
      //click done
      nclicks++;
    }

    if (NUM_POINTS == 4) displayClicks(Ic, nclicks);
    if (NUM_POINTS == 5) display5Clicks(Ic, nclicks);
    if (NUM_POINTS == 6) display6Clicks(Ic, nclicks);

    vpDisplay::flush(Ic);
  }
  ROS_DEBUG_STREAM(clicks_[0]);
  ROS_DEBUG_STREAM(clicks_[1]);
  ROS_DEBUG_STREAM(clicks_[2]);
  ROS_DEBUG_STREAM(clicks_[3]);
  if (NUM_POINTS == 5)
    ROS_DEBUG_STREAM(clicks_[4]);
  if (NUM_POINTS == 6)
    ROS_DEBUG_STREAM(clicks_[5]);

  //Compute 3D points from clicked image points.
  for (int i = 0; i < NUM_POINTS; i++)
  {
    double x = 0, y = 0;
    vpPixelMeterConversion::convertPoint(g.K, clicks_[i], x, y);
    P[i].set_x(x);
    P[i].set_y(y);
    pose.addPoint(P[i]);
  }

  // Compute template 3D pose choosing best solver
  vpHomogeneousMatrix cMo, cMo_dem, cMo_lag;
  pose.computePose(vpPose::DEMENTHON, cMo_dem);
  pose.computePose(vpPose::LAGRANGE, cMo_lag);
  double residual_dem = pose.computeResidual(cMo_dem);
  double residual_lag = pose.computeResidual(cMo_lag);
  if (residual_dem < residual_lag){
    cMo = vpHomogeneousMatrix(cMo_dem);
    ROS_INFO_STREAM("Computed pose [score: " << residual_dem <<"] (DEMENTHON): ");
    ROS_INFO_STREAM(cMo_dem);
  }else{
    cMo = vpHomogeneousMatrix(cMo_lag);
    ROS_INFO_STREAM("Computed pose [score: " << residual_lag <<"] (LAGRANGE): ");
    ROS_INFO_STREAM(cMo_lag);
  }

  //Publish the pose
  geometry_msgs::Transform cMot;
  cMot = VispTools::geometryTransFromVispHomog(cMo);
  pose_pub.publish(cMot);

  ROS_DEBUG_STREAM("Computed pose (geometry transform): " << cMot);
  ROS_DEBUG_STREAM("Computed pose (ViSP): " << cMo);

  //Visualization
  if (enable_visualization)
  {
    if (NUM_POINTS == 4) displayClicks(Ic, 4);
    if (NUM_POINTS == 5) display5Clicks(Ic, 5);
    if (NUM_POINTS == 6) display6Clicks(Ic, 6);
    vpDisplay::displayFrame(Ic, cMo, g.K, 0.1, vpColor::none, 3);
    //Display distance to target
    char dtt[32];
    sprintf(dtt, "DTT: %.3f m", cMo[2][3]);
    vpDisplay::displayRectangle(Ic, 0, 0, 90, 20, vpColor::black, true);
    vpDisplay::displayCharString(Ic, 15, 5, dtt, vpColor::white);
    vpDisplay::flush(Ic);
    ros::Duration(4).sleep();
  }
  return cMo;

}

void PoseEstimation::displayClicks(vpImage<vpRGBa> & Ic, int nclicks)
{
  vpDisplay::display(Ic);
  if (nclicks >= 1)
  {
    //draw 1st click point
    vpDisplay::displayCross(Ic, clicks_[0], 10, vpColor::red, 3);
  }
  if (nclicks >= 2)
  {
    //draw 2nd point
    vpDisplay::displayCross(Ic, clicks_[1], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[0], clicks_[1], vpColor::green, 3);
  }
  if (nclicks >= 3)
  {
    //draw 3rd point
    vpDisplay::displayCross(Ic, clicks_[2], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[1], clicks_[2], vpColor::green, 3);
  }
  if (nclicks == 4)
  {
    //draw 4th point
    vpDisplay::displayCross(Ic, clicks_[3], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[2], clicks_[3], vpColor::green, 3);
    vpDisplay::displayLine(Ic, clicks_[3], clicks_[0], vpColor::green, 3);
  }
}

void PoseEstimation::display5Clicks(vpImage<vpRGBa> & Ic, int nclicks)
{
  vpDisplay::display(Ic);
  if (nclicks >= 1)
  {
    //draw 1st click point
    vpDisplay::displayCross(Ic, clicks_[0], 10, vpColor::red, 3);
  }
  if (nclicks >= 2)
  {
    //draw 2nd point
    vpDisplay::displayCross(Ic, clicks_[1], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[0], clicks_[1], vpColor::green, 3);
  }
  if (nclicks >= 3)
  {
    //draw 3rd point
    vpDisplay::displayCross(Ic, clicks_[2], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[1], clicks_[2], vpColor::green, 3);
  }
  if (nclicks >= 4)
  {
    //draw 4th point
    vpDisplay::displayCross(Ic, clicks_[3], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[1], clicks_[3], vpColor::green, 3);
  }
  if (nclicks == 5)
  {
    //draw 5th point
    vpDisplay::displayCross(Ic, clicks_[4], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[3], clicks_[4], vpColor::green, 3);
    vpDisplay::displayLine(Ic, clicks_[4], clicks_[0], vpColor::green, 3);
  }
}

void PoseEstimation::display6Clicks(vpImage<vpRGBa> & Ic, int nclicks)
{
  vpDisplay::display(Ic);
  if (nclicks >= 1)
  {
    //draw 1st click point
    vpDisplay::displayCross(Ic, clicks_[0], 10, vpColor::red, 3);
  }
  if (nclicks >= 2)
  {
    //draw 2nd point
    vpDisplay::displayCross(Ic, clicks_[1], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[0], clicks_[1], vpColor::green, 3);
  }
  if (nclicks >= 3)
  {
    //draw 3rd point
    vpDisplay::displayCross(Ic, clicks_[2], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[1], clicks_[2], vpColor::green, 3);
  }
  if (nclicks >= 4)
  {
    //draw 4th point
    vpDisplay::displayCross(Ic, clicks_[3], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[1], clicks_[3], vpColor::green, 3);
  }
  if (nclicks >= 5)
  {
    //draw 5th point
    vpDisplay::displayCross(Ic, clicks_[4], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[3], clicks_[4], vpColor::green, 3);
  }
  if (nclicks == 6)
  {
    //draw 6th point
    vpDisplay::displayCross(Ic, clicks_[5], 10, vpColor::red, 3);
    vpDisplay::displayLine(Ic, clicks_[3], clicks_[5], vpColor::green, 3);
    vpDisplay::displayLine(Ic, clicks_[5], clicks_[0], vpColor::green, 3);
  }
}

