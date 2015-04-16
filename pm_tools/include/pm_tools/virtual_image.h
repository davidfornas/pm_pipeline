/** Imported from MAR (Mario Prats) **/

#ifndef VIRTUALIMAGE_H
#define VIRTUALIMAGE_H

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpCameraParameters.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <boost/shared_ptr.hpp>

/** Framegrabber for the simulator camera
*/
class VirtualImage: public vpFrameGrabber
{
	std::string image_topic, info_topic;
	image_transport::ImageTransport *it;
	image_transport::Subscriber image_sub;
	ros::Subscriber image_info_sub;
	int subsample_;
	bool ready_;	//true if images have been acquired

public:
    //image data
    unsigned int width, height;
    vpImage<vpRGBa> image;
    vpCameraParameters K;

    /** Constructor from the server and port where the simulator is listening */
    VirtualImage(ros::NodeHandle &nh, std::string image_topic, std::string info_topic, int subsample=1);

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

    virtual void open(vpImage<unsigned char> &I) {} ///< Initialize image (set size)
    virtual void open(vpImage<vpRGBa> &I) {}	///< Initialize image (set size)

    virtual void acquire(vpImage<unsigned char> &I);	///< B/W image capture
    virtual void acquire(vpImage<vpRGBa> &I);		///< Color image capture

    virtual void close();

    bool ready() {return ready_;}

    ~VirtualImage();
};
typedef boost::shared_ptr<VirtualImage> VirtualImagePtr;

#endif
