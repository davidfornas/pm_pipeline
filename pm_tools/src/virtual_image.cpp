#include <pm_tools/virtual_image.h>
#include <visp/vpImageConvert.h>

VirtualImage::VirtualImage(ros::NodeHandle &nh, std::string image_topic, std::string info_topic, int subsample) {
	it=new image_transport::ImageTransport(nh);

	image_sub=it->subscribe(image_topic, 1, &VirtualImage::imageCallback, this);	
	image_info_sub=nh.subscribe<sensor_msgs::CameraInfo>(info_topic, 1, &VirtualImage::imageInfoCallback, this);	
	ready_=false;
	subsample_=subsample;
}

void VirtualImage::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	//Receive image, convert to ViSP, store in a class attribute
	this->width=msg->width/subsample_;
	this->height=msg->height/subsample_;

	if (image.getCols()!=width || image.getRows()!=height)
		image.resize(height, width);
	
	if (msg->encoding==std::string("bgr8")) {
		for (unsigned int r=0;r<height; r++)
			for (unsigned int c=0; c<width; c++) {
				int index=(width*subsample_)*r*subsample_*3+c*subsample_*3;
				image[r][c].R=msg->data[index+2];
				image[r][c].G=msg->data[index+1];
				image[r][c].B=msg->data[index];
			}
	} else if (msg->encoding==std::string("mono8")) {
		for (unsigned int r=0;r<height; r++)
			for (unsigned int c=0; c<width; c++) {
				int index=(width*subsample_)*r+c*subsample_;
				image[r][c].R=msg->data[index];
				image[r][c].G=msg->data[index];
				image[r][c].B=msg->data[index];
			}
	} else {
		//Assume rgb8
		for (unsigned int r=0;r<height; r++)
			for (unsigned int c=0; c<width; c++) {
				int index=(width*subsample_)*r*subsample_*3+c*subsample_*3;
				image[r][c].R=msg->data[index];
				image[r][c].G=msg->data[index+1];
				image[r][c].B=msg->data[index+2];
			}
	}

	ready_=true;
}


void VirtualImage::imageInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
	//Receive image info, store in a class attribute
	int binx=(msg->binning_x == 0) ? 1 : msg->binning_x;
	int biny=(msg->binning_y ==0) ? 1 : msg->binning_y;
	K.initPersProjWithoutDistortion (msg->P[0]/(binx*subsample_), msg->P[5]/(biny*subsample_), msg->P[2]/(binx*subsample_), msg->P[6]/(biny*subsample_));
}


void VirtualImage::acquire(vpImage<unsigned char> &I) {
	vpImageConvert::convert(image,I);
}

void VirtualImage::acquire(vpImage<vpRGBa> &I) {
	I=image;
}

void VirtualImage::close() {}

VirtualImage::~VirtualImage() {
	if (it!=NULL) delete it;
}

