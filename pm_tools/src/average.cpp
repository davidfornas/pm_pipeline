/*
 */
#include <pm_tools/average.h>
#include <pm_tools/visp_tools.h>

AverageFloat32::AverageFloat32(ros::NodeHandle & nh, const char * in, const char * out) : nh_(nh), count_(0) {
    averagePublisher = nh.advertise<std_msgs::Float32>(out, 10);
    averageSubscriber = nh.subscribe<std_msgs::Float32>(in, 1, &AverageFloat32::msgCallback, this);
}


void AverageFloat32::msgCallback(const std_msgs::Float32ConstPtr& msg) {

	if (count_ == 0) {
		count_++;
		average_ = *msg;
	} else {
		average_.data = (average_.data * count_ + msg->data) / (count_ + 1);
		count_++;
	}
	averagePublisher.publish(average_);
}

AverageFloat32MultiArray::AverageFloat32MultiArray(ros::NodeHandle & nh, const char * in, const char * out) : nh_(nh), count_(0) {
	averagePublisher = nh.advertise<std_msgs::Float32MultiArray>(out, 10);
	averageSubscriber = nh.subscribe<std_msgs::Float32MultiArray>(in, 1, &AverageFloat32MultiArray::msgCallback, this);
}


void AverageFloat32MultiArray::msgCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
	if (count_ == 0) {
		count_++;
		averages_ = *msg;
	} else {
		for(int i = 0; i < msg->data.size(); i++) {
			averages_.data[i] = (averages_.data[i] * count_ + msg->data[i]) / (count_ + 1);
		}
		count_++;
	}
	averagePublisher.publish(averages_);
}

AveragePose::AveragePose(ros::NodeHandle & nh, const char * in, const char * out) : nh_(nh), count_(0) {
	averagePublisher = nh.advertise<geometry_msgs::Pose>(out, 10);
	averageSubscriber = nh.subscribe<geometry_msgs::Pose>(in, 1, &AveragePose::msgCallback, this);
}


void AveragePose::msgCallback(const geometry_msgs::PoseConstPtr& msg) {
	if (count_ == 0) {
		count_++;
		averagePose_ = *msg;
	} else {
		vpHomogeneousMatrix result;
		result = VispTools::weightedAverage(VispTools::vispHomogFromGeometryPose(averagePose_), count_,
																				VispTools::vispHomogFromGeometryPose(*msg));
		averagePose_ = VispTools::geometryPoseFromVispHomog(result);
		count_++;
	}
	averagePublisher.publish(averagePose_);

}
