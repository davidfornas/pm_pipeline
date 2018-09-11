/*
 */
#include <pm_tools/average.h>
#include <pm_tools/visp_tools.h>

AverageFloat32::AverageFloat32(ros::NodeHandle & nh, const char * in, const char * mean, const char * stdDev) : nh_(nh), count_(0) {
    averagePublisher = nh.advertise<std_msgs::Float32>(mean, 10);
	  stdDevPublisher= nh.advertise<std_msgs::Float32>(stdDev, 10);
    averageSubscriber = nh.subscribe<std_msgs::Float32>(in, 1, &AverageFloat32::msgCallback, this);
}


void AverageFloat32::msgCallback(const std_msgs::Float32ConstPtr& msg) {

	if (count_ == 0) {
		count_++;
		average_ = *msg;
		averageOfSqares_ = *msg;
		averageOfSqares_.data = average_.data * average_.data;
		stdDev_ = *msg;
		stdDev_.data = 0;
	} else {
		average_.data = (average_.data * count_ + msg->data) / (count_ + 1);
		averageOfSqares_.data = (averageOfSqares_.data * count_ + msg->data * msg->data) / (count_ + 1);
		stdDev_.data = sqrt( averageOfSqares_.data - average_.data * average_.data );
		count_++;
	}
	averagePublisher.publish(average_);
	stdDevPublisher.publish(stdDev_);
}

AverageFloat32MultiArray::AverageFloat32MultiArray(ros::NodeHandle & nh, const char * in, const char * mean, const char * stdDev) : nh_(nh), count_(0) {
	averagePublisher = nh.advertise<std_msgs::Float32MultiArray>(mean, 10);
	stdDevPublisher= nh.advertise<std_msgs::Float32MultiArray>(stdDev, 10);
	averageSubscriber = nh.subscribe<std_msgs::Float32MultiArray>(in, 1, &AverageFloat32MultiArray::msgCallback, this);
}


void AverageFloat32MultiArray::msgCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
	if (count_ == 0) {
		count_++;
		averages_ = *msg;

		averageOfSqares_ = *msg;
		for(int i = 0; i < msg->data.size(); i++) {
			averageOfSqares_.data[i] = averages_.data[i] * averages_.data[i];
		}
		stdDevs_ = *msg;
		for(int i = 0; i < msg->data.size(); i++) {
			stdDevs_.data[i] = 0;
		}

	} else {
		for(int i = 0; i < msg->data.size(); i++) {
			averages_.data[i] = (averages_.data[i] * count_ + msg->data[i]) / (count_ + 1);
		}
		for(int i = 0; i < msg->data.size(); i++) {
			averageOfSqares_.data[i] = (averageOfSqares_.data[i] * count_ + msg->data[i] * msg->data[i]) / (count_ + 1);
		}
		for(int i = 0; i < msg->data.size(); i++) {
			stdDevs_.data[i] = sqrt( averageOfSqares_.data[i] - averages_.data[i] * averages_.data[i] );
		}
		count_++;
	}
	stdDevPublisher.publish(stdDevs_);
	averagePublisher.publish(averages_);
}

AveragePose::AveragePose(ros::NodeHandle & nh, const char * in, const char * mean, const char * stdDev) : nh_(nh), count_(0) {
	averagePublisher = nh.advertise<geometry_msgs::Pose>(mean, 10);
	stdDevPublisher= nh.advertise<std_msgs::Float32MultiArray>(stdDev, 10);
	averageSubscriber = nh.subscribe<geometry_msgs::Pose>(in, 1, &AveragePose::msgCallback, this);
}


void AveragePose::msgCallback(const geometry_msgs::PoseConstPtr& msg) {
	if (count_ == 0) {
		count_++;
		averagePose_ = *msg;

		double r, p, y;
		VispTools::rpyFromQuaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w, r, p, y);
		std_msgs::Float64MultiArray pose;
		pose.data.push_back(msg->position.x);
		pose.data.push_back(msg->position.y);
		pose.data.push_back(msg->position.z);
		pose.data.push_back(r);
		pose.data.push_back(p);
		pose.data.push_back(y);
		poses_.push_back(pose);

		for(int i = 0; i < 6; i++) {
			stdDevs_.push_back(0);
		}

	} else {
		vpHomogeneousMatrix result;
		result = VispTools::weightedAverage(VispTools::vispHomogFromGeometryPose(averagePose_), count_,
																				VispTools::vispHomogFromGeometryPose(*msg));

		averagePose_ = VispTools::geometryPoseFromVispHomog(result);

		double r, p, y;
		VispTools::rpyFromQuaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w, r, p, y);
		std_msgs::Float64MultiArray pose;
		pose.data.push_back(msg->position.x);
		pose.data.push_back(msg->position.y);
		pose.data.push_back(msg->position.z);
		pose.data.push_back(r);
		pose.data.push_back(p);
		pose.data.push_back(y);
		poses_.push_back(pose);

		computeAverage();

		count_++;
	}
	std_msgs::Float32MultiArray msgx;
	for(int i=0; i<stdDevs_.size();i++){
		msgx.data.push_back(stdDevs_[i]);
	}
	stdDevPublisher.publish(msgx);
	averagePublisher.publish(averagePose_);
}

void AveragePose::computeAverage()
{
	double sums[6], means[6];

	for(int dim=0; dim < 6; dim++) {

		sums[dim] = 0.0;
		for (int i = 0; i < poses_.size(); ++i) {
			sums[dim] += poses_[i].data[dim];
		}
		means[dim] = sums[dim] / poses_.size();

		stdDevs_[dim] = 0.0;
		for (int i = 0; i < poses_.size(); ++i)
			stdDevs_[dim] += (poses_[i].data[dim] - means[dim]) *  (poses_[i].data[dim] - means[dim]);

		stdDevs_[dim] = sqrt( stdDevs_[dim] / poses_.size());
	}
}


