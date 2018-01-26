#include <pm_manipulation/joint_offset.h>

void JointOffset::readJointsCallback(const sensor_msgs::JointState::ConstPtr& m){
	if(bMc_init){
		sensor_msgs::JointState joints_fixed;
		for(uint i=0; i<m->name.size(); i++){
			joints_fixed.name.push_back(m->name[i]);
			joints_fixed.position.push_back(m->position[i]+offset_[i]);
			joints_fixed.effort.push_back(m->effort[i]);
		}
		joints_fixed.header.stamp=ros::Time::now();
		joint_state_pub.publish(joints_fixed);
	}
}

void JointOffset::markerCallback(const geometry_msgs::PoseStamped::ConstPtr &m){

	cMm_tf.setOrigin(tf::Vector3(m->pose.position.x, m->pose.position.y, m->pose.position.z));
	cMm_tf.setRotation(tf::Quaternion(m->pose.orientation.x, m->pose.orientation.y, m->pose.orientation.z, m->pose.orientation.w));
	if(cMm_found && bMc_init){
		vpHomogeneousMatrix cMe=markerToEndEffector(cMm_tf);
		vpHomogeneousMatrix bMe=bMc*cMe;
		vpColVector desired_joints, current_joints;
		ros::spinOnce();
		robot->getJointValues(current_joints);
		desired_joints=robot->armIK(bMe);
		if(desired_joints[0]>-1.57 && desired_joints[0]<2.1195 && desired_joints[1]>0 && desired_joints[1]<1.58665 && desired_joints[2]>0 && desired_joints[2]<2.15294){
			for(uint i=0; i<3; i++){
				offset_[i]=desired_joints[i]-current_joints[i];
			}
		}
		else{
			std::cout<<"inverse Kinematic not solve: "<<desired_joints<<std::endl;
		}
	}
	else{
		cMm_found=true;
	}

}

/*void JointOffset::markerCallback(const ar_pose::ARMarkers::ConstPtr &m){
	if(m->markers.size()!=0){
		cMm_tf.setOrigin(tf::Vector3(m->markers[0].pose.pose.position.x, m->markers[0].pose.pose.position.y, m->markers[0].pose.pose.position.z));
		cMm_tf.setRotation(tf::Quaternion(m->markers[0].pose.pose.orientation.x, m->markers[0].pose.pose.orientation.y, m->markers[0].pose.pose.orientation.z, m->markers[0].pose.pose.orientation.w));
		if(cMm_found && bMc_init){
			vpHomogeneousMatrix cMe=markerToEndEffector(cMm_tf);
			vpHomogeneousMatrix bMe=bMc*cMe;
			vpColVector desired_joints, current_joints;
			ros::spinOnce();
			robot->getJointValues(current_joints);
			desired_joints=robot->armIK(bMe);
			if(desired_joints[0]>-1.57 && desired_joints[0]<2.1195 && desired_joints[1]>0 && desired_joints[1]<1.58665 && desired_joints[2]>0 && desired_joints[2]<2.15294){
				for(uint i=0; i<3; i++){
					offset_[i]=desired_joints[i]-current_joints[i];
				}
			}
			else{
				std::cout<<"inverse Kinematic not solve: "<<desired_joints<<std::endl;
			}
		}
		else{
			cMm_found=true;
		}
	}
}*/

JointOffset::JointOffset(ros::NodeHandle& nh, std::string topic_joint_state, std::string topic_command_joint, std::string topic_joint_state_fixed): nh_(nh){
	robot=new ARM5Arm(nh_, topic_joint_state, topic_command_joint);
	joint_state_sub=nh.subscribe<sensor_msgs::JointState>(topic_joint_state, 1, &JointOffset::readJointsCallback, this);
	marker_sub=nh.subscribe<geometry_msgs::PoseStamped>("/marker_filter_node/marker_pose", 1, &JointOffset::markerCallback, this);
	//marker_sub=nh.subscribe<ar_pose::ARMarkers>("/ar_pose_ee", 1, &JointOffset::markerCallback, this);
	joint_state_pub=nh.advertise<sensor_msgs::JointState>(topic_joint_state_fixed,1);
	cMm_found=false;
	bMc_init=false;
	offset_.resize(5);
	offset_=0;
}

vpHomogeneousMatrix JointOffset::markerToEndEffector(tf::Transform cMm_tf){
	//cMm to cMe

	tf::Transform mMw_tf,wMe_tf;

	/*
	//ROBOT SIMULADO
	tf::Transform initRotation(tf::Quaternion(tf::Vector3(-1,0,0),0.3053),tf::Vector3(0,0,0));

	//marker to wrist mMw
		mMw_tf.setOrigin(tf::Vector3(0,0.06643,0.00545));
		mMw_tf.setBasis(tf::Matrix3x3(1, 0, 0,
				0, 1, 0,
				0, 0, 1));


		//wrist rotation
		vpColVector current_joints;
		robot->getJointValues(current_joints);
		tf::Transform wristRotation(tf::Quaternion(tf::Vector3(1,0,0),current_joints[4]),tf::Vector3(0,0,0));

		//wrist to endEffector wMe
		wMe_tf.setOrigin(tf::Vector3(0,-0.105,-0.04));
		wMe_tf.setBasis(tf::Matrix3x3(1, 0, 0,
				0, 1, 0,
				0, 0, 1));

		//Correct orientation of the final frame
		tf::Transform finalRotation(tf::Quaternion(tf::Vector3(1,0,0),1.57),tf::Vector3(0,0,0));
		tf::Transform finalRotation2(tf::Quaternion(tf::Vector3(0,0,1),1.57),tf::Vector3(0,0,0));


		//changes auxiliar
		tf::Transform aux_chan;
		aux_chan.setOrigin(tf::Vector3(0.0,0,0));
		aux_chan.setBasis(tf::Matrix3x3(1, 0, 0,
				0, 1, 0,
				0, 0, 1));*/


	//Robot Real
	tf::Transform initRotation(tf::Quaternion(tf::Vector3(-1,0,0),0.3087),tf::Vector3(0,0,0));
	//marker to wrist mMw
	mMw_tf.setOrigin(tf::Vector3(0,0.06680,0.00864));
	mMw_tf.setBasis(tf::Matrix3x3(1, 0, 0,
			0, 1, 0,
			0, 0, 1));


	//wrist rotation
	vpColVector current_joints;
	robot->getJointValues(current_joints);
	tf::Transform wristRotation(tf::Quaternion(tf::Vector3(1,0,0),current_joints[4]/2),tf::Vector3(0,0,0));

	//wrist to endEffector wMe
	wMe_tf.setOrigin(tf::Vector3(0,-0.10677,-0.03914));
	wMe_tf.setBasis(tf::Matrix3x3(1, 0, 0,
			0, 1, 0,
			0, 0, 1));

	//Correct orientation of the final frame
	tf::Transform finalRotation(tf::Quaternion(tf::Vector3(1,0,0),1.57),tf::Vector3(0,0,0));
	tf::Transform finalRotation2(tf::Quaternion(tf::Vector3(0,0,1),1.57),tf::Vector3(0,0,0));


	//changes auxiliar
	tf::Transform aux_chan;
	aux_chan.setOrigin(tf::Vector3(0.0,0,0));
	aux_chan.setBasis(tf::Matrix3x3(1, 0, 0,
			0, 1, 0,
			0, 0, 1));

	//Camera to endEffector
	tf::Transform cMe_tf=cMm_tf*initRotation*mMw_tf*wristRotation*wMe_tf*finalRotation*finalRotation2;//*aux_chan;
	//tf::StampedTransform cMe_st(cMe_tf, ros::Time::now(), "/stereo_down_optical", "/end_effector");
	//broadcaster.sendTransform(cMe_tf);

	vpHomogeneousMatrix cMe;
	cMe[0][0]=cMe_tf.getBasis()[0][0]; cMe[0][1]=cMe_tf.getBasis()[0][1]; cMe[0][2]=cMe_tf.getBasis()[0][2]; cMe[0][3]=cMe_tf.getOrigin().x();
	cMe[1][0]=cMe_tf.getBasis()[1][0]; cMe[1][1]=cMe_tf.getBasis()[1][1]; cMe[1][2]=cMe_tf.getBasis()[1][2]; cMe[1][3]=cMe_tf.getOrigin().y();
	cMe[2][0]=cMe_tf.getBasis()[2][0]; cMe[2][1]=cMe_tf.getBasis()[2][1]; cMe[2][2]=cMe_tf.getBasis()[2][2]; cMe[2][3]=cMe_tf.getOrigin().z();

	return cMe;
}


int JointOffset::reset_bMc(vpColVector initial_posture){
	vpColVector current_joints;
	robot->getJointValues(current_joints);
	if(initial_posture.size()!=5)
		initial_posture=current_joints;
	while((initial_posture-current_joints).euclideanNorm()>0.02 && ros::ok()){
		robot->setJointVelocity(initial_posture-current_joints);
		ros::spinOnce();
		robot->getJointValues(current_joints);
		std::cout<<(initial_posture-current_joints).euclideanNorm()<<std::endl;
	}
	cMm_found=false;
	ros::Time time;
	time=ros::Time::now();
	while(!cMm_found && (ros::Time::now()-time).toSec()<5){
		try{
			listener.lookupTransform("/stereo_down_optical", "/ee_marker", ros::Time(0), cMm_tf);
			cMm_found=true;
		}
		catch(tf::TransformException & ex){
			std::cerr<<"cMm not found"<<std::endl;
		}
		ros::spinOnce();
	}
	if(!cMm_found){
		std::cerr<<"cMm not found in 5 seconds"<<std::endl;
		return -1;
	}
	vpHomogeneousMatrix cMe, bMe;


	cMe=markerToEndEffector(cMm_tf);
	robot->getPosition(bMe);

	bMc=bMe*cMe.inverse();
	bMc_init=true;

	return 0;
}
