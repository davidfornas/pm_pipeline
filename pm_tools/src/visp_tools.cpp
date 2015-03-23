/*
 *  VispTools
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#include <pm_tools/visp_tools.h>
//#include <kdl/frames.hpp>
#include <tf/transform_broadcaster.h>
#include <visp/vpHomogeneousMatrix.h>
#include <stdlib.h>

std::ostream& operator<<(std::ostream& out, Frame& x ) 
{
	  out << "Pose origin  [x: " << x.pose.getOrigin().x() << " y: " << x.pose.getOrigin().y() << " z: " << x.pose.getOrigin().z() << "] "
	  << "Pose rotation [x:" << x.pose.getRotation().x() << " y:" << x.pose.getRotation().y() << " z:" << x.pose.getRotation().z() << " w:" << x.pose.getRotation().w() << "] "
	  << std::endl << "Parent: " << x.parent << " Child: " << x.child << std::endl;
	  return out;
}


tf::Transform VispTools::tfTransFromVispHomog( vpHomogeneousMatrix sMs){
	
	  tf::Vector3 translation(sMs[0][3],sMs[1][3],sMs[2][3]);
      
      vpQuaternionVector q;
      sMs.extract(q);
	  tf::Quaternion rotation( q.x(), q.y(), q.z(), q.w());
	  
	  tf::Transform pose(rotation, translation);	  
	  return pose;
}

vpHomogeneousMatrix VispTools::vispHomogFromTfTransform( tf::Transform transform ){
	vpTranslationVector t(transform.getOrigin()[0], transform.getOrigin()[1], transform.getOrigin()[2]);
	vpQuaternionVector q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w() ); 
	return vpHomogeneousMatrix(t, q); 
}

vpHomogeneousMatrix VispTools::vispHomogFromXyzrpy( double x, double y, double z, double r, double p, double yaw ){
	return vpHomogeneousMatrix(vpTranslationVector(x, y, z), vpRotationMatrix(vpRxyzVector(r, p, yaw)) );
	
}


VispToTF::VispToTF( vpHomogeneousMatrix sMs, std::string parent, std::string child){
	  broadcaster_ = new tf::TransformBroadcaster();
	  addTransform(sMs, parent, child, "0");
}

VispToTF::VispToTF( tf::Transform sMs, std::string parent, std::string child){
	  broadcaster_ = new tf::TransformBroadcaster();
	  addTransform(sMs, parent, child, "0");
}

VispToTF::VispToTF( ){
	  broadcaster_ = new tf::TransformBroadcaster();
}

void VispToTF::addTransform( vpHomogeneousMatrix sMs, std::string parent, std::string child, std::string id){

	  tf::Transform pose=VispTools::tfTransFromVispHomog(sMs);
      addTransform( pose, parent, child, id);
	  
}

void VispToTF::addTransform( tf::Transform sMs, std::string parent, std::string child, std::string id){

	  //TODO: It's possible to do further checks.
      if(frames_.count(id)>0){
		  std::cerr << "ID [" << id << "] already in use. It won't be added." << std::endl;
		  return;		  
	  }
	  // Check for transform duplicates, traverse through the map so keep it small.
	  bool duplicate=false;
	  std::string old_id;
	  for( std::map<std::string, Frame>::iterator ii=frames_.begin(); ii!=frames_.end(); ++ii)
	  {
			if( (*ii).second.parent == parent &&  (*ii).second.child == child){
				duplicate=true;
				old_id=(*ii).first;
			}	
	  }
	  //Valid tree structure it's not checked. TF does this check, though.
	  
	  if(duplicate){
		  std::cerr << "Frame from " << child << " to " << parent << " already specified. Reset it with resetTransform(sMs, " << old_id << " )." << std::endl;
		  return;		  
	  }
	  
	  	  
	  Frame f; 
	  f.pose=sMs;
	  f.parent=parent;
	  f.child=child;
	  
	  frames_[id]=f;
	  
}

void VispToTF::resetTransform( vpHomogeneousMatrix sMs, std::string id){
	  
	  tf::Transform pose=VispTools::tfTransFromVispHomog(sMs);
	  resetTransform( pose, id);
}

void VispToTF::resetTransform( tf::Transform sMs, std::string id){
	  
	  if(frames_.count(id)<1){
		  std::cerr << "Can't reset this item. ID [" << id << "] not found." << std::endl;
		  return;		  
	  }  
	  frames_[id].pose=sMs;
	  
}

void VispToTF::removeTransform( std::string id){
	  if(frames_.count(id)<1){
		  std::cerr << "Can't delete this item. ID [" << id << "] not found." << std::endl;
		  return;		  
	  }
	  frames_.erase(id);
}

void VispToTF::publish(){

	for( std::map<std::string, Frame>::iterator ii=frames_.begin(); ii!=frames_.end(); ++ii)
		{
			tf::StampedTransform transform((*ii).second.pose, ros::Time::now(), (*ii).second.parent, (*ii).second.child);
			broadcaster_->sendTransform(transform);
		}

}

void VispToTF::print(){

	for( std::map<std::string, Frame>::iterator ii=frames_.begin(); ii!=frames_.end(); ++ii)
		{
			std::cout << "ID string: " << (*ii).first  << std::endl << 
			"Frame -> " << (*ii).second << std::endl;			
		}

}


