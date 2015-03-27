/*
 * trajectory_following.cpp
 *
 *  Created on: 24/03/2014
 *      Author: dfornas
 */

#include <pm_manipulation/trajectory_following.h>

void TrajectoryFollowing::moveToNextWaypoint(){

  path_index_++;
  //Oobtain next point
  vpTranslationVector t( path_.poses[path_index_].pose.position.x, path_.poses[path_index_].pose.position.y, path_.poses[path_index_].pose.position.z);
  vpQuaternionVector q(path_.poses[path_index_].pose.orientation.x, path_.poses[path_index_].pose.orientation.y,
                       path_.poses[path_index_].pose.orientation.z, path_.poses[path_index_].pose.orientation.w);
  vpHomogeneousMatrix target(t, q);

  //CALCULAR CINEMATICA
  vpColVector joints(8);
  joints = robot_->vehicleArmIK(target);
  std::cout << "Reachable position vehicle config: " << std::endl << joints << std::endl;





  //AVISAR PUNTO SIGUIENTE

  //ENVIAR ORDENES AL VEHICULO/BRAZO


}

void TrajectoryFollowing::getCurrentWaypointPose(){

}
