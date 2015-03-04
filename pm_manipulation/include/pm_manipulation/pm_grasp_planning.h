/*
 * PCGraspPlanning.h
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef PMGRASPPLANNING_H_
#define PMGRASPPLANNING_H_

#include <pm_perception/background_removal.h>
#include <pm_perception/object_segmentation.h>
#include <pm_manipulation/hypothesis_generation.h>
#include <pcl/io/pcd_io.h>

/** Description
 */
class PMGraspPlanning {


    public:
	/** Constructor.
	 * @param cloud, background_remover, segmentator, hypothesis_generator
	 * */
	PMGraspPlanning(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, BackgroundRemoval background_remover,
	                ObjectSegmentation segmentator, HypothesisGeneration hypothesis_generation){

	}

	void proccessScene();

	/** Description */
	void getGraspHypothesis();


	~PMGraspPlanning() {}
};

#endif /* PMGRASPPLANNING_H_ */
