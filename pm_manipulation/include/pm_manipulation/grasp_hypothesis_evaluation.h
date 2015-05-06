/*
 * grasp_hypothesis_evaluation.h To implemet
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef GRASPHYPOTHESISEVALUATION_H_
#define GRASPHYPOTHESISEVALUATION_H_

/** Description
 */
class GraspHypothesisEvaluation {


    public:
	/** Constructor.
	 * @param cloud, background_remover, segmentator, hypothesis_generator
	 * */
        GraspHypothesisEvaluation(){

	}

	void getBestGrasp();

	~GraspHypothesisEvaluation() {}
};

#endif /* GRASPHYPOTHESISEVALUATION_H_ */
