/*
 * object_segmentation.h
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef OBJECTSEGMENTATION_H_
#define OBJECTSEGMENTATION_H_

/** Description
 */
class ObjectSegmentation {


    public:
	/** Constructor.
	 * @param cloud, background_remover, segmentator, hypothesis_generator
	 * */
        ObjectSegmentation(){

	}

	void process();

	~ObjectSegmentation() {}
};

#endif /* OBJECTSEGMENTATION_H_ */
