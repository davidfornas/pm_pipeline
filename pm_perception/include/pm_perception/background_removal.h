/*
 * background_removal.h
 *
 *  Created on: 03/03/2014
 *      Author: dfornas
 */

#ifndef BACKGROUNDREMOVAL_H_
#define BACKGROUNDREMOVAL_H_

/** Description
 */
class BackgroundRemoval {


    public:
	/** Constructor.
	 * @param cloud, background_remover, segmentator, hypothesis_generator
	 * */
        BackgroundRemoval(){

	}

	void process();

	~BackgroundRemoval() {}
};

#endif /* BACKGROUNDREMOVAL_H_ */
