//
//
//
//
//


#ifndef STEREOLIB_FLOORSUBSTRACTORCCS_H_
#define STEREOLIB_FLOORSUBSTRACTORCCS_H_

#include "FloorSubstractor.h"

class FloorSubstractorCCS: public FloorSubstractor {
public:
	bool train(const std::vector<cv::Mat> &_images);

	bool substract(const cv::Mat &_in, cv::Mat &_out);

};

#endif	//	 STEREOLIB_FLOORSUBSTRACTORCCS_H_
