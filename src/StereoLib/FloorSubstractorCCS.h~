//
//
//
//
//


#ifndef STEREOLIB_FLOORSUBSTRACTORCCS_H_
#define STEREOLIB_FLOORSUBSTRACTORCCS_H_

#include "FloorSubstractor.h"
#include <algorithms/segmentation/color_clustering/ColorClustering.h>

class FloorSubstractorCCS: public FloorSubstractor {
public:
	bool train(const std::vector<cv::Mat> &_images);

	bool substract(const cv::Mat &_in, cv::Mat &_out, cv::Mat &_mask = cv::Mat());

private:
	BOViL::ColorClusterSpace *mClusterizer;
};

#endif	//	 STEREOLIB_FLOORSUBSTRACTORCCS_H_
