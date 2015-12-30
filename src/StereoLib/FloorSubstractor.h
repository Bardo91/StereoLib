//
//
//
//
//


#ifndef STEREOLIB_FLOORSUBSTRACTOR_H_
#define STEREOLIB_FLOORSUBSTRACTOR_H_

#include <opencv2/opencv.hpp>

class FloorSubstractor {
public:
	virtual bool train			(const std::vector<cv::Mat> &_images)	= 0;

	virtual bool substract		(const cv::Mat &_in, cv::Mat &_out)		= 0;

	bool isTrained() { return mIsTrained; };

protected:
	bool mIsTrained = false;
};

#endif	//	 STEREOLIB_FLOORSUBSTRACTOR_H_
