//
//
//
//
//

#include "FloorSubstractorCCS.h"
#include <algorithms/segmentation/color_clustering/ColorClustering.h>
#include <algorithms/segmentation/color_clustering/types/ccsCreation.h>

using namespace cv;
using namespace std;
using namespace BOViL;

//---------------------------------------------------------------------------------------------------------------------
bool FloorSubstractorCCS::train(const std::vector<cv::Mat>& _images) {
	mIsTrained = true;
	return true;
}


//---------------------------------------------------------------------------------------------------------------------
bool FloorSubstractorCCS::substract(const cv::Mat & _in, cv::Mat & _out) {
	Mat mask;
	_in.copyTo(mask);
	
	blur(mask, mask, Size(5, 5));
	cv::cvtColor(mask, mask, CV_BGR2HSV);
	std::vector<ImageObject> objects;
	ColorClusterSpace ccs = createSingleClusteredSpace(0, 180, 0, 20, 50, 110, 180, 255, 255, 36);
	algorithms::ColorClustering<unsigned char>(mask.data, mask.cols, mask.rows, 10, objects, ccs);
	cv::cvtColor(mask, mask, CV_HSV2BGR);

	cv::threshold(mask, mask, 50, 255, 1);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * 5 + 1, 2 * 5 + 1), Point(5, 5));
	dilate(mask, mask, element);

	bitwise_and(_in, mask, _out);
	return true;
}