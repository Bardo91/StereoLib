//
//
//
//
//


#include "ParallelFeatureMatcher.h"

using namespace std;
using namespace cv;

//---------------------------------------------------------------------------------------------------------------------
ParallelFeatureMatcher::ParallelFeatureMatcher(const Mat &_frame1, const Mat &_frame2,
	const vector<KeyPoint> &_kps,
	const vector<Vec3f> &_epis,
	const pair<int, int> &_disparityRange,
	const int &_squareSize, 
	const double &_maxTemplateScore,
	vector<vector<KeyPoint>> &_points1, vector<vector<KeyPoint>> &_points2,
	Rect _vl, Rect _vr) :	frame1(_frame1), frame2(_frame2), kps(_kps), epis(_epis), disparityRange(_disparityRange), 
	squareSize(_squareSize), maxTemplateScore(_maxTemplateScore), points1(_points1), points2(_points2), validLeft(_vl), validRight(_vr) {};

//---------------------------------------------------------------------------------------------------------------------
void ParallelFeatureMatcher::operator()(const cv::Range& range) const{
	int ini = epis.size()*range.start/8;
	int end = epis.size()*(range.start+1)/8;

	for (int i = ini; i < end; i++){
		//std::cout << "Computing: " << i << std::endl;
		if(!validLeft.contains(kps[i].pt))	// Ignore keypoint if it is outside valid region.
			continue;

		// Calculate matching and add points
		Point2i matchedPoint = findMatch(frame1, frame2, kps[i].pt, epis[i], disparityRange, squareSize);
		if(!validRight.contains(matchedPoint))
			continue;

		points1[range.start].push_back(kps[i]);
		KeyPoint kp;
		kp.pt = matchedPoint;
		points2[range.start].push_back(kp);
	}
}

//---------------------------------------------------------------------------------------------------------------------
// Private interface.

cv::Point2i ParallelFeatureMatcher::findMatch(const Mat &_frame1, const Mat &_frame2, const Point2i &_point, const Vec3f &_epiline, const std::pair<int, int> _disparityRange, const int _squareSize) const {
	// Compute template matching over the epipolar line
	// Get template from first image.
	Mat imgTemplate = _frame1(Rect(	Point2i(_point.x - _squareSize/2, _point.y - _squareSize/2),
		Point2i(_point.x + _squareSize/2, _point.y + _squareSize/2)));
	const double cMaxDiff = _squareSize*_squareSize*255*255;
	double minVal = cMaxDiff;
	Point2i minLoc(-1,-1), p1;
	Mat subImage;

	int minX = _point.x - _disparityRange.second;
	minX = minX < 0 ? 0 : minX;
	int maxX = _point.x - _disparityRange.first;
	maxX = maxX < 0 ? 0 : maxX;
	maxX = maxX + _squareSize/2 + 1 > _frame1.cols ? maxX - (_squareSize/2 + 1) : maxX;
	for(int i = minX ; i < maxX ;i++){
		// Compute point over epiline
		p1.x = i;
		p1.y = int(-1*(_epiline[2] + _epiline[0] * i)/_epiline[1]);

		Point2i sp1, sp2;
		sp1 = p1 - Point2i(_squareSize/2, _squareSize/2);
		sp2 = p1 + Point2i(_squareSize/2, _squareSize/2);
		Rect imageBound(0,0,_frame1.cols, _frame2.rows);
		if(!imageBound.contains(sp1) || !imageBound.contains(sp2))
			continue;
		// Get subimage from image 2;
		subImage = _frame2(Rect(sp1, sp2));
		// Compute correlation

		double val = 0;
		for(int row = 0; row < subImage.rows; row++){
			for(int col = 0; col < subImage.cols; col++){
				val += pow(double(subImage.at<uchar>(row, col)) - double(imgTemplate.at<uchar>(row, col)),2);
			}
		}
		if(val < minVal){
			minVal = val;
			minLoc = p1;
		}
	}

	if ((minVal/cMaxDiff) < maxTemplateScore) {
		return minLoc;
	}
	else {
		return Point2i(-1,-1);
	}

}
