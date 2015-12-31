//
//
//
//
//

#include <array>

#include "FloorSubstractorCCS.h"
#include <algorithms/segmentation/color_clustering/types/ccsCreation.h>

using namespace cv;
using namespace std;
using namespace BOViL;

//---------------------------------------------------------------------------------------------------------------------
bool FloorSubstractorCCS::train(const std::vector<cv::Mat>& _images) {
	std::vector<array<double, 3>> means{}, vars{};

	// Calculate mean and standard deviation of all patches of images.
	for (Mat image:_images) {
		Mat usedImage;
		cvtColor(image, usedImage, CV_RGB2HSV);
		array<double, 3> mean{}, var{};
		
		double nSamples = usedImage.rows*usedImage.cols;
		
		// Calculate mean of patch
		for (int i = 0; i < usedImage.rows; i++) {
			for (int j = 0; j < usedImage.cols; j++) {
				Vec3b vals = usedImage.at<Vec3b>(i, j);
				mean[0] += vals[0];
				mean[1] += vals[1];
				mean[2] += vals[2];
			}
		}

		mean[0] /= nSamples;
		mean[1] /= nSamples;
		mean[2] /= nSamples;

		for (int i = 0; i < usedImage.rows; i++) {
			for (int j = 0; j < usedImage.cols; j++) {
				auto vals = usedImage.at<Vec3b>(i, j);
				var[0] += pow(vals[0] - mean[0],2);
				var[1] += pow(vals[1] - mean[1],2);
				var[2] += pow(vals[2] - mean[2],2);
			}
		}

		var[0] = var[0]/nSamples;
		var[1] = var[1]/nSamples;
		var[2] = var[2]/nSamples;

		means.push_back(mean);
		vars.push_back(var);

	}

	// Remove outliers
	/* Not done yet */

	// Calculate final mean.
	array<double, 3> meanF{}, devF{};
	double totalSize = 0;
	for (unsigned i = 0; i < means.size(); i++) {
		double size = _images[i].rows*_images[i].cols;
		totalSize += size;

		meanF[0] += means[i][0] * size;
		meanF[1] += means[i][1] * size;
		meanF[2] += means[i][2] * size;
	}

	meanF[0] /= totalSize;
	meanF[1] /= totalSize;
	meanF[2] /= totalSize;


	std::vector<array<double, 3>> ESSs, TGSSs;
	for (unsigned i = 0; i < means.size(); i++) {
		double size = _images[i].rows*_images[i].cols;

		ESSs.push_back({ vars[i][0] * (size - 1), vars[i][1] * (size - 1), vars[i][2] * (size - 1) });
		TGSSs.push_back({ pow(means[i][0] - meanF[0],2)*size ,pow(means[i][1] - meanF[1],2)*size ,pow(means[i][2] - meanF[2],2)*size });
	}

	array<double, 3> ESS{}, TGSS{};
	for (unsigned i = 0; i < ESS.size(); i++) {
		ESS[0] += ESSs[i][0];
		ESS[1] += ESSs[i][1];
		ESS[2] += ESSs[i][2];
		TGSS[0] += TGSSs[i][0];
		TGSS[1] += TGSSs[i][1];
		TGSS[2] += TGSSs[i][2];
	}

	devF[0]	= (ESS[0] + TGSS[0])/(totalSize - 1);
	devF[1] = (ESS[1] + TGSS[1])/(totalSize - 1);
	devF[2] = (ESS[2] + TGSS[2])/(totalSize - 1);


	std::cout << "-> FLOORSUBSTRACTOR: channel H, mean: " << meanF[0] << " , dev: " << devF[0] << endl;
	std::cout << "-> FLOORSUBSTRACTOR: channel S, mean: " << meanF[1] << " , dev: " << devF[1] << endl;
	std::cout << "-> FLOORSUBSTRACTOR: channel V, mean: " << meanF[2] << " , dev: " << devF[2] << endl;

	int minH, maxH, minS, maxS,  minV, maxV;

	minH = (minH = meanF[0] - devF[0]) < 0 ? 0 : minH;
	minS = (minS = meanF[1] - devF[1]) < 0 ? 0 : minS;
	minV = (minV = meanF[2] - devF[2]) < 0 ? 0 : minV;
	maxH = (maxH = meanF[0] + devF[0]) >180 ? 180 : maxH;
	maxS = (maxS = meanF[1] + devF[1]) >255 ? 255 : maxS;
	maxV = (maxV = meanF[2] + devF[2]) >255 ? 255 : maxV;


	mClusterizer = createSingleClusteredSpace(minH, maxH, minS, maxS, minV, maxV, 180, 255, 255, 36);

	// Evaluate clusterizer
	double nSamples, score;
	for (Mat image : _images) {
		Mat out, mask;
		substract(image, out, mask);

		for (int i = 0; i < mask.rows; i++) {
			for (int j = 0; j < mask.cols; j++) {
				if (mask.at<uchar>(i, j) == 0) {
					score++;
				}
				nSamples++;
			}
		}

		//hconcat(image, out, image);
		//imshow("display", image);
		//waitKey();
	}
	score /= nSamples;
	cout << "-> FLOORSUBSTRACTOR: Learning score :" << score << endl;

	if (score > 0.8) {
		mIsTrained = true;
	}
	else {
		mIsTrained = false;
	}

	return mIsTrained;
}


//---------------------------------------------------------------------------------------------------------------------
bool FloorSubstractorCCS::substract(const cv::Mat & _in, cv::Mat & _out, cv::Mat &_mask) {
	_in.copyTo(_mask);
	
	blur(_mask, _mask, Size(5, 5));
	cv::cvtColor(_mask, _mask, CV_BGR2HSV);
	std::vector<ImageObject> objects;
	algorithms::ColorClustering<unsigned char>(_mask.data, _mask.cols, _mask.rows, 10, objects, *mClusterizer);
	cv::cvtColor(_mask, _mask, CV_HSV2BGR);

	cv::threshold(_mask, _mask, 50, 255, 1);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * 5 + 1, 2 * 5 + 1), Point(5, 5));
	dilate(_mask, _mask, element);

	bitwise_and(_in, _mask, _out);
	return true;
}