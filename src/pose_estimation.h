#pragma once
#include "utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace mf {

class PoseEstimation
{
public:
	PoseEstimation();
	~PoseEstimation();

	void find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2,
		std::vector<cv::KeyPoint>& keypoints_1,
		std::vector<cv::KeyPoint>& keypoints_2,
		std::vector<cv::DMatch>& matches);

	void visualization(const cv::Mat &img_1, const cv::Mat &img_2,
		const std::vector<cv::KeyPoint> &keypoints_1,
		const std::vector<cv::KeyPoint> &keypoints_2,
		const std::vector<cv::DMatch> &matches);


private:
};

} //namespace mf
