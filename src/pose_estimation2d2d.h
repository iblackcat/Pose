#pragma once
#include "utils.h"

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace mf {

class PoseEstimation2d2d
{
public:
	PoseEstimation2d2d();
	~PoseEstimation2d2d();

	CameraPose pose_estimation2d2d(u32 *image1, u32 *image2);
	void find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2, 
							std::vector<cv::KeyPoint>& keypoints_1,
							std::vector<cv::KeyPoint>& keypoints_2,
							std::vector<cv::DMatch>& matches);
};

} // namespace mf;
