#include "pose_estimation.h"
#include "utils.h"

using namespace std;
using namespace cv;

namespace mf {

PoseEstimation::PoseEstimation()
{
}


PoseEstimation::~PoseEstimation()
{
}

void PoseEstimation::visualization(const Mat &img_1, const Mat &img_2,
	const vector<KeyPoint> &keypoints_1,
	const vector<KeyPoint> &keypoints_2,
	const vector<DMatch> &matches) {
	imshow("i1", img_1);
	imshow("i2", img_2);
	//Mat img_match;
	//drawMatches(temp_1, keypoints_1, temp_2, keypoints_2, matches, img_match, Scalar::all(-1), Scalar::all(-1), \
				vector<char>(0), DrawMatchesFlags::DEFAULT);
//imshow("matches", img_match);

	Mat img_show(img_1.rows * 2, img_1.cols, CV_8UC4);
	img_1.copyTo(img_show(Rect(0, 0, img_1.cols, img_1.rows)));
	img_2.copyTo(img_show(Rect(0, img_1.rows, img_1.cols, img_1.rows)));
	for (DMatch m : matches) {
		Point2d pt1 = keypoints_1[m.queryIdx].pt;
		Point2d pt2 = keypoints_2[m.trainIdx].pt;

		float b = 255 * float(rand()) / RAND_MAX;
		float g = 255 * float(rand()) / RAND_MAX;
		float r = 255 * float(rand()) / RAND_MAX;
		circle(img_show, pt1, 8, Scalar(b, g, r), 2);
		circle(img_show, Point2d(pt2.x, pt2.y + img_1.rows), 8, Scalar(b, g, r), 2);
		line(img_show, pt1, Point2d(pt2.x, pt2.y + img_1.rows), Scalar(b, g, r), 1);

		cout << "pt1: " << pt1 << endl;
		cout << "pt2: " << pt2 << endl;
	}

	imshow("result", img_show);

	waitKey(0);
}

void PoseEstimation::find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2,
	std::vector<KeyPoint>& keypoints_1,
	std::vector<KeyPoint>& keypoints_2,
	std::vector<DMatch>& matches) {
	Mat descriptors_1, descriptors_2;
	Ptr<FeatureDetector> detector = ORB::create();
	Ptr<DescriptorExtractor> descriptor = ORB::create();
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	//step 1. 检测Oriented FAST角点
	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);
	//step 2. 计算BRIEF描述子
	descriptor->compute(img_1, keypoints_1, descriptors_1);
	descriptor->compute(img_2, keypoints_2, descriptors_2);

	//step 3. 根据BRIEF算Hamming距离
	vector<DMatch> match;
	matcher->match(descriptors_1, descriptors_2, match);
	//step 4. 匹配点对筛选
	double min_dist = 10000, max_dist = 0;
	for (int i = 0; i < descriptors_1.rows; ++i) {
		double dist = match[i].distance;
		if (dist > max_dist) max_dist = dist;
		if (dist < min_dist) min_dist = dist;
	}
	//当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限
	for (int i = 0; i < descriptors_1.rows; ++i) {
		if (match[i].distance <= max(2 * min_dist, 26.0))
			matches.push_back(match[i]);
	}
}

} //namespace mf