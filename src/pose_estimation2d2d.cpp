#include "pose_estimation2d2d.h"
#include <vector>

using namespace std;
using namespace cv;

namespace mf {

PoseEstimation2d2d::PoseEstimation2d2d()
{
}


PoseEstimation2d2d::~PoseEstimation2d2d()
{
}

CameraPose PoseEstimation2d2d::pose_estimation2d2d(u32 *image1, u32 *image2) {
	Mat temp_1(G.h, G.w, CV_8UC4, image1);
	Mat temp_2(G.h, G.w, CV_8UC4, image2);
	Mat img_1, img_2;
	cvtColor(temp_1, img_1, CV_RGBA2BGRA);
	cvtColor(temp_2, img_2, CV_RGBA2BGRA);


	vector<KeyPoint> keypoints_1, keypoints_2;
	vector<DMatch> matches;
	find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
	cout << "found " << matches.size() << "matches." << endl;

	imshow("i1", img_1);
	imshow("i2", img_2);
	Mat img_match;
	drawMatches(temp_1, keypoints_1, temp_2, keypoints_2, matches, img_match, Scalar::all(-1), Scalar::all(-1),
		vector<char>(0), DrawMatchesFlags::DEFAULT);
	imshow("matches", img_match);


	cvWaitKey(0);

	// -- get E
	Mat R, t;
	vector<Point2f> points1, points2;
	for (int i = 0; i < (int)matches.size(); ++i) {
		points1.push_back(keypoints_1[matches[i].queryIdx].pt);
		points2.push_back(keypoints_2[matches[i].trainIdx].pt);
	}
	Mat fundamental_matrix;
	fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT); //8 points method
	cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

	Point2d principal_point(G.cx, G.cy);

	return CameraPose();
}

void PoseEstimation2d2d::find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2,
	std::vector<KeyPoint>& keypoints_1,
	std::vector<KeyPoint>& keypoints_2,
	std::vector<DMatch>& matches) {
	Mat descriptors_1, descriptors_2;
	Ptr<FeatureDetector> detector = ORB::create();
	Ptr<DescriptorExtractor> descriptor = ORB::create();
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	//step 1. ���Oriented FAST�ǵ�
	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);
	//step 2. ����BRIEF������
	descriptor->compute(img_1, keypoints_1, descriptors_1);
	descriptor->compute(img_2, keypoints_2, descriptors_2);

	//step 3. ����BRIEF��Hamming����
	vector<DMatch> match;
	matcher->match(descriptors_1, descriptors_2, match);
	//step 4. ƥ����ɸѡ
	double min_dist = 10000, max_dist = 0;
	for (int i = 0; i < descriptors_1.rows; ++i) {
		double dist = match[i].distance;
		if (dist > max_dist) max_dist = dist;
		if (dist < min_dist) min_dist = dist;
	}
	//��������֮��ľ��������������С����ʱ,����Ϊƥ������.����ʱ����С�����ǳ�С,����һ������ֵ30��Ϊ����
	for (int i = 0; i < descriptors_1.rows; ++i) {
		if (match[i].distance <= max(2 * min_dist, 30.0))
			matches.push_back(match[i]);
	}
}

} // namespace mf;
