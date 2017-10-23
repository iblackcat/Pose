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

void PoseEstimation2d2d::visualization(const Mat &img_1, const Mat &img_2,
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

CameraPose PoseEstimation2d2d::pose_estimation2d2d(u32 *image1, u32 *image2) {
	Mat temp_1(G.h, G.w, CV_8UC4, image1);
	Mat temp_2(G.h, G.w, CV_8UC4, image2);
	Mat img_1, img_2;
	cvtColor(temp_1, img_1, CV_RGBA2BGRA);
	cvtColor(temp_2, img_2, CV_RGBA2BGRA);

	//find matches
	vector<KeyPoint> keypoints_1, keypoints_2;
	vector<DMatch> matches;
	find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
	cout << "found " << matches.size() << "matches." << endl;

	//visualization
	visualization(img_1, img_2, keypoints_1, keypoints_2, matches);

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
	Mat essential_matrix;
	essential_matrix = findEssentialMat(points1, points2, G.fx, principal_point);

	recoverPose(essential_matrix, points1, points2, R, t, G.fx, principal_point);


	Eigen::Matrix3d poseR;
	Eigen::Vector3d poset;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			poseR(i, j) = R.at<double>(i, j);
		}
		poset[i] = t.at<double>(i, 0);
	}
	/*
	cout << "R" << endl << R << endl;
	cout << "R - " << endl << poseR << endl;
	cout << "t" << endl << t << endl;
	cout << "t - " << endl << poset << endl;
	*/
	return CameraPose(G.Intrinsic, poseR, poset);
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
		if (match[i].distance <= max(2 * min_dist, 25.0))
			matches.push_back(match[i]);
	}
}

} // namespace mf;
