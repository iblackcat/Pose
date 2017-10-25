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

} // namespace mf;
