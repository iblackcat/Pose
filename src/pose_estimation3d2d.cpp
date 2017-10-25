#include "pose_estimation3d2d.h"

using namespace std;
using namespace cv;

namespace mf{

PoseEstimation3d2d::PoseEstimation3d2d()
{
}


PoseEstimation3d2d::~PoseEstimation3d2d()
{
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
	return Point2d{
		(p.x - K.at<double> (0, 2) ) / K.at<double>(0,0),
		(p.y - K.at<double> (1, 2) ) / K.at<double>(1,1)
	};
}

void bundleAjustment(const vector<Point3f> points_3d,
	const vector<Point2f> points_2d,
	const Mat& K,
	Mat& R, Mat& t) {
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
	Block* solver_ptr = new Block(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(solver);
}

CameraPose PoseEstimation3d2d::pose_estimation3d2d(u32 *image1, u32 *depth, u32 *image2) {
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

	Mat d1 = imread("res/1_depth.png", CV_LOAD_IMAGE_UNCHANGED);;
	Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	vector<Point3f> pts_3d;
	vector<Point2f> pts_2d;
	for (DMatch m : matches) {
		ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
		if (d == 0) continue;

		float dd = d / 1000.0;
		Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
		pts_3d.push_back(Point3f(p1.x*dd, p1.y*dd, dd));
		pts_2d.push_back(keypoints_2[m.trainIdx].pt);
	}

	Mat r, t;
	cv::solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false);
	Mat R;
	cv::Rodrigues(r, R);

	cout << "R:" << endl << R << endl;
	cout << "t:" << endl << t << endl;




	return CameraPose();
}

} // namespace mf
