#include "pose_estimation3d2d.h"

#include <chrono>

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

	//vertex
	g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); //Camera Pose
	Eigen::Matrix3d R_mat;
	R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
	pose->setId(0);
	pose->setEstimate(g2o::SE3Quat(
						R_mat,
						Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2,0))
					));
	optimizer.addVertex(pose);

	int index = 1;
	for (const Point3f p : points_3d) {
		g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
		point->setId(index++);
		point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
		point->setMarginalized(true);
		point->setFixed(true); //点不动
		optimizer.addVertex(point);
	}

	//parameter: camera intrinsics
	g2o::CameraParameters *camera = new g2o::CameraParameters(
		K.at<double> (0, 0),  Eigen::Vector2d (K.at<double>(0, 2), K.at<double>(1, 2) ), 0
	);
	camera->setId(0);
	optimizer.addParameter(camera);

	//edges
	index = 1;
	for (const Point2f p : points_2d) {
		g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
		edge->setId(index);
		edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(index))); //vertex index 从1开始
		edge->setVertex(1, pose);
		edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
		edge->setParameterId(0, 0);
		edge->setInformation(Eigen::Matrix2d::Identity());
		optimizer.addEdge(edge);
		++index;
	}

	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	optimizer.setVerbose(true);
	optimizer.initializeOptimization();
	optimizer.optimize(100);
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
	cout << "optimization costs time: " << time_used.count() << "seconds." << endl;

}

CameraPose PoseEstimation3d2d::pose_estimation3d2d(u32 *image1, const char *path, u32 *image2) {
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

	Mat d1 = imread(path, CV_LOAD_IMAGE_UNCHANGED);
	Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	vector<Point3f> pts_3d;
	vector<Point2f> pts_2d;

	int sum = 0;
	for (DMatch m : matches) {
		ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
		if (d == 0) continue;

		float dd = d / 1000.0;
		Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
		pts_3d.push_back(Point3f(p1.x*dd, p1.y*dd, dd));
		pts_2d.push_back(keypoints_2[m.trainIdx].pt);
		++sum;
	}
	cout << "sum: " << sum << endl;

	Mat r, t;

	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	cv::solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false, 0);

	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
	cout << "optimization costs time: " << time_used.count() << "seconds." << endl;
	
	
	Mat R;
	cv::Rodrigues(r, R);

	cout << "R:" << endl << R << endl;
	cout << "t:" << endl << t << endl;

	//R = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//t = Mat_<double>(3, 1);
	bundleAjustment(pts_3d, pts_2d, K, R, t);

	Eigen::Matrix3d poseK, poseR;
	Eigen::Vector3d poset;

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			poseR(i, j) = R.at<double>(i, j);
			poseK(i, j) = K.at<double>(i, j);
		}
		poset[i] = t.at<double>(i, 0);
	}

	return CameraPose(poseK, poseR, poset);
}

} // namespace mf
