#include <cstdio>
#include <iostream>
#include "utils.h"
#include "dataset_helper.h"
#include "pose_estimation2d2d.h"
#include "pose_estimation3d2d.h"
#include "image_rectification.h"
#include "stereo_matching.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

using namespace mf;
using namespace std;

//mf::GlobalCoeff mf::G(320, 240, 500.0, 500.0, 150.0, 100.0, 3.0);
mf::GlobalCoeff mf::G(640, 480, 700.0, 700.0, 313.0, 256.0, 3.67);

void ignore_blank_pixel(u32 *image) {
	for (int i = 0; i < G.h; ++i) {
		for (int j = 0; j < G.w; ++j) {
			if ((image[i*G.w + j] >> 24) == 0) image[i*G.w + j] = 0;
		}
	}
}

int main() {
	G = G640;
	cout << G.w << endl;
	jhw_gl::GLInit(G.w, G.h);

	FILE *fin = fopen("res/test1.txt", "r");
	char path1[1024], path2[1024], path3[1024];
	fscanf(fin, " %s %s %s", path1, path2, path3);
	
	DatasetHelper DH;
	
	u32 *i1 = (u32*)DH.readImage(path1);
	u32 *i2 = (u32*)DH.readImage(path2);

	//ignore_blank_pixel(i1);
	//ignore_blank_pixel(i2);

	//2d2d
	
	PoseEstimation2d2d poseguess;
	CameraPose p2 = poseguess.pose_estimation2d2d(i1, i2);
	CameraPose p1 = CameraPose::Identity();

	//3d2d
	/*
	PoseEstimation3d2d poseguess;
	CameraPose p2 = poseguess.pose_estimation3d2d(i1, path3, i2);
	CameraPose p1(p2.intrinsics, Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
	*/
	cout << "p2 " << endl << p2.R << endl << p2.t.transpose() << endl;
	 
	u32 *i1_rec = nullptr, *i2_rec = nullptr;
	CameraPose p1_rec, p2_rec;
	ImageRectification imagerectify;
	imagerectify.init();
	imagerectify.image_rectification(i1, i2, p1, p2, &i1_rec, &i2_rec, p1_rec, p2_rec);


	DH.writeImage(i1_rec, "out1.png");
	DH.writeImage(i2_rec, "out2.png");
	
	
	float *delta1, *delta2;
	StereoMatching stereomatch;
	stereomatch.init();
	stereomatch.disparity_estimation(i1_rec, i2_rec, &delta1, &delta2);

	int index = 0;
	for (int i = 0; i < G.h; ++i) {
		for (int j = 0; j < G.w; ++j) {
			if (index == 100) break;
			if (delta1[i*G.w + j] > 0) {
				cout << delta1[i*G.w + j] << endl;
				index++;
			}
		}
	}
	
	return 0;
}