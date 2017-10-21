#include <cstdio>
#include <iostream>
#include "utils.h"
#include "dataset_helper.h"
#include "pose_estimation2d2d.h"
#include "image_rectification.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

using namespace mf;
using namespace std;

mf::GlobalCoeff mf::G(320, 240, 500.0, 500.0, 150.0, 100.0, 3.0);

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
	
	DatasetHelper DH;
	
	u32 *i1 = (u32*)DH.readImage("res/test0.png");
	u32 *i2 = (u32*)DH.readImage("res/test3.png");

	//ignore_blank_pixel(i1);
	//ignore_blank_pixel(i2);

	PoseEstimation2d2d poseguess;
	CameraPose p2 = poseguess.pose_estimation2d2d(i1, i2);
	CameraPose p1 = CameraPose::Identity();
	 
	u32 *i1_rec = nullptr, *i2_rec = nullptr;
	ImageRectification imagerectify;
	imagerectify.init();
	imagerectify.image_rectification(i1, i2, p1, p2, &i1_rec, &i2_rec);

	DH.writeImage(i1_rec, "out1.png");
	DH.writeImage(i2_rec, "out2.png");
	return 0;
}