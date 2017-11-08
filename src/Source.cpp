#include <cstdio>
#include <iostream>
#include "utils.h"
#include "dataset_helper.h"
#include "pose_estimation2d2d.h"
#include "pose_estimation3d2d.h"
#include "image_rectification.h"
#include "stereo_matching.h"
#include "tsdf_model.h"

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
//mf::GlobalCoeff mf::G(1067, 800, 752.82, 564.44, 533.5, 400, 5);
//mf::GlobalCoeff mf::G(450, 375, 500.0, 500.0, 225.0, 175.0, 3.0);

void ignore_blank_pixel(u32 *image) {
	for (int i = 0; i < G.h; ++i) {
		for (int j = 0; j < G.w; ++j) {
			if ((image[i*G.w + j] >> 24) == 0) image[i*G.w + j] = 0;
		}
	}
}

int main() {
	//G = G640;
	cout << G.w << endl;
	jhw_gl::GLInit(G.w, G.h);
	DatasetHelper DH;

	/*
	u32 *img1 = (u32*)DH.readImage("E:/projects/mobilefusion/datasets/stereomatch/cones-png-2/cones/im2.png");
	u32 *img2 = (u32*)DH.readImage("E:/projects/mobilefusion/datasets/stereomatch/cones-png-2/cones/im6.png");

	StereoMatching sm;
	sm.init(StereoMatching::STEREOMATCH_SSD);

	u8 *delta1 = nullptr, *delta2 = nullptr;
	u8 *dd = nullptr;

	sm.disparity_estimationu8(img1, img2, &delta1, &delta2, 2);
	DH.writeImage(delta1, "E:/projects/mobilefusion/datasets/stereomatch/cones-png-2/cones/delta1.png", 1);
	DH.writeImage(delta2, "E:/projects/mobilefusion/datasets/stereomatch/cones-png-2/cones/delta2.png", 1);

	//dd = sm.lrcheck_and_depth(delta1, delta2, 1, 2);
	//dd = sm.stereo_matching(img1, img2, 1.f);
	//DH.writeImage(dd, "E:/projects/mobilefusion/datasets/stereomatch/cones-png-2/cones/depth2.png", 1);

	return 0;

	*/


	FILE *fin = fopen("res/test3.txt", "r");
	int n = 0;
	fscanf(fin, "%d", &n);

	char path1[1024], path2[1024];
	u32 *i1, *i2;


	TSDFModel tsdfmodel;
	float model_size = 4.0;
	printf("please input model size: ");
	scanf("%f", &model_size);
	tsdfmodel.init(model_size);

	fscanf(fin, " %s", path1);
	i1 = (u32*)DH.readImage(path1);
	CameraPose p1(G.Intrinsic, Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 5));
	CameraPose p0 = p1;
	for (int index = 1; index < 50; ++index) {
		fscanf(fin, "%s", path2);
		if (index != 3) continue;
		i2 = (u32*)DH.readImage(path2);

		//2d2d

		PoseEstimation2d2d poseguess;
		CameraPose p2 = poseguess.pose_estimation2d2d(i1, i2);
		//CameraPose p1 = CameraPose::Identity();
		p2 = CameraPose(G.Intrinsic, p2.SE3_Rt * p1.SE3_Rt);

		//3d2d
		/*
		PoseEstimation3d2d poseguess;
		CameraPose p2 = poseguess.pose_estimation3d2d(i1, path3, i2);
		CameraPose p1(p2.intrinsics, Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
		*/

		u32 *i1_rec = nullptr, *i2_rec = nullptr;
		CameraPose p1_rec, p2_rec;
		ImageRectification imagerectify;
		imagerectify.init();
		imagerectify.image_rectification(i1, i2, p1, p2, &i1_rec, &i2_rec, p1_rec, p2_rec);
		DH.writeImage(i1_rec, "out1.png");
		DH.writeImage(i2_rec, "out2.png");

		StereoMatching stereomatch;
		cout << "init ok? " << stereomatch.init(StereoMatching::STEREOMATCH_ZNCC) << endl;
		u8 *depth = stereomatch.stereo_matching(i1_rec, p1_rec, i2_rec, p2_rec, 2, 3);

		/*
		//DH.writeImage(i2_rec, "out22.png");
		for (int i = 0; i < G.h; ++i) {
			for (int j = 0; j < G.w; ++j) {
				depth[i*G.w + j] *= 2;
			}
		}
		*/

		DH.writeImage(depth, "depth.png", 1);

		tsdfmodel.model_updating(i1_rec, depth, p1_rec);

		u32 *ii;
		tsdfmodel.ray_tracing(p0, &ii, nullptr, nullptr);
		DH.writeImage(ii, "ii.png");
		free(ii); ii = nullptr;

		/*
		int times = 0;
		for (int i = 0; i < TSDFModel::ModelTexSize; ++i) {
			for (int j = 0; j < TSDFModel::ModelTexSize; ++j) {
				if (times == 100) break;
				if (tsdfmodel.modelC(i, j) != 0) {
					printf("%x\n", tsdfmodel.modelC(i, j));
					++times;
				}
			}
		}
		*/
		imagerectify.destroy();
		stereomatch.destroy();

		free(i1_rec); i1_rec = nullptr;
		free(i2_rec); i2_rec = nullptr;

		free(depth); depth = nullptr;

		strcpy(path1, path2);
		free(i1); i1 = i2;  i2 = nullptr;
		p1 = p2;
	}
	DH.writeModel(tsdfmodel.get_modelC(), tsdfmodel.get_modelSW(), 10);
	tsdfmodel.destroy();

	return 0;
}