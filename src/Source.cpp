#include <cstdio>
#include <iostream>
#include <time.h>

#include "utils.h"
#include "dataset_helper.h"
#include "pose_estimation2d2d.h"
#include "pose_estimation3d2d.h"
#include "pose_estimation_dense.h"
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


DatasetHelper DH;

void ignore_blank_pixel(u32 *image) {
	for (int i = 0; i < G.h; ++i) {
		for (int j = 0; j < G.w; ++j) {
			if ((image[i*G.w + j] >> 24) == 0) image[i*G.w + j] = 0;
		}
	}
}

void visualizeIDY(const u32 const *ii, const float const *dd, const u8 const *yy) {
	DH.writeImage(ii, "o_i.png");

	u8 *output = (u8*)malloc(sizeof(u8) * G.w * G.h);

	for (int i = 0; i < G.h; ++i) {
		for (int j = 0; j < G.w; ++j) {
			output[i*G.w + j] = (u8)dd[i*G.w + j] * 5;
		}
	}
	DH.writeImage(output, "o_d.png", 1);

	for (int i = 0; i < G.h; ++i) {
		for (int j = 0; j < G.w; ++j) {
			output[i*G.w + j] = yy[i*G.w + j] * 50;
		}
	}
	DH.writeImage(output, "o_y.png", 1);

	free(output); output = nullptr;
}

int main() {
	//G = G640;
	cout << G.w << endl;

	jhw_gl::GLInit(G.w, G.h);
	
/*
	u32 *i1_rec = nullptr, *i2_rec = nullptr;
	
	CameraPose p1_rec, p2_rec;
	ImageRectification imagerectify;
	imagerectify.init();
	imagerectify.image_rectification(i1, i2, p1, p2, &i1_rec, &i2_rec, p1_rec, p2_rec);
	DH.writeImage(i1_rec, "out1.png");
	DH.writeImage(i2_rec, "out2.png");
	
	i2_rec = (u32*)DH.readImage("E:/projects/mobilefusion/datasets/stereomatch/cones-png-2/cones/im2.png");
	i1_rec = (u32*)DH.readImage("E:/projects/mobilefusion/datasets/stereomatch/cones-png-2/cones/im6.png");


	StereoMatching stereomatch;
	cout << "init ok? " << stereomatch.init(StereoMatching::STEREOMATCH_ZNCC) << endl;
	//float *depth = stereomatch.stereo_matching(i1_rec, p1_rec, i2_rec, p2_rec, 2, 3);
	float *del1 = nullptr, *del2 = nullptr;
	stereomatch.disparity_estimation(i1_rec, i2_rec, &del1, &del2);
	float *depth = stereomatch.lrcheck_and_depth(del1, del2, 1);


	u8 *output_depth = (u8*)malloc(sizeof(u8) * G.w * G.h);
	//DH.writeImage(i2_rec, "out22.png");
	for (int i = 0; i < G.h; ++i) {
		for (int j = 0; j < G.w; ++j) {
			output_depth[i*G.w + j] = (u8)depth[i*G.w + j] * 5;
		}
	}
	DH.writeImage(output_depth, "depth.png", 1);
	free(output_depth);
*/

	
	float model_size = 4.0;
	printf("please input model size: ");
	scanf("%f", &model_size);
	if (!model_size) {
		cerr << "model_size cannot be zero!" << endl;
	}



	FILE *fin = fopen("res/test.txt", "r");
	int n = 0;
	fscanf(fin, "%d", &n);

	char path1[1024], path2[1024];
	u32 *i1, *i2;


	TSDFModel tsdfmodel;
	tsdfmodel.init(model_size);

	fscanf(fin, " %s", path1);
	i1 = (u32*)DH.readImage(path1);
	//CameraPose p1(G.Intrinsic, Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 5));
	CameraPose p1(G.Intrinsic, Eigen::AngleAxisd(-0.5, Eigen::Vector3d(0.0, 1.0, 0.0).normalized()).toRotationMatrix(), Eigen::Vector3d(0, 0, 25));
	CameraPose p0 = p1;
	for (int index = 1; index < 50; ++index) {
		cout << "index: " << index << endl;
 		fscanf(fin, "%s", path2);
		i2 = (u32*)DH.readImage(path2);

		u32		*ii;
		float	*dd = nullptr;
		u8		*yy = nullptr;

		CameraPose p2;
		
		Eigen::Matrix3d R = Eigen::AngleAxisd(index * 0.05 -0.5, Eigen::Vector3d(0.0, 1.0, 0.0).normalized()).toRotationMatrix();
		//cout << "R = " << endl;
		//cout << R << endl;
		Eigen::Vector3d t = Eigen::Vector3d(0.0, double(index) / 100, 25) ;
		p2 = mf::CameraPose(G.Intrinsic, R, t);
		/*
		//2d2d
		if (index == 1) {
			PoseEstimation2d2d poseguess;
			p2 = poseguess.pose_estimation2d2d(i1, i2);
			//CameraPose p1 = CameraPose::Identity();
			p2 = CameraPose(G.Intrinsic, p2.SE3_Rt * p1.SE3_Rt);
			 
			cout << "!!!!!p2.R" << endl << p2.R << endl;
			cout << "!!!!!p2.R.row2" << endl << p2.R.row(2) << endl;
		}
		//3d2d
		else {
			tsdfmodel.ray_tracing(p1, &i1, &dd, &yy); 
			visualizeIDY(i1, dd, yy);
			//free(dd); dd = nullptr;
			free(yy); yy = nullptr;

			PoseEstimationDense poseguess;
			p2 = poseguess.pose_estimation_dense(i1, dd, i2);
			cout << "P2 - se3 : " << p2.se3 << endl;

			//PoseEstimation3d2d poseguess;
			//p2 = poseguess.pose_estimation3d2d(i1, "res/o_d.png", i2);

			p2 = CameraPose(G.Intrinsic, p2.SE3_Rt * p1.SE3_Rt);
			//CameraPose p1(p2.intrinsics, Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
		}
		*/



		u32 *i1_rec = nullptr, *i2_rec = nullptr;
		LARGE_INTEGER t1, t2, freq;
		QueryPerformanceFrequency(&freq);

		CameraPose p1_rec, p2_rec;
		ImageRectification imagerectify;
		imagerectify.init();

		//QueryPerformanceCounter(&t1);
		imagerectify.image_rectification(i1, i2, p1, p2, &i1_rec, &i2_rec, p1_rec, p2_rec);
		//QueryPerformanceCounter(&t2);

		//cout << "time: " << (t2.QuadPart - t1.QuadPart) * 1000.0 / freq.QuadPart << endl;
		//scanf("%*d");

		DH.writeImage(i1_rec, "out1.png");
		DH.writeImage(i2_rec, "out2.png");
		
		StereoMatching stereomatch;
		cout << "init ok? " << stereomatch.init(StereoMatching::STEREOMATCH_ZNCC) << endl;
		//float *depth = stereomatch.stereo_matching(i1_rec, p1_rec, i2_rec, p2_rec, 2, 3);
		float *del1 = nullptr, *del2 = nullptr;
		//stereomatch.disparity_estimation(i1_rec, i2_rec, &del1, &del2, 3);

		stereomatch.disparity_estimation(i1, i2, &del1, &del2, 3);

		float *depth = stereomatch.lrcheck_and_depth(del1, del2, (p1_rec.center - p2_rec.center).norm(), 5);

		//float *depth = stereomatch.lrcheck_and_depth(del1, del2, 1, 1);
		
		u8 *output_depth = (u8*)malloc(sizeof(u8) * G.w * G.h);
		//DH.writeImage(i2_rec, "out22.png");

		for (int i = 0; i < G.h; ++i) {
			for (int j = 0; j < G.w; ++j) {
				output_depth[i*G.w + j] = (u8)del1[i*G.w + j];
			}
		}
		DH.writeImage(output_depth, "delta1.png", 1);
		
		for (int i = 0; i < G.h; ++i) {
			for (int j = 0; j < G.w; ++j) {
				output_depth[i*G.w + j] = (u8)del2[i*G.w + j];
			}
		}
		DH.writeImage(output_depth, "delta2.png", 1);

		for (int i = 0; i < G.h; ++i) {
			for (int j = 0; j < G.w; ++j) {
				output_depth[i*G.w + j] = (u8)depth[i*G.w + j];
			}
		}
		DH.writeImage(output_depth, "depth.png", 1);
		free(output_depth);
		output_depth = nullptr;
		//return 0;

		tsdfmodel.model_updating(i1_rec, depth, p1_rec);

		tsdfmodel.ray_tracing(p2, &ii, &dd, &yy);
		visualizeIDY(ii, dd, yy);

		free(ii); ii = nullptr;
		free(dd); dd = nullptr;
		free(yy); yy = nullptr;
		

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
		//free(i1); i1 = nullptr;
		//free(i2); i2 = nullptr;
		p1 = p2;
	}
	DH.writeModel(tsdfmodel.get_modelC(), tsdfmodel.get_modelSW(), 10);
/*
	G.h = 4096;
	G.w = 4096;

	u32 *model = (u32*)DH.readImage("E:/projects/mobilefusion/QtGuiApplication/res/modelSW10.png", 4);
	u32 *modelc = (u32*)DH.readImage("E:/projects/mobilefusion/QtGuiApplication/res/modelC10.png", 4);
	vector<Eigen::Vector3i> v;
	vector<Eigen::Vector3i> vi;
	for (int i = 0; i < 4096; ++i) {
		for (int j = 0; j < 4096; ++j) {
			int f = model[i * 4096 + j] & 0xff;
			f = f - 128;
			int w = modelc[i * 4096 + j] & 0xff;
			if (f > -2 && f < 2 && w > 20) {

				int z = int(i / 256) * 16 + int(j / 256);
				int y = i - int(i / 256) * 256;
				int x = j - int(j / 256) * 256;
				v.push_back(Eigen::Vector3i(x, y, z));
				vi.push_back(Eigen::Vector3i(modelc[i * 4096 + j] & 0xff, (modelc[i * 4096 + j] >> 8) & 0xff, (modelc[i * 4096 + j] >> 16) & 0xff));
			}
		}
	}
	FILE *fout = fopen("test_model.ply", "w");
	fprintf(fout, "ply\nformat ascii 1.0\ncomment author : Greg Turk\ncomment object : another cube\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nelement face 0\nproperty list uchar int vertex_index\nelement edge 0\nproperty int vertex1\nproperty int vertex2\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n", v.size());

	//fprintf(fout, "%d\n", v.size());
	for (int i = 0; i < v.size(); ++i) {
		fprintf(fout, "%d %d %d %d %d %d\n", v[i][0], v[i][1], v[i][2], vi[i][0], vi[i][1], vi[i][2]);
	}
	fclose(fout);
*/
	//tsdfmodel.destroy();
	
	return 0;
}