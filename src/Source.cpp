#include <cstdio>
#include <iostream>
#include "src/utils.h"
#include "src/dataset_helper.h"
#include "src/pose_estimation2d2d.h"

using namespace mf;
using namespace std;

mf::GlobalCoeff mf::G(320, 240, 500.0, 500.0, 150.0, 100.0, 3.0);

int main() {
	G = G640;
	cout << G.w << endl;
	
	DatasetHelper DH;

	u32 *i1 = (u32*)DH.readImage("res/im12.png");
	u32 *i2 = (u32*)DH.readImage("res/im14.png");

	PoseEstimation2d2d poseguess;
	poseguess.pose_estimation2d2d(i1, i2);

	return 0;
}