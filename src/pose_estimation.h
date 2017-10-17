#pragma once
#include "utils.h"
#include "pose_estimation2d2d.h"
//#include "pose_estimation2d3d.h"

namespace mf {

class PoseEstimation
{
public:
	PoseEstimation();
	~PoseEstimation();

	bool init(); //initiate
	CameraPose pose_estimation2d2d(u32 *image1, u32 *image2);
	CameraPose pose_estimation2d3d(u32 *image1, u32 *depth1, u32 *image2);

private:
	PoseEstimation2d2d pose2d2d;
	//PoseEstimation2d3d pose2d3d;
};

} //namespace mf
