#include "pose_estimation.h"
#include "utils.h"

namespace mf {

PoseEstimation::PoseEstimation()
{
}


PoseEstimation::~PoseEstimation()
{
}


CameraPose PoseEstimation::pose_estimation2d2d(u32 *image1, u32 *image2) {
	return CameraPose::Identity();
}

CameraPose PoseEstimation::pose_estimation2d3d(u32 *image1, u32 *depth1, u32 *image2) {
	return CameraPose::Identity();
}

} //namespace mf