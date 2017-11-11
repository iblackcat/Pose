#pragma once
#include "pose_estimation.h"
#include "utils.h"

namespace mf {


struct PointData {
public:
	PointData(float x_, float y_, float d_)
		: x(x_)
		, y(y_)
		, d(d_) {};
	const Eigen::Vector2d get_point2d() { return Eigen::Vector2d(x, y); }
	const Eigen::Vector3d get_point3d() { return Eigen::Vector3d(x*d, y*d, d); }
	float get_d() { return d; }
private:
	float x, y, d;
};


class PoseEstimationDense :
	public PoseEstimation
{
public:
	PoseEstimationDense();
	~PoseEstimationDense();

	CameraPose pose_estimation3d2d(u32 *image1, float *depth1, u32 *image2);


	static std::vector<PointData> m_points;
private:
};

}
