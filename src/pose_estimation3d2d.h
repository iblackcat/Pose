#pragma once

#include "utils.h"
#include "pose_estimation.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

namespace mf{

class PoseEstimation3d2d : public PoseEstimation
{
public:
	PoseEstimation3d2d();
	~PoseEstimation3d2d();

	CameraPose pose_estimation3d2d(u32 *image1, u32 *depth, u32 *image2);
};

} // namespace mf
