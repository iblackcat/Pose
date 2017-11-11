#include "pose_estimation_dense.h"

#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>

using namespace std;
using namespace Eigen;
using std::sqrt;

namespace mf {

PoseEstimationDense::PoseEstimationDense()
{
}


PoseEstimationDense::~PoseEstimationDense()
{
}


// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
	typedef _Scalar Scalar;
	enum {
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }

	// you should define that in the subclass :
	//  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};

/*
struct my_functor : Functor<double>
{
	my_functor(void) : Functor<double>(2, 2) {}
	int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
	{
		// Implement y = 10*(x0+3)^2 + (x1-5)^2
		fvec(0) = 10.0*pow(x(0) + 3.0, 2) + pow(x(1) - 5.0, 2);
		fvec(1) = 0;

		return 0;
	}
};
*/
struct lmdif_functor : Functor<double>
{
	lmdif_functor() : Functor<double>(6, 1000) { N = 6; M = 1000;}
	int operator()(const VectorXd &x, VectorXd &fvec) const
	{
		//double tmp1, tmp2, tmp3;
		//static const double y[15] = { 1.4e-1,1.8e-1,2.2e-1,2.5e-1,2.9e-1,3.2e-1,3.5e-1,3.9e-1,3.7e-1,5.8e-1,7.3e-1,9.6e-1,1.34e0,2.1e0,4.39e0 };

		assert(x.size() == N);
		assert(fvec.size() == M);
		assert(PoseEstimationDense::m_points.size == M);

		CameraPose pose(G.Intrinsic, Vector6d(x));

		for (int i = 0; i < M; ++i) {
			Vector3d point = PoseEstimationDense::m_points[i].get_point3d();
			point = pose.Q * point + pose.q;
			point[0] /= point[2];
			point[1] /= point[2];

			fvec[i] = ;
		}
		/*
		for (i = 0; i<15; i++)
		{
			tmp1 = i + 1;
			tmp2 = 15 - i;
			tmp3 = tmp1;

			if (i >= 8) tmp3 = tmp2;
			fvec[i] = y[i] - (x[0] + tmp1 / (x[1] * tmp2 + x[2] * tmp3));
		}
		*/
		return 0;
	}
private:
	int N, M;
};



CameraPose PoseEstimationDense::pose_estimation3d2d(u32 *image1, float *depth1, u32 *image2) {
	CameraPose pose = CameraPose::Identity();
	VectorXd se3(6);

	int index = 0;
	for (int i = 0; i < G.h; ++i) {
		if (index == 1000) break;
		for (int j = 0; j < G.w; ++j) {
			if (index == 1000) break;

			if (image1[i*G.w + j] && depth1[i + G.w + j]) {

			}
		}
	}

	lmdif_functor functor;
	NumericalDiff<lmdif_functor> numDiff(functor);
	LevenbergMarquardt<NumericalDiff<lmdif_functor>> lm(numDiff);
	int info = lm.minimize(se3);
}

}
