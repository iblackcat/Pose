#include "pose_estimation_dense.h"

#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>

using namespace std;
using namespace Eigen;
using std::sqrt;

namespace mf {

const int g_M = 10000;

PoseEstimationDense::PoseEstimationDense()
{
}


PoseEstimationDense::~PoseEstimationDense()
{
}

std::vector<PointData> PoseEstimationDense::m_points(g_M);

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
	lmdif_functor(u8 *i1, u8 *i2) : Functor<double>(6, g_M) { N = 6; M = g_M; image1 = i1; image2 = i2; }
	int operator()(const VectorXd &x, VectorXd &fvec) const
	{
		//double tmp1, tmp2, tmp3;
		//static const double y[15] = { 1.4e-1,1.8e-1,2.2e-1,2.5e-1,2.9e-1,3.2e-1,3.5e-1,3.9e-1,3.7e-1,5.8e-1,7.3e-1,9.6e-1,1.34e0,2.1e0,4.39e0 };

		assert(x.size() == N);
		assert(fvec.size() == M);
		assert(PoseEstimationDense::m_points.size() == M);

		CameraPose pose(G.Intrinsic, Vector6d(x));

		double sum = 0.0; 

		for (int i = 0; i < M; ++i) {
			
			Vector3d p1 = PoseEstimationDense::m_points[i].get_point3d();
			Vector3d p2 = pose.Q * p1 + pose.q;
			Vector2d m1 = PoseEstimationDense::m_points[i].get_point2d();
			Vector2d m2 = Vector2d(p2[0] / p2[2], p2[1] / p2[2]);

			if (m2[0] >= 0 && m2[0] < G.w && m2[1] >= 0 && m2[1] < G.h) {
				fvec[i] = binterd(image1, m1[0], m1[1], G.w) - binterd(image2, m2[0], m2[1], G.w);
				fvec[i] *= fvec[i];
			}
			else {
				fvec[i] = 255*255;
			}
			
			sum += fvec[i];
			//cout << i << ": " << fvec[i] << endl;
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
		cout << x.transpose() << ": " << sum << endl;

		return 0;
	}
private:
	int N, M;
	u8 *image1 = nullptr, *image2 = nullptr;
};



CameraPose PoseEstimationDense::pose_estimation_dense(u32 *image1, float *depth1, u32 *image2) {
	CameraPose pose = CameraPose::Identity();
	VectorXd se3(6);

	for (int i = 0; i < 6; ++i) {
		se3[i] = 0.0;
	}

	int index = 0;
	for (int i = 10; i < G.h - 10; ++i) {
		if (index == g_M) break;
		for (int j = 10; j < G.w - 10; ++j) {
			if (index == g_M) break;

			if (image1[i*G.w + j] && depth1[i*G.w + j]) {
				PoseEstimationDense::m_points[index++] = PointData(j, i, depth1[i*G.w + j]);
				//index++;
			}
		}
	}
	cout << "index = " << index << endl;

	u8 *i1 = (u8*)malloc(sizeof(u8)*G.w*G.h);
	u8 *i2 = (u8*)malloc(sizeof(u8)*G.w*G.h);

	for (int i = 0; i < G.h; ++i) {
		for (int j = 0; j < G.w; ++j) {
			int r, g, b, a;
			if (!image1[i*G.w + j]) i1[i*G.w + j] = 0;
			else {
				r = image1[i*G.w + j] & 0xff;
				g = image1[i*G.w + j] >> 8 & 0xff;
				b = image1[i*G.w + j] >> 16 & 0xff;
				a = image1[i*G.w + j] >> 24 & 0xff;

				i1[i*G.w + j] = u8((r * 299 + g * 587 + b * 114 + 500) / 1000);
			}

			if (!image2[i*G.w + j]) i2[i*G.w + j] = 0;
			else {
				r = image2[i*G.w + j] & 0xff;
				g = image2[i*G.w + j] >> 8 & 0xff;
				b = image2[i*G.w + j] >> 16 & 0xff;
				a = image2[i*G.w + j] >> 24 & 0xff;

				i2[i*G.w + j] = u8((r * 299 + g * 587 + b * 114 + 500) / 1000);
			}
		}
	}

	lmdif_functor functor(i1, i2);
	NumericalDiff<lmdif_functor> numDiff(functor);
	LevenbergMarquardt<NumericalDiff<lmdif_functor>> lm(numDiff);
	int info = lm.minimize(se3);

	cout << "se3: " << se3.transpose() << endl;
	return CameraPose(G.Intrinsic, Vector6d(se3));
}

}


