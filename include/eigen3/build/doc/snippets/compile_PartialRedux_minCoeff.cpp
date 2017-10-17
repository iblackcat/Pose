#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;

int main(int, char**)
{
  cout.precision(3);
  Matrix3d m = Matrix3d::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is the minimum of each column:" << endl << m.colwise().minCoeff() << endl;

  return 0;
}
