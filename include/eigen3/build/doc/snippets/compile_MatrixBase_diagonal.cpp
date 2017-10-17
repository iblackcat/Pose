#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;

int main(int, char**)
{
  cout.precision(3);
  Matrix3i m = Matrix3i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here are the coefficients on the main diagonal of m:" << endl
     << m.diagonal() << endl;

  return 0;
}
