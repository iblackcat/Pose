#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;

int main(int, char**)
{
  cout.precision(3);
  cout << MatrixXi::Ones(2,3) << endl;

  return 0;
}
