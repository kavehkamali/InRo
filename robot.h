#ifndef ROBOT_H
#define ROBOT_H

#define M_PI 3.1415
#include "link.h"
#include <Eigen>

using namespace Eigen;

struct DH_table {
  Matrix<double,6,1>  theta_0;
  Matrix<double,6,1>  alpha;
  Matrix<double,6,1>  a;
  Matrix<double,6,1>  d;
};

class robot
{
private:
    unsigned int nrOfLinks;
    DH_table dh_table;
    Matrix<double,6,1>   theta;
    Matrix<double,6,1>   theta_home;
public:
    link Links [7];
    robot(DH_table _dh_table);
    robot();
 //   ~robot();
    int setDH(DH_table _dh_table);
    int setJoints(Matrix<double,6,1>   _theta);
    int goHome();
    int solveFK();
    Matrix4d FK(Matrix<double,6,1>   _theta);
    Matrix<double,3,1> get_xyz(Matrix<double,6,1>   _theta);
    Matrix<double,4,1> get_q(Matrix<double,6,1>   _theta);
    int IK(Matrix4d H, Matrix<double,6,1>& IK_theta, double conf1, double conf2, double conf5);
};


#endif // ROBOT_H

