#ifndef FRAME_H
#define FRAME_H

#include <Eigen>
using namespace Eigen;

class frame
{
public:
    frame();
    Matrix4d rH;
    Matrix4d H;
};

#endif // FRAME_H
