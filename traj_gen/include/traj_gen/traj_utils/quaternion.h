#ifndef _quaternion_h
#define _quaternion_h
#include <math.h>
struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll);
EulerAngles ToEulerAngles(Quaternion q);
Quaternion multQuat(Quaternion q1, Quaternion q2);
#endif
