#pragma once
#include <tuple>
#include <tf/tf.h>
#include <cmath>

using namespace std;

typedef tuple<float, float, float> triple;
auto quarternion_to_euler(float x, float y, float z, float w) -> triple {

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    double roll, pitch, theta;
    m.getRPY(roll, pitch, theta);

    return make_tuple(roll, pitch, theta);
}

auto rad_to_deg(float rad) -> float {
    return rad * 180/M_PI;
}