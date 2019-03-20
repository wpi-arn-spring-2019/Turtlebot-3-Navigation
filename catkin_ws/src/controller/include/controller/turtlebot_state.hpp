#pragma once

namespace Turtlebot
{

struct TurtlebotState
{
    TurtlebotState(const double &x_, const double &y_, const double &th_,
                   const double &v_, const double &th_dot_) :
        x(x_),
        y(y_),
        th(th_),
        v(v_),
        th_dot(th_dot_)
        {}
    ~TurtlebotState() = default;

    double x;
    double y;
    double th;
    double v;
    double th_dot;

};

}
