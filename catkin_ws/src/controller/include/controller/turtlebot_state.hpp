#pragma once


namespace Turtlebot
{

struct TurtlebotState
{
    TurtlebotState(const double &x_, const double &y_, const double &th_,
                   const double &v_, const double &th_dot_,
                   const double &x_dot_, const double &y_dot_,
                   const double &x_ddot_, const double &y_ddot_) :
        x(x_),
        y(y_),
        th(th_),
        v(v_),
        th_dot(th_dot_),
        x_dot(x_dot_),
        y_dot(y_dot_),
        x_ddot(x_ddot_),
        y_ddot(y_ddot_)
        {}
    ~TurtlebotState() = default;

    double x;
    double y;
    double th;
    double v;
    double th_dot;
    double x_dot;
    double y_dot;
    double x_ddot;
    double y_ddot;

};

}
