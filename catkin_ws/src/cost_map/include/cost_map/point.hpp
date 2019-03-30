#pragma once

namespace Turtlebot
{

struct Point
{
    Point(const double &x_, const double &y_) :
        x(x_),
        y(y_)
        {}
    ~Point() = default;
      bool operator==(const Point &rhs) const
      {
          return (rhs.x == x && rhs.y == y);
      }

      double x;
      double y;
};

}
