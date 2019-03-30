#pragma once

namespace Turtlebot
{

template <typename Object>

struct Point
{
    Point(){}
    Point(const Object &x_, const Object &y_) :
        x(x_),
        y(y_)
        {}
    ~Point() = default;
      bool operator==(const Point<Object> &rhs) const
      {
          return (rhs.x == x && rhs.y == y);
      }

      Object x;
      Object y;

};

}
