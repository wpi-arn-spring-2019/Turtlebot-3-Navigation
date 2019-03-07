#pragma once
#include <tf/tf.h>

namespace Turtlebot
{

struct Particle
{
    Particle(const tf::Pose &pose_, const double &weight_) :
             pose(pose_), weight(weight_)
             {}
    ~Particle() = default;

    bool operator<(const Particle &rhs) const
    {
        return rhs.weight < weight;
    }

    tf::Pose pose;
    double weight;


};

}
