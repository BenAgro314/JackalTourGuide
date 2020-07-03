#pragma once 
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Rand.hh>

namespace math_utils{

ignition::math::Vector3d RandomPointInCircle(double radius){
    double t = 2*3.141*ignition::math::Rand::DblUniform(0,1);
    double u = ignition::math::Rand::DblUniform(0,1)+ignition::math::Rand::DblUniform(0,1);
    double r;
    if (u > 1){
        r = 2 -u;
    } else {
        r = u;
    }
    return ignition::math::Vector3d(radius*r*std::cos(t), radius*r*std::sin(t), 0);
}


}