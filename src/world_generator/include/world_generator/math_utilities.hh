#pragma once 
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/common/common.hh"
#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>

namespace math_utils{

ignition::math::Vector3d RandomPointInCircle(double radius);

}