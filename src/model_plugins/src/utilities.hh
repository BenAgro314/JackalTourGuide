#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Line3.hh>
#include <vector>
#include <string>

namespace utilities{

    std::vector<ignition::math::Line3d> get_edges(gazebo::physics::EntityPtr entity);

    std::vector<ignition::math::Vector3d> get_corners(gazebo::physics::EntityPtr entity);

    ignition::math::Vector3d min_repulsive_vector(ignition::math::Vector3d pos, gazebo::physics::EntityPtr entity);
}