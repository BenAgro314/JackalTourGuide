#ifndef UTILITIES_HH
#define UTILITIES_HH

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

    std::vector<ignition::math::Vector3d> get_box_corners(ignition::math::Box box);

    bool inside_box(ignition::math::Box box, ignition::math::Vector3d point);

    bool get_normal_to_edge(ignition::math::Vector3d pos, ignition::math::Line3d edge, ignition::math::Vector3d &normal);

    ignition::math::Vector3d min_repulsive_vector(ignition::math::Vector3d pos, gazebo::physics::EntityPtr entity);

    bool contains(ignition::math::Box b1, ignition::math::Box b2); // returns true if b1 contains b2 (in a 2d)


    class Path{

        public:

            double radius = 0.5;    

            Path();

            Path(double _radius);

            void AddPoint(ignition::math::Vector3d _point);

            std::vector<ignition::math::Vector3d> points;

    };
}

#endif