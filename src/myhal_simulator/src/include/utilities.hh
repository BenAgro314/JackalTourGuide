#ifndef UTILITIES_HH
#define UTILITIES_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Line3.hh>
#include <ignition/math/Plane.hh>
#include <ignition/math/Matrix4.hh>   
#include <vector>
#include <string>

namespace utilities{

    std::vector<ignition::math::Line3d> get_edges(gazebo::physics::EntityPtr entity);

    std::vector<ignition::math::Line3d> get_box_edges(ignition::math::Box box);

    std::vector<ignition::math::Vector3d> get_corners(gazebo::physics::EntityPtr entity);

    std::vector<ignition::math::Vector3d> get_box_corners(ignition::math::Box box);

    double width(ignition::math::Box box);

    double height(ignition::math::Box box);

    bool inside_box(ignition::math::Box box, ignition::math::Vector3d point, bool edge = false);

    bool get_normal_to_edge(ignition::math::Vector3d pos, ignition::math::Line3d edge, ignition::math::Vector3d &normal);

    ignition::math::Vector3d min_repulsive_vector(ignition::math::Vector3d pos, gazebo::physics::EntityPtr entity);

    ignition::math::Vector3d min_box_repulsive_vector(ignition::math::Vector3d pos, ignition::math::Box box);

    double map(double val, double from_min, double from_max, double to_min, double to_max);

    void print_vector(ignition::math::Vector3d vec, bool newline = true);

    bool contains(ignition::math::Box b1, ignition::math::Box b2); // returns true if b1 contains b2 (in a 2d)


    double dist_to_box(ignition::math::Vector3d pos, ignition::math::Box box);

    ignition::math::Matrix4d InterpolatePose(double target_time, double t1, double t2, ignition::math::Matrix4d aff1, ignition::math::Matrix4d aff2);


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