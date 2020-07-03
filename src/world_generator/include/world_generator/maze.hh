#pragma once 
#include "objects.hh"
#include "math_utils.hh"
#include "collisions.hh"

ignition::math::Vector3d RandomPointInCircle(double radius);


namespace dungeon{

class Room{

    private:

        ignition::math::Box boundary;

        double wall_width;

        std::vector<objects::Box> walls;

        void CreateWalls();

    public:

        Room(ignition::math::Box boundary, double wall_width = 0.15);

        void AddToWorld(gazebo::physics::WorldPtr world);

        
};

class Dungeon{

    private:

        std::vector<Room> rooms;

        double init_radius;

        int num_rooms;

    public:

        Dungeon(int num_rooms, double init_radius);

        void FillCircle();

        void AddToWorld(gazebo::physics::WorldPtr world);
};

}