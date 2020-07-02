#include "maze.hh"

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

namespace dungeon{


Room::Room(ignition::math::Box boundary, double wall_width){
    this->boundary = boundary;
    this->wall_width = wall_width;
}


void Room::AddToWorld(gazebo::physics::WorldPtr world){
    this->CreateWalls();
    for (auto wall: this->walls){
        wall.AddToWorld(world);
    }
}

void Room::CreateWalls(){
    auto bottom = ignition::math::Box(boundary.Min(), ignition::math::Vector3d(boundary.Max().X(), boundary.Min().Y() + wall_width, boundary.Max().Z()));
    auto top = ignition::math::Box(ignition::math::Vector3d(boundary.Min().X(), boundary.Max().Y()-wall_width, boundary.Min().Z()), boundary.Max());
    auto left = ignition::math::Box(ignition::math::Vector3d(boundary.Min().X(), boundary.Min().Y() + wall_width, boundary.Min().Z()), ignition::math::Vector3d(boundary.Min().X()+wall_width, boundary.Max().Y()-wall_width, boundary.Max().Z()));
    auto right = ignition::math::Box(ignition::math::Vector3d(boundary.Max().X()-wall_width, boundary.Min().Y() + wall_width, boundary.Min().Z()), ignition::math::Vector3d(boundary.Max().X(), boundary.Max().Y()-wall_width, boundary.Max().Z()));

    auto bot_box = objects::Box(bottom);
    auto top_box = objects::Box(top);
    auto left_box = objects::Box(left);
    auto right_box = objects::Box(right);

    this->walls = {bot_box, top_box, left_box, right_box};
}


Dungeon::Dungeon(int num_rooms, double init_radius){
    this->num_rooms = num_rooms;
    this->init_radius = init_radius;
}

void Dungeon::FillCircle(){
    for (int i =0; i< num_rooms;i++){
        auto pos = RandomPointInCircle(init_radius);

        double width = std::max(3.0,ignition::math::Rand::DblNormal(7,2));
        double length = std::max(3.0,ignition::math::Rand::DblNormal(7,2));

        auto min = ignition::math::Vector3d(pos.X()-width/2, pos.Y()-length/2, 0);
        auto max = ignition::math::Vector3d(pos.X()+width/2, pos.Y() + length/2, 2);
        
        this->rooms.push_back(Room(ignition::math::Box(min,max)));
    }
}


void Dungeon::AddToWorld(gazebo::physics::WorldPtr world){
    for (auto room: rooms){
        room.AddToWorld(world);
    }
}

}