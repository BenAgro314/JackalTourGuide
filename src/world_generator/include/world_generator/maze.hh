#pragma once 
#include "objects.hh"
#include "math_utils.hh"

#include <vector>


namespace dungeon{

class Grid{

    protected:

        ignition::math::Box bounds;

        double x_res, y_res;

        int rows,cols;


    public:

        boost::shared_ptr<objects::Boxes> boxes;

        std::vector<std::vector<int>> binary;

        math_utils::Tuple<int> PosToIndicies(ignition::math::Vector3d pos);

        ignition::math::Vector3d IndiciesToPos(math_utils::Tuple<int> t);

        ignition::math::Vector3d IndiciesToPos(int r, int c);

        Grid(ignition::math::Box bounds, double x_res, double y_res);

        void ToString();

        void Fill();

    
        virtual void AddToWorld(gazebo::physics::WorldPtr world);

};


class BSPDungeon: public Grid{

    protected:

        double min_w, min_l, wall_w, hallway_w, min_room_w, min_room_l;

        boost::shared_ptr<BSPDungeon> child_a = nullptr;

        boost::shared_ptr<BSPDungeon> child_b = nullptr;

        void CreateChildren();

        

        void ConnectChildren();

    public:

        void CreateRoom();

        BSPDungeon(ignition::math::Box bounds, double x_res, double y_res, double min_w, double min_l, double wall_w, double hallway_w, double min_room_w = -1, double min_room_l = -1);

        virtual void FillCells();

};


}