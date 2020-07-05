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

        double min_w, min_l, wall_w, hallway_w;

        boost::shared_ptr<BSPDungeon> child_a = nullptr;

        boost::shared_ptr<BSPDungeon> child_b = nullptr;

        void CreateChildren();

        

        void ConnectChildren();

    public:

        void CreateRoom();

        BSPDungeon(ignition::math::Box bounds, double x_res, double y_res, double min_w, double min_l, double wall_w, double hallway_w);

        virtual void FillCells();

};


}