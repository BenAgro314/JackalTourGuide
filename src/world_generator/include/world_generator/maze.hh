#pragma once 
#include "objects.hh"
#include "math_utils.hh"
#include <vector>

namespace dungeon{

struct Tuple{
    int r;
    int c;

    Tuple(int r, int c){
        this->r = r;
        this->c = c;
    }
};


class Cell{

    protected:

        ignition::math::Box bounds;

        bool filled;

        int cost;

    public:

        Cell(ignition::math::Box bounds, bool filled = false);

        void SetFill(bool filled);

        void AddToWorld(gazebo::physics::WorldPtr world);

};

class Grid{

    protected:

        

        double x_res,y_res;

        int rows, cols;

        ignition::math::Box bounds;

        std::vector<std::vector<boost::shared_ptr<Cell>>> cells;

        Tuple PosToIndicies(ignition::math::Vector3d pos);

        ignition::math::Vector3d IndiciesToPos(int r, int c);

        ignition::math::Vector3d IndiciesToPos(Tuple t);


    public:

        Grid(ignition::math::Box bounds, double x_res, double y_res);

        void AddToWorld(gazebo::physics::WorldPtr world);

        void FillCells();
};

}