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

        ignition::math::Box GetBounds();

        bool Filled();

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

        int MaxHistogramArea(std::vector<int> hist, int &l, int &r);

        ignition::math::Box MaxRectangle(std::vector<std::vector<int>> &grid, int &num_on); // returns the largest rectangle in the array that isn't included in taken and modifies taken to include that rectangle

    public:

        Grid(ignition::math::Box bounds, double x_res, double y_res);

        void AddToWorld(gazebo::physics::WorldPtr world);

        virtual void FillCells();
};

class BSPDungeon: public Grid{

    protected:

        int min_w, min_l, wall_w, hallway_w;

        bool split = false;

        boost::shared_ptr<BSPDungeon> child_a = nullptr;

        boost::shared_ptr<BSPDungeon> child_b = nullptr;

        void CreateRooms();

        void FillRoom();

        void ConnectHallways();
        
    public:

        BSPDungeon(ignition::math::Box bounds, double x_res, double y_res, int min_w, int min_l, int wall_w, int hallway_w);

        virtual void FillCells();

        

};

}