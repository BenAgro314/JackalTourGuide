#pragma once 
#include "objects.hh"
#include "math_utils.hh"
#include <ignition/math/graph/Graph.hh>
#include <vector>
#include <map>


namespace dungeon{

struct RoomInfo{

    double width, length, min_wall_width, max_wall_width;

    RoomInfo(double width, double length, double min_wall_width, double max_wall_width);

};

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

        double hallway_w;

        boost::shared_ptr<BSPDungeon> child_a = nullptr;

        boost::shared_ptr<BSPDungeon> child_b = nullptr;

        ignition::math::Box room_bounds;

        //std::vector<boost::shared_ptr<BSPDungeon>> leaves;

        std::map<boost::shared_ptr<BSPDungeon>, int> leaves;

        double dense_prob = 1;

        boost::shared_ptr<RoomInfo> room_specs;
        
        ignition::math::graph::UndirectedGraph<boost::shared_ptr<BSPDungeon>, ignition::math::Line3d> connections;

        void CreateChildren();

        void ConnectChildren();

        void CreateConnections();

        void Slice(bool vert);

    public:

        BSPDungeon(ignition::math::Box bounds, double x_res, double y_res, boost::shared_ptr<RoomInfo> room_specs, double hallway_width);

        void SetDensity(double prob);

        void CreateRoom();

        virtual void FillCells();

};


}