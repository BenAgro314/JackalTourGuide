#ifndef FLOW_FIELD_HH
#define FLOW_FIELD_HH

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Box.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <vector>
#include "utilities.hh"
#include "Perlin.h"

class FlowField{

    private:

        std::vector<std::vector<ignition::math::Vector3d>> field;

        std::vector<std::vector<double>> costmap;

        int rows;

        int cols;

        double resolution;

        ignition::math::Box rect;

        void PerlinInit();

        

    

    public:

        void CostMap(std::vector<gazebo::physics::EntityPtr> collision_entities);

        ignition::math::Vector3d IndiciesToPos(int r, int c);

        bool PosToIndicies(ignition::math::Vector3d pos, int &r, int &c);

        FlowField(ignition::math::Vector3d top_left, double width, double height, double resolution);

        bool Lookup(ignition::math::Vector3d pos, ignition::math::Vector3d &res);

        void PrintField();
};

#endif