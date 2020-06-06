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
#include <algorithm>

class FlowField{

    private:

        std::vector<std::vector<ignition::math::Vector3d>> field;

        std::vector<std::vector<double>> costmap;

        std::vector<std::vector<double>> integration_field;

        int rows;

        int cols;

        double resolution;

        ignition::math::Box rect;

        std::vector<std::vector<int>> GetNeighbours(std::vector<int> curr_ind, bool diag = false);

        void PerlinInit();

        void Init();

        void IntegrationField(double x, double y);

        void CostMap(std::vector<gazebo::physics::EntityPtr> collision_entities);

    public:

        FlowField(ignition::math::Vector3d top_left, double width, double height, double resolution);

        void TargetInit(std::vector<gazebo::physics::EntityPtr> collision_entities, ignition::math::Vector3d target);

        void SetTarget(ignition::math::Vector3d target);

        ignition::math::Vector3d IndiciesToPos(int r, int c);

        bool PosToIndicies(ignition::math::Vector3d pos, int &r, int &c);

        bool Lookup(ignition::math::Vector3d pos, ignition::math::Vector3d &res);

};

#endif