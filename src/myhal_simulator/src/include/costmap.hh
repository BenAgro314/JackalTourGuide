#pragma once

#include <vector>
#include <sstream> 
#include <string>
#include <iostream>
#include "utilities.hh"


class Costmap{

    private:

        ignition::math::Box boundary;

        double resolution;

        ignition::math::Vector3d top_left;

        double width;

        double height;

        int rows;

        int cols;

        std::vector<std::vector<int>> costmap;

        std::vector<std::vector<double>> integration_field;

        bool PosToIndicies(ignition::math::Vector3d pos, int &r, int &c);

        bool IndiciesToPos(ignition::math::Vector3d &pos, int r, int c);

        bool Integrate(ignition::math::Vector3d goal);

        std::vector<std::vector<int>> GetNeighbours(std::vector<int> curr_ind, bool diag = false);

    public:

        Costmap(ignition::math::Box boundary, double resolution);

        void AddObject(ignition::math::Box object);

        std::string ToString();

        std::string IntegrationToString();

        bool FindPath(ignition::math::Vector3d start, ignition::math::Vector3d end,  std::vector<ignition::math::Vector3d> &path);

}; 
