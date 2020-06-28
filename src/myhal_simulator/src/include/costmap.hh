#pragma once

#include <vector>
#include <sstream> 
#include <string>
#include <iostream>
#include "utilities.hh"
#include <fstream>
#include "frame.hh"
#include <queue>
#include <map>
#include <utility>  
#include <algorithm>


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

        std::vector<std::vector<int>> last_path;

        std::vector<std::vector<double>> integration_field;

    public:

        bool PosToIndicies(ignition::math::Vector3d pos, int &r, int &c);

        bool IndiciesToPos(ignition::math::Vector3d &pos, int r, int c);

        bool Integrate(ignition::math::Vector3d goal);

        std::vector<std::vector<int>> GetNeighbours(std::vector<int> curr_ind, bool diag = false);

        Costmap(ignition::math::Box boundary, double resolution);

        void AddObject(ignition::math::Box object);

        std::string ToString();

        std::string PathString(std::vector<TrajPoint> path);

        bool FindPath(ignition::math::Vector3d start, ignition::math::Vector3d end,  std::vector<ignition::math::Vector3d> &path);

        bool Walkable(ignition::math::Vector3d start, ignition::math::Vector3d end);

        bool DijkstraSearch(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d>& path);

        double Heuristic(std::vector<int> loc1, std::vector<int> loc2);

        bool AStarSearch(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d>& path);

        bool ThetaStarSearch(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d>& path);
}; 

template<typename T, typename priority_t>
struct PriorityQueue {

  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }

  
};