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
#include <set>
#include <limits>
#include <chrono>
#include "priority_queue.hh"



class Costmap
{

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

    std::map<std::vector<int>, double> g_cost;

    std::map<std::vector<int>, std::vector<int>> parent; 

    PriorityQueue<std::vector<int>, double> open;

    std::vector<int> target;

    void UpdateVertexA(std::vector<int> s, std::vector<int> n);

    void UpdateVertexB(std::vector<int> s, std::vector<int> n);

    double DistCost(std::vector<int> s, std::vector<int> n);

    bool Walkable(ignition::math::Vector3d start, ignition::math::Vector3d end);

    double Heuristic(std::vector<int> loc1, std::vector<int> loc2);

public:

    int obj_count;

	bool PosToIndicies(ignition::math::Vector3d pos, int &r, int &c);

	bool IndiciesToPos(ignition::math::Vector3d &pos, int r, int c);

	bool Integrate(ignition::math::Vector3d goal);

	std::vector<std::vector<int>> GetNeighbours(std::vector<int> curr_ind, bool diag = false);

	Costmap(ignition::math::Box boundary, double resolution);

	void AddObject(ignition::math::Box object);

	std::string ToString();

	std::string PathString(std::vector<TrajPoint> path);

	bool FindPath(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path);

	bool ThetaStar(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path);

    bool AStar(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path, bool straighten = true);

    bool Occupied(ignition::math::Vector3d pos);

    ignition::math::Vector3d RandPos();
};


