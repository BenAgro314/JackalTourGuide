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
#include <queue>
#include <set>
#include <limits>
#include <chrono>

template <class T, class priority_t>
class PriorityQueue : public std::priority_queue<std::pair<priority_t, T>, std::vector<std::pair<priority_t, T>>, std::greater<std::pair<priority_t, T>>>
{
	public:

		typedef typename 
			std::priority_queue<
				T, 
				std::vector<std::pair<priority_t, T>>,  
				std::greater<std::pair<priority_t, T>>>::container_type::const_iterator const_iterator;

		const_iterator find(const T&val) const{
            auto first = this->c.cbegin();
            auto last = this->c.cend();
            while (first!=last) {
                if ((*first).second==val) return first;
                ++first;
            }
            return last;
        }

        const_iterator last() const{
            return this->c.cend();
        }

		T get(){
			T best = this->top().second;
			this->pop();
			return best;
		}

        void put(T item, priority_t priority){
		    this->emplace(priority, item);
	    }

        void remove(const T&val){
            auto first = this->c.cbegin();
            auto last = this->c.cend();
            while (first!=last) {
                if ((*first).second==val){
                    this->c.erase(first);
                }
                ++first;
            }
            
        }

        int size(){
            return this->c.size();
        }

        bool clear() {
		    this->c.clear();
	    }

};

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

public:

    std::vector<int> target;

	bool PosToIndicies(ignition::math::Vector3d pos, int &r, int &c);

	bool IndiciesToPos(ignition::math::Vector3d &pos, int r, int c);

	bool Integrate(ignition::math::Vector3d goal);

	std::vector<std::vector<int>> GetNeighbours(std::vector<int> curr_ind, bool diag = false);

	Costmap(ignition::math::Box boundary, double resolution);

	void AddObject(ignition::math::Box object);

	std::string ToString();

	std::string PathString(std::vector<TrajPoint> path);

	bool FindPath(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path);

	bool Walkable(ignition::math::Vector3d start, ignition::math::Vector3d end);

	bool DijkstraSearch(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path);

	double Heuristic(std::vector<int> loc1, std::vector<int> loc2);

	bool AStarSearch(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path);

	bool ThetaStarSearch(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path);

    bool AStar(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path);

    void UpdateVertex(std::vector<int> s, std::vector<int> n);

    std::map<std::vector<int>, double> g_cost;

    std::map<std::vector<int>, std::vector<int>> parent; 

    double NewHeuristic(std::vector<int> loc1, std::vector<int> loc2);

    double DistCost(std::vector<int> s, std::vector<int> n);

    PriorityQueue<std::vector<int>, double> open;
};

// template <typename T, typename priority_t>
// struct PriorityQueue
// {

// 	typedef std::pair<priority_t, T> PQElement;
// 	std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

// 	inline bool empty() const
// 	{
// 		return elements.empty();
// 	}

// 	inline void put(T item, priority_t priority)
// 	{
// 		elements.emplace(priority, item);
// 	}

// 	T get()
// 	{
// 		T best_item = elements.top().second;
// 		elements.pop();
// 		return best_item;
// 	}
// };

