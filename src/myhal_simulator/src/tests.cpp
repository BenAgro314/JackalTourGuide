#include "world_entities.hh"
#include "quadtree.hh"
#include "puppeteer.hh"
#include <iterator>
#include <algorithm>   
#include <ctime>        
#include <cstdlib> 
#include <iostream>
#include <fstream>
#include "vehicles.hh"
#include "utilities.hh"
#include "costmap.hh"


int main(int argc, char ** argv){

    // Costmap map = Costmap(ignition::math::Box(ignition::math::Vector3d(0,-10,0), ignition::math::Vector3d(10,0,0)), 0.5);

    // double min_x, min_y, max_x, max_y;
    // std::cout << "input object coords:\n";
    // std::cin >> min_x >> min_y >> max_x >> max_y;
    
    // auto obj1= ignition::math::Box(ignition::math::Vector3d(min_x,min_y,0), ignition::math::Vector3d(max_x,max_y,0));
    // //auto obj2= ignition::math::Box(ignition::math::Vector3d(2,1,0), ignition::math::Vector3d(19,2,0));

    // map.AddObject(obj1);
    // //map.AddObject(obj2);

    // std::cout << map.ToString();

    //std::vector<ignition::math::Vector3d> path;

    //map.FindPath(ignition::math::Vector3d(2, 9,0), ignition::math::Vector3d(8, 0.5, 0), path);


    //std::cout << std::endl;

 

    //for (auto p: path){
    //    std::cout << p << std::endl;
    //}

    PriorityQueue<std::vector<int>, double> frontier; 



    std::vector<int> test =  {1,2,3};
    std::vector<int> test2 =  {3,2,3};
    frontier.put( test,1);
    frontier.put({4,2},2);


    //frontier.find(test);
    if (frontier.find(test2) != frontier.last()){
        std::cout << "HI\n";
    } else {
        std::cout << "NO\n";
    }

    std::cout << frontier.empty() << std::endl;

    PriorityQueue<std::vector<int>, double> Q2;

    std::vector<int> one = {1};

    Q2.put(one,1);
    Q2.put({0},0);
    Q2.put({5},5);
    Q2.put({3},3);

    Q2.remove(one);

    while (!Q2.empty()){
        auto el = Q2.get();
        for (auto i: el){
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }

    
    
    
    return 0;
}
