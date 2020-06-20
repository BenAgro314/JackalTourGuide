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

    Costmap map = Costmap(ignition::math::Box(ignition::math::Vector3d(0,0,0), ignition::math::Vector3d(10,10,0)), 0.2);
    
    auto obj1= ignition::math::Box(ignition::math::Vector3d(0,0,0), ignition::math::Vector3d(1,9,0));
    auto obj2= ignition::math::Box(ignition::math::Vector3d(3,5,0), ignition::math::Vector3d(11,6.5,0));

    map.AddObject(obj1);
    map.AddObject(obj2);

    //std::cout << map.ToString();

    std::vector<ignition::math::Vector3d> path;

    map.FindPath(ignition::math::Vector3d(2, 9,0), ignition::math::Vector3d(8, 0.5, 0), path);


    std::cout << std::endl;

 

    for (auto p: path){
        std::cout << p << std::endl;
    }
    
    return 0;
}
