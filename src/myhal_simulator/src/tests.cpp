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

    auto pose1 = ignition::math::Pose3d(ignition::math::Vector3d(0,0,0), ignition::math::Quaterniond(0,1,0,0));
    auto pose2 = ignition::math::Pose3d(ignition::math::Vector3d(0,1,2), ignition::math::Quaterniond(0,0,1,0));

    auto res = utilities::InterpolatePose(0.5,0,1,pose1,pose2);
    std::cout << res.Rot().X() << ", " << res.Rot().Y() << ", " << res.Rot().Z() << ", " << res.Rot().W() << std::endl;
    
    
    return 0;
}
