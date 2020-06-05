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


void print_nodes(std::vector<QTData> list);

int main(int argc, char ** argv){


    auto box = ignition::math::Box(ignition::math::Vector3d(10,0,0), ignition::math::Vector3d(11,10,5));
    auto pos = ignition::math::Vector3d(1,1,1);

    std::cout << utilities::dist_to_box(pos,box) << std::endl;

    /*
    std::printf("hello world\n");

    FlowField F = FlowField(ignition::math::Vector3d(0,0,0),10,10,1);

    F.PrintField();

    int r;
    int c;

    F.PosToIndicies(ignition::math::Vector3d(9.5,-9.5,0), r, c );
    std::printf("%d %d\n", r, c);
    */
    
    return 0;
}

void print_nodes(std::vector<QTData> list){
    for (auto node : list){
        auto box = node.box;
        std::printf("node box: (%f, %f) -> (%f, %f)\n", box.Min().X(), box.Min().Y(), box.Max().X(), box.Max().Y());
    }
}