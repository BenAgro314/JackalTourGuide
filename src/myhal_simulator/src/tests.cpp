#include "world_entities.hh"
#include "quadtree.hh"
#include "puppeteer.hh"
#include <iterator>
#include <algorithm>   
#include <ctime>        
#include <cstdlib> 
#include <iostream>
#include <fstream>


void print_nodes(std::vector<QTData> list);

int main(int argc, char ** argv){
    /*

    std::cout << "Helloworld\n";

    auto test_data = std::make_shared<std::string>("data");

    auto n1 = QTData(ignition::math::Box(ignition::math::Vector3d(1,1,0), ignition::math::Vector3d(9,9,0)), test_data, string_type);
    auto n2 = QTData(ignition::math::Box(ignition::math::Vector3d(1,1,0), ignition::math::Vector3d(2,2,0)), test_data, string_type);
    auto n3 = QTData(ignition::math::Box(ignition::math::Vector3d(1,1,0), ignition::math::Vector3d(2,9,0)), test_data, string_type);

    auto Q = std::make_shared<QuadTree>(ignition::math::Box(ignition::math::Vector3d(0,0,0), ignition::math::Vector3d(10,10,0)));
    Q->Insert(n1);
    Q->Insert(n2);
    Q->Insert(n3);

    Q->Print();

    print_nodes(Q->QueryRange(ignition::math::Box(ignition::math::Vector3d(5,0,0), ignition::math::Vector3d(10,10,0))));
    */
    return 0;
}

void print_nodes(std::vector<QTData> list){
    for (auto node : list){
        auto box = node.box;
        std::printf("node box: (%f, %f) -> (%f, %f)\n", box.Min().X(), box.Min().Y(), box.Max().X(), box.Max().Y());
    }
}