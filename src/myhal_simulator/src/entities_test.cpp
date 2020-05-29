#include "world_entities.hh"
#include <iterator>
#include <algorithm>   
#include <ctime>        
#include <cstdlib> 
#include <iostream>
#include <fstream>


int main(int argc, char ** argv){

    auto box_model = std::make_shared<myhal::BoundaryBox>(0,0,10,5);

    auto other = std::make_shared<myhal::BoundaryBox>(11,11,1,1);

    box_model->Reposition(5,5);
    box_model->Reposition(0,0);

    box_model->RotateClockwise(0.7853);
    box_model->RotateClockwise(0.7853);

    auto col_box = box_model->GetCollisionBox();
    auto col_box2 = other->GetCollisionBox();

    std::printf("pose: (%f, %f, %f)(%f, %f, %f)\n", box_model->pose.Pos().X(),box_model->pose.Pos().Y(),box_model->pose.Pos().Z(), box_model->pose.Rot().Roll(),box_model->pose.Rot().Pitch(),box_model->pose.Rot().Yaw());
    std::printf("collision box: (%f, %f, %f)  (%f, %f, %f)\n", col_box.Min().X(),col_box.Min().Y(), col_box.Min().Z(), col_box.Max().X(),col_box.Max().Y(), col_box.Max().Z());
    std::printf("other box: (%f, %f, %f)  (%f, %f, %f)\n", col_box2.Min().X(),col_box2.Min().Y(), col_box2.Min().Z(), col_box2.Max().X(),col_box2.Max().Y(), col_box2.Max().Z());
    std::cout << col_box.Intersects(col_box2) << std::endl;
    std::cout << other->DoesCollide(box_model) << std::endl;
    return 0;
}