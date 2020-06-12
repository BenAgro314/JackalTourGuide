#pragma once 

#include "quadtree.hh"
#include "frame.hh"
#include <string>
#include <vector>

class Classifier{

    private:

        std::vector<Frame> input_frames;

        std::string filename;

        std::string username;

        std::string filepath;

        std::vector<TrajPoint> robot_trajectory;


    public:

        Classifier(std::string filename, std::string username);

        void Load();


};