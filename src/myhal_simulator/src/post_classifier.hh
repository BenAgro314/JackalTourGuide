#pragma once 

#include "quadtree.hh"
#include "frame.hh"
#include <string>
#include <vector>
#include "data_processing.hh"
#include "utilities.hh"
#include <utility>


class Classifier{

    private:

        std::vector<Frame> input_frames;

        std::vector<Frame> output_frames;

        std::string filename;

        std::string username;

        std::string filepath;

        std::vector<TrajPoint> robot_trajectory;

        std::vector<BoxObject> static_objects;

        boost::shared_ptr<QuadTree> static_quadtree;

        bool colorize;

    public:

        Classifier(std::string filename, std::string username, bool colorize);

        void Load();

        void WriteToPLY();

};

