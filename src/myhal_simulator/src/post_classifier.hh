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

        std::vector<Frame> stander_frames;

        std::string filename;

        std::string username;

        std::string filepath;

        std::vector<TrajPoint> robot_trajectory;

        std::vector<BoxObject> static_objects;

        boost::shared_ptr<QuadTree> static_quadtree;

        boost::shared_ptr<QuadTree> active_quadtree;

        ignition::math::Box qt_box;

        bool translate;

    public:

        Classifier(std::string filename, std::string username, bool translate);

        void Load();

        void WriteToPLY();

};

