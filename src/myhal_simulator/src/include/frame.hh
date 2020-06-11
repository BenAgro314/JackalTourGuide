#ifndef FRAME_HH
#define FRAME_HH

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <vector>
#include "happily.h"

class Point{

    private:

        ignition::math::Vector3d pos;
        char cat;

    public:

        double X(){
            return this->pos.X();
        }

        double Y(){
            return this->pos.Y();
        }

        double Z(){
            return this->pos.Z();
        }

        char Cat(){
            return this->cat;
        }

        Point(ignition::math::Vector3d pos, char cat): pos(pos), cat(cat){};
};

void addPoints(happly::PLYData &plyOut, std::vector<Point>& vertexPositions){

    std::string vertexName = "vertex";
    size_t N = vertexPositions.size();

    // Create the element
    if (!plyOut.hasElement(vertexName)) {
    plyOut.addElement(vertexName, N);
    }

    // De-interleave
    std::vector<float> xPos(N);
    std::vector<float> yPos(N);
    std::vector<float> zPos(N);
    std::vector<char> cat(N);
    for (size_t i = 0; i < vertexPositions.size(); i++) {
    xPos[i] = (float) vertexPositions[i].X();
    yPos[i] = (float) vertexPositions[i].Y();
    zPos[i] = (float) vertexPositions[i].Z();
    cat[i] = vertexPositions[i].Cat();
    }

    // Store
    plyOut.getElement(vertexName).addProperty<float>("x", xPos);
    plyOut.getElement(vertexName).addProperty<float>("y", yPos);
    plyOut.getElement(vertexName).addProperty<float>("z", zPos);
    plyOut.getElement(vertexName).addProperty<char>("category", cat);
}

void addPose(happly::PLYData &plyOut, ignition::math::Pose3d pose){

    plyOut.addElement("gt_pose", 1);
    plyOut.getElement("gt_pose").addProperty<float>("pos.x", {(float) pose.Pos().X()});
    plyOut.getElement("gt_pose").addProperty<float>("pos.y", {(float) pose.Pos().Y()});
    plyOut.getElement("gt_pose").addProperty<float>("pos.z", {(float) pose.Pos().Z()});
    plyOut.getElement("gt_pose").addProperty<double>("rot.x", {pose.Rot().X()});
    plyOut.getElement("gt_pose").addProperty<double>("rot.y", {pose.Rot().Y()});
    plyOut.getElement("gt_pose").addProperty<double>("rot.z", {pose.Rot().Z()});
    plyOut.getElement("gt_pose").addProperty<double>("rot.w", {pose.Rot().W()});
}

class Frame{

    private:

        std::vector<Point> points;
        ignition::math::Pose3d gt_pose;
        double time;

    public:

        Frame(ignition::math::Pose3d gt_pose, double time): gt_pose(gt_pose), time(time){};

        void AddPoint(ignition::math::Vector3d pos, char cat){
            this->points.push_back(Point(pos,cat));
        }

        void WriteToFile(std::string path){
            happly::PLYData plyOut;
            addPose(plyOut, this->gt_pose);
            addPoints(plyOut, this->points);
            plyOut.write(path + std::to_string(this->time) + ".ply", happly::DataFormat::Binary);
        }
    
};

#endif