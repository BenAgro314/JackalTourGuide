#pragma once 

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ignition/math/Pose3.hh>

namespace objects{

class Object{

    protected:

        static int obj_count;

        std::string name;

        ignition::math::Pose3d pose;

    public:

        Object(std::string name, ignition::math::Pose3d pose);

        virtual boost::shared_ptr<sdf::SDF> GetSDF();

        void AddToWorld(gazebo::physics::WorldPtr world);

};

class Boxes: public Object{

    protected:

        std::vector<ignition::math::Box> boxes;

    public:

        Boxes(std::string name);

        void AddBox(ignition::math::Box box);

        boost::shared_ptr<sdf::SDF> GetSDF();
};

class Box: public Object{

    protected:

        ignition::math::Box box;

    public:

        Box(ignition::math::Box box, std::string name = "box");

        boost::shared_ptr<sdf::SDF> GetSDF();

};

}