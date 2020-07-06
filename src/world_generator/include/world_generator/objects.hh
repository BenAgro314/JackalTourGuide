#pragma once 

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ignition/math/Pose3.hh>
#include "math_utils.hh"

namespace objects{

class Object{

    protected:

        static int obj_count;

        std::string name;

        ignition::math::Pose3d pose;

        gazebo::physics::ModelPtr world_model = nullptr;

    public:

        Object(std::string name, ignition::math::Pose3d pose);

        virtual boost::shared_ptr<sdf::SDF> GetSDF();

        void AddToWorld(gazebo::physics::WorldPtr world);

        std::string Name();

        gazebo::physics::ModelPtr &Model();

};

class Model: public Object{

    protected:

        std::string uri;

    public:

        Model(std::string name, ignition::math::Pose3d pose, std::string uri);

        boost::shared_ptr<sdf::SDF> GetSDF();
};

class Actor: public Object{

    protected:

        std::string type;

    public: 

        Actor(std::string name, ignition::math::Pose3d initial_pose, std::string type);

        boost::shared_ptr<sdf::SDF> GetSDF();

};

class Boxes: public Object{

    protected:

        std::vector<ignition::math::Box> boxes;

    public:

        Boxes(std::string name);

        void AddBox(ignition::math::Box box);

        boost::shared_ptr<sdf::SDF> GetSDF();
};

// class Box: public Object{

//     protected:

//         ignition::math::Box box;

//     public:

//         Box(ignition::math::Box box, std::string name = "box");

//         boost::shared_ptr<sdf::SDF> GetSDF();

// };

}