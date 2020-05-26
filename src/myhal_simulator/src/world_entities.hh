#ifndef WORLD_ENTITIES_HH
#define WORLD_ENTITIES_HH

#include <vector>
#include <string>
#include "sdfstring.hh"
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace myhal{


    class Model{

        protected:

            static int num_models;

            std::string name;
            ignition::math::Pose3d pose;
            std::string model_file;
            std::vector<std::shared_ptr<SDFPlugin>> plugins;

        public:

            Model(std::string _name, ignition::math::Pose3d _pose, std::string _model_file);

            void AddPlugin(std::shared_ptr<SDFPlugin> plugin);

            std::string CreateSDF();

            void InsertIntoWorld(gazebo::physics::WorldPtr _world);

    };

    class Actor: public Model{

        private:
            
            std::vector<std::shared_ptr<SDFAnimation>> animations;

        public:

            using Model::Model;
            
            void AddAnimation(std::shared_ptr<SDFAnimation> animation);

            std::string CreateSDF();

            void InsertIntoWorld(gazebo::physics::WorldPtr _world);
    };

}

#endif