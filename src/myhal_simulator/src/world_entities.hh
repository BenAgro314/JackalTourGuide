#ifndef WORLD_ENTITIES_HH
#define WORLD_ENTITIES_HH

#include <vector>
#include <string>
#include "sdfstring.hh"
#include <ignition/math/Pose3.hh>

namespace myhal{


    class Model{

        protected:

            static int num_models;

            std::string name;
            ignition::math::Pose3d pose;
            std::vector<std::shared_ptr<SDFPlugin>> plugins;

        public:

            Model(std::string _name, ignition::math::Pose3d _pose);

            void AddPlugin(std::shared_ptr<SDFPlugin> plugin);

            virtual std::string CreateSDF();

    };

    class Actor: public Model{

        private:

            
            std::string skin_file;
            
            std::vector<std::shared_ptr<SDFAnimation>> animations;

        public:

            Actor(std::string _name, ignition::math::Pose3d _pose, std::string _skin_file);
            
            void AddAnimation(std::shared_ptr<SDFAnimation> animation);

            std::string CreateSDF();
    };

}

#endif