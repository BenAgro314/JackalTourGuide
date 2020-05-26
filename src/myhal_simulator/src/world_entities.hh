#ifndef WORLD_ENTITIES_HH
#define WORLD_ENTITIES_HH

#include <vector>
#include <string>
#include "sdfstring.hh"
#include <ignition/math/Pose3.hh>

namespace myhal{

    class Actor{

        private:

            static int num_actors;

            std::string name;
            ignition::math::Pose3d pose;
            std::string skin_file;

            std::vector<std::shared_ptr<SDFPlugin>> plugins;
            std::vector<std::shared_ptr<SDFAnimation>> animations;

        public:

            Actor(std::string _name, ignition::math::Pose3d _pose, std::string _skin_file);

            void AddPlugin(std::shared_ptr<SDFPlugin> plugin);
            void AddAnimation(std::shared_ptr<SDFAnimation> animation);

            std::string CreateSDF();
    };

}

#endif