#ifndef WORLD_ENTITIES_HH
#define WORLD_ENTITIES_HH

#include <vector>
#include <string>
#include "sdfstring.hh"
#include <ignition/math/Pose3.hh>
#include <ignition/math/Box.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace myhal{


    class Model{

        protected:

            static int num_models;

            std::string name;
            
            std::string model_file;
            std::vector<std::shared_ptr<SDFPlugin>> plugins;

        public:

            ignition::math::Pose3d pose;

            Model(std::string _name, ignition::math::Pose3d _pose, std::string _model_file);

            void AddPlugin(std::shared_ptr<SDFPlugin> plugin);

            virtual std::string CreateSDF();

            void AddToWorld(gazebo::physics::WorldPtr _world);

    };

    class Actor: public Model{

        private:
            
            std::vector<std::shared_ptr<SDFAnimation>> animations;

        public:

            using Model::Model;
            
            void AddAnimation(std::shared_ptr<SDFAnimation> animation);

            std::string CreateSDF();

    };

    class IncludeModel: public Model{

        public:

            using Model::Model;

            std::string CreateSDF();

    };


    class BoundaryBox: public Model{

        public:

            double width,length;
            
            BoundaryBox(double _x, double _y, double _width, double _length);

            std::string CreateSDF();
    };

    class ModelGroup{ //used for tables 

        public:

            std::vector<std::shared_ptr<IncludeModel>> group; 
            std::shared_ptr<IncludeModel> center;

            ModelGroup(std::string _name, ignition::math::Pose3d _pose, std::string _model_file);

            void AddObject(std::string _name, ignition::math::Pose3d _pose, std::string _model_file);

            ignition::math::Pose3d GetCenterPose();

    };

    class Room{

        protected:

            ignition::math::Box boundary; 
            std::vector<std::shared_ptr<Model>> models;
            bool enclosed;

        public: 

            Room(double x_min, double y_min, double x_max, double y_max, bool _enclosed);

            void AddModel(std::shared_ptr<Model> model);

            void AddToWorld(gazebo::physics::WorldPtr _world);
    };

}

#endif