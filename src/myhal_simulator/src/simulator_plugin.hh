#ifndef SIMULATOR_PLUGIN_HH
#define SIMULATOR_PLUGIN_HH

#include <functional>
#include "world_entities.hh"
#include <utility>
#include <ros/ros.h>

class RoomInfo{

    public:
        std::shared_ptr<myhal::Room> room;
        std::string scenario;
        std::vector<double> positions;

        RoomInfo(std::shared_ptr<myhal::Room> _room, std::string _scenario, std::vector<double> _positions):
        room(_room), scenario(_scenario), positions(_positions){}

};

class ModelInfo{

    public: 
        std::string name;
        std::string filename;

        ModelInfo(std::string _name, std::string _filename) : name(_name), filename(_filename)
        {}
};

class ActorInfo: public ModelInfo{

    public: 
        std::string plugin;
        double obstacle_margin;

        ActorInfo(std::string _name, std::string _filename, std::string _plugin, double _obstacle_margin):
        ModelInfo(_name,_filename), plugin(_plugin), obstacle_margin(_obstacle_margin){}
};

class Scenario{

    protected:

        std::vector<std::shared_ptr<ModelInfo>> models;

    public:

        double pop_density;
        double model_percentage;
        std::string actor;
        

        Scenario(double _pop_denisty, double _model_percentage, std::string _actor);

        void AddModel(std::shared_ptr<ModelInfo> model);

        std::shared_ptr<ModelInfo> GetRandomModel();
};

class WorldHandler: public gazebo::WorldPlugin{

    private: 

        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

        void OnUpdate(const gazebo::common::UpdateInfo &_info);

        void LoadParams();

        void FillRoomModels(std::shared_ptr<RoomInfo> room_info);

        void FillRoomActors(std::shared_ptr<RoomInfo> room_info);

        gazebo::event::ConnectionPtr update_connection;


        gazebo::physics::WorldPtr world;

        sdf::ElementPtr sdf;

        gazebo::physics::ModelPtr building;


        

        //std::vector<std::shared_ptr<SDFAnimation>> actor_animations;

        int tick = 0;

        // filled by parameters 

        std::map<std::string, std::shared_ptr<SDFPlugin>> vehicle_plugins; //one per actor
        std::vector<std::shared_ptr<SDFAnimation>> animation_list; //added to all actors 
        std::map<std::string, std::shared_ptr<ModelInfo>> model_info;
        std::map<std::string, std::shared_ptr<Scenario>> scenarios;
        std::map<std::string , std::shared_ptr<ActorInfo>> actor_info;

        std::vector<std::shared_ptr<RoomInfo>> rooms;
        //std::map<std::string , std::shared_ptr<myhal::Room>> rooms;
};


#endif