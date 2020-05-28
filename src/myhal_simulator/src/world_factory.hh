#ifndef WORLD_FACTORY_HH
#define  WORLD_FACTORY_HH

#include <functional>
#include "world_entities.hh"
#include <utility>
#include <ros/ros.h>

class RoomInfo{

    public:
        std::shared_ptr<myhal::Room> room;
        std::string scenario;
        std::vector<std::vector<double>> positions;

        RoomInfo(std::shared_ptr<myhal::Room> _room, std::string _scenario, std::vector<std::vector<double>> _positions):
        room(_room), scenario(_scenario), positions(_positions){}

};

class ModelInfo{

    public: 
        std::string name;
        std::string filename;
        double width;
        double length;

        ModelInfo(std::string _name, std::string _filename, double _width, double _length) : name(_name), filename(_filename), width(_width), length(_length)
        {}
};

class ActorInfo: public ModelInfo{

    public: 
        std::string plugin;

        ActorInfo(std::string _name, std::string _filename, std::string _plugin, double _obstacle_margin):
        ModelInfo(_name,_filename, _obstacle_margin, _obstacle_margin), plugin(_plugin){}
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

class WorldHandler{

    public: 
        WorldHandler();

        void Load();

        void LoadParams();

        void WriteToFile(std::string out_name);

        void FillRoom(std::shared_ptr<RoomInfo> room_info);

        std::string world_string;

        std::vector<std::shared_ptr<myhal::Model>> world_models;

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