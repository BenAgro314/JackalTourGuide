#include "vehicle_plugin.hh"
#include <functional>
#include <ignition/math/Rand.hh>
#include <iostream>
#include <sstream>
#include <stdlib.h> 

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ModelHandler)

void ModelHandler::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    this->ReadSDF(_sdf);

    this->ReadParams();

    this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelHandler::OnUpdate, this, std::placeholders::_1));

    double mass = this->vehicle_params[0];
    double max_force = this->vehicle_params[1];
    double max_speed = this->vehicle_params[2];
    double slowing_distance = this->vehicle_params[3];
    double arrival_distance = this->vehicle_params[4];
    double obstacle_margin = this->vehicle_params[5];

    if (this->vehicle_type == "boid"){
        
        ignition::math::Vector3d random_vel = ignition::math::Vector3d(ignition::math::Rand::DblUniform(-1,1),ignition::math::Rand::DblUniform(-1,1),0);
        random_vel.Normalize(); 
        random_vel*=this->max_speed;
        
        double ali = this->boid_params[0];
        double coh = this->boid_params[1];
        double sep = this->boid_params[2];
        double FOV_angle = this->boid_params[3];
        double FOV_radius = this->boid_params[4];

        this->vehicle = std::make_unique<Boid>(boost::dynamic_pointer_cast<physics::Actor>(_parent), 
        mass, 
        max_force, 
        max_speed, 
        _parent->WorldPose(), 
        random_vel, 
        this->animation, 
        this->building, 
        ali,
        coh,
        sep,
        FOV_angle,
        FOV_radius);

        

    } else if (this->vehicle_type == "random_walker"){
        
        this->vehicle = std::make_unique<RandomWalker>(boost::dynamic_pointer_cast<physics::Actor>(_parent), 
        mass, 
        max_force, 
        max_speed, 
        _parent->WorldPose(), 
        ignition::math::Vector3d(0,0,0), 
        this->animation, 
        this->building);

    } else if (this->vehicle_type == "path_follower"){

        this->ReadPath(_sdf);

        
        this->vehicle = std::make_unique<PathFollower>(boost::dynamic_pointer_cast<physics::Actor>(_parent), 
        mass, 
        max_force, 
        max_speed, 
        _parent->WorldPose(), 
        ignition::math::Vector3d(0,0,0), 
        this->animation, 
        this->building,
        this->path);
        
    } else if (this->vehicle_type == "stander"){
        
        this->vehicle = std::make_unique<Stander>(boost::dynamic_pointer_cast<physics::Actor>(_parent), 
        mass, 
        max_force, 
        max_speed, 
        _parent->WorldPose(), 
        ignition::math::Vector3d(0,0,0), 
        this->animation, 
        this->building,
        5,
        5); //todo: read these in as parameters 
        
    } else{
        //wanderer
        
        this->vehicle = std::make_unique<Wanderer>(boost::dynamic_pointer_cast<physics::Actor>(_parent), 
        mass, 
        max_force, 
        max_speed, 
        _parent->WorldPose(), 
        ignition::math::Vector3d(0,0,0), 
        this->animation, 
        this->building);

    }

    
    

}

void ModelHandler::ReadSDF(sdf::ElementPtr _sdf){

    if (_sdf->HasElement("vehicle_type")){
        this->vehicle_type =_sdf->GetElement("vehicle_type")->Get<std::string>();
    }

    if (_sdf->HasElement("building")){
        this->building =_sdf->GetElement("building")->Get<std::string>();
    }

    if (_sdf->HasElement("max_speed")){
        this->max_speed =_sdf->GetElement("max_speed")->Get<double>();
    }
  
}

void ModelHandler::ReadPath(sdf::ElementPtr _sdf){

    this->path = std::make_shared<utilities::Path>();
    
    if (_sdf->HasElement("path_point")){
		auto point_elem = _sdf->GetElement("path_point");
        
		while (point_elem){
			this->path->AddPoint(point_elem->Get<ignition::math::Vector3d>());
            
			point_elem = point_elem->GetNextElement("path_point");
		}
	}
    
}

void ModelHandler::ReadParams(){
    std::string line;
    std::ifstream param_file("/home/default/catkin_ws/src/model_plugins/src/params/vehicle_params.txt");
    if (param_file.is_open()){

        while (std::getline(param_file, line)){
            if (line == "Vehicle:"){
                std::getline(param_file, line);
                std::getline(param_file, line);
                std::stringstream s_stream(line);

                while(s_stream.good()) {
                    std::string substr;
                    std::getline(s_stream, substr, ','); //get first string delimited by comma
                    this->vehicle_params.push_back(std::stod(substr));
                    //std::cout << std::stod(substr) <<std::endl;
                }
            }

            if (line == "Boid:"){
                std::getline(param_file, line);
                std::getline(param_file, line);
                std::stringstream s_stream(line);

                while(s_stream.good()) {
                    std::string substr;
                    std::getline(s_stream, substr, ','); //get first string delimited by comma
                    this->boid_params.push_back(std::stod(substr));
                    //std::cout << std::stod(substr) <<std::endl;
                }
            }
        }

        param_file.close();

    } else{
        std::cout << "Unable to open param file" << std::endl;
    }
}

void ModelHandler::OnUpdate(const common::UpdateInfo &_info){
    this->vehicle->OnUpdate(_info);
    
}