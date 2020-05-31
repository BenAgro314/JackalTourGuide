#ifndef PUPPETEER_HH
#define PUPPETEER_HH

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <vector>
#include <map>
#include <utility>
#include "quadtree.hh"
#include <string>
#include <ros/ros.h>
#include "vehicles.hh"


class Puppeteer: public gazebo::WorldPlugin{

    private:

        gazebo::event::ConnectionPtr update_connection;

        gazebo::physics::WorldPtr world;

        sdf::ElementPtr sdf;

        std::vector<boost::shared_ptr<Vehicle>> vehicles;

        std::vector<gazebo::physics::EntityPtr> collision_entities;

        std::string building_name;

        gazebo::physics::EntityPtr building; 

        double update_freq = 60;

        gazebo::common::Time last_update;

        std::map<std::string, double> vehicle_params;

        std::map<std::string, double> boid_params;
        
        boost::shared_ptr<QuadTree> static_quadtree; 
        boost::shared_ptr<QuadTree> vehicle_quadtree; 

        ignition::math::Box building_box; 

        std::vector<boost::shared_ptr<Follower>> follower_queue;

    public: 
        
        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

        void OnUpdate(const gazebo::common::UpdateInfo &_info);

        void ReadSDF();

        void ReadParams();

        boost::shared_ptr<Vehicle> CreateVehicle(gazebo::physics::ActorPtr actor);

};


#endif