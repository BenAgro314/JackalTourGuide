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

class Vehicle;

class Puppeteer: public gazebo::WorldPlugin{

    private:

        gazebo::event::ConnectionPtr update_connection;

        gazebo::physics::WorldPtr world;

        sdf::ElementPtr sdf;

        std::vector<std::shared_ptr<Vehicle>> vehicles;

        std::vector<gazebo::physics::EntityPtr> collision_entities;

        std::string building_name;

        gazebo::physics::EntityPtr building; 

        double update_freq = 60;

        gazebo::common::Time last_update;

    public: 
        
        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

        void OnUpdate(const gazebo::common::UpdateInfo &_info);

        void ReadSDF();

        std::shared_ptr<Vehicle> CreateVehicle(gazebo::physics::ActorPtr actor);

};

class Vehicle{

    protected:

        double mass;

        double max_force;

        double max_speed;

        ignition::math::Pose3d pose;

        ignition::math::Vector3d velocity;

        ignition::math::Vector3d acceleration;

        gazebo::physics::ActorPtr actor;

        ignition::math::Vector3d curr_target;

        std::map<std::string, std::shared_ptr<gazebo::physics::TrajectoryInfo>> trajectories; 

        double animation_factor = 5.1;

        double obstacle_margin = 0.5;

        void UpdateModel();

        void ApplyForce(ignition::math::Vector3d force);

        void Seek(ignition::math::Vector3d target);

        void UpdatePosition(double dt);

        void AvoidActors(std::vector<std::shared_ptr<Vehicle>> vehicles); 

        void AvoidObstacles(std::vector<gazebo::physics::EntityPtr> objects);

    public:

        Vehicle(gazebo::physics::ActorPtr _actor, 
        double _mass, 
        double _max_force, 
        double _max_speed, 
        ignition::math::Pose3d initial_pose, 
        ignition::math::Vector3d initial_velocity);

        virtual void OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<std::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects); //to be made virtual

        gazebo::physics::ActorPtr GetActor();
};

class Wanderer: public Vehicle{

    protected:

        double curr_theta =0;

        double rand_amp = 0.4;

        void SetNextTarget(); 

    public:
        using Vehicle::Vehicle;

        void OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<std::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects);

};

#endif