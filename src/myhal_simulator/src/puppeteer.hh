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

#define ALI 0 
#define COH 1
#define SEP 2

class Vehicle;
class Follower;

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

        double animation_factor = 5.1; //TODO: read these in as common vehicle parameters 

        double obstacle_margin = 0.5;

        double slowing_distance = 1;

        double arrival_distance  = 0.5;

        std::vector<gazebo::physics::EntityPtr> all_objects;

        void UpdateModel();

        void ApplyForce(ignition::math::Vector3d force);

        void Seek(ignition::math::Vector3d target, double weight = 1);

        void Arrival(ignition::math::Vector3d target, double weight = 1);

        void UpdatePosition(double dt);

        void AvoidActors(std::vector<boost::shared_ptr<Vehicle>> vehicles); 

        void AvoidObstacles(std::vector<gazebo::physics::EntityPtr> objects);


    public:

        Vehicle(gazebo::physics::ActorPtr _actor, 
        double _mass, 
        double _max_force, 
        double _max_speed, 
        ignition::math::Pose3d initial_pose, 
        ignition::math::Vector3d initial_velocity,
        std::vector<gazebo::physics::EntityPtr> all_objects);

        virtual void OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects); //to be made virtual

        gazebo::physics::ActorPtr GetActor();

        void SetAllObjects(std::vector<gazebo::physics::EntityPtr> objects);

        ignition::math::Pose3d GetPose();

        ignition::math::Vector3d GetVelocity();

        std::string GetName();
};

class Wanderer: public Vehicle{

    protected:

        double curr_theta =0;

        double rand_amp = 0.4;

        void SetNextTarget(); 

    public:
        using Vehicle::Vehicle;

        void OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects);

};

class RandomWalker: public Vehicle{


    protected:

        void SetNextTarget(std::vector<gazebo::physics::EntityPtr> objects); 

    public:

        using Vehicle::Vehicle;

        void OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects);

};

class Boid: public Vehicle{

    protected:

        double weights[3];

        double FOV_angle = 4;

        double FOV_radius = 3;

        void Separation(std::vector<boost::shared_ptr<Vehicle>> vehicles);

        void Alignement(double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles);
        
        void Cohesion(std::vector<boost::shared_ptr<Vehicle>> vehicles);
        

    public:

        Boid(gazebo::physics::ActorPtr _actor,
         double _mass,
         double _max_force, 
         double _max_speed, 
         ignition::math::Pose3d initial_pose, 
         ignition::math::Vector3d initial_velocity, 
         std::vector<gazebo::physics::EntityPtr> objects, 
         double _alignement, 
         double _cohesion, 
         double _separation, 
         double angle, 
         double radius);

        void OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects);
};

class Stander: public Wanderer{

    protected: 

        bool standing = true;

        bool never_walk = false;

        double standing_duration;

        double walking_duration;

        gazebo::common::Time standing_start;

        gazebo::common::Time walking_start;

        void UpdateModel(double dt);

    public:

        Stander(gazebo::physics::ActorPtr _actor,
         double _mass,
         double _max_force, 
         double _max_speed, 
         ignition::math::Pose3d initial_pose, 
         ignition::math::Vector3d initial_velocity,  
         std::vector<gazebo::physics::EntityPtr> objects, 
         double _standing_duration,
         double _walking_duration);


        void OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects);

};

class Sitter: public Vehicle{

    protected:
        
        std::string chair_name;

        gazebo::physics::EntityPtr chair;

        void UpdateModel(double dt);


    public:
        
        Sitter(gazebo::physics::ActorPtr _actor, std::string chair_name, std::vector<gazebo::physics::EntityPtr> objects, double height = 0.65);

        void OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects);

};

class Follower: public Vehicle{

    protected:

        double rand_angle = ignition::math::Rand::DblUniform(0,6.28);
        std::string leader_name;
        boost::shared_ptr<Vehicle> leader;
        void SetNextTarget(double dt);

    public:

        Follower(gazebo::physics::ActorPtr _actor,
         double _mass,
         double _max_force, 
         double _max_speed, 
         ignition::math::Pose3d initial_pose, 
         ignition::math::Vector3d initial_velocity, 
         std::vector<gazebo::physics::EntityPtr> objects, 
         std::string _leader_name);

        void OnUpdate(const gazebo::common::UpdateInfo &_info , double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects);

        void LoadLeader(std::vector<boost::shared_ptr<Vehicle>> vehicles);
};





#endif