#ifndef VEHICLES_HH
#define VEHICLES_HH
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Box.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <vector>
#include <map>
#include <utility>
#include <string>
#include "utilities.hh"

#define ALI 0 
#define COH 1
#define SEP 2


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

        double rand_angle_offset = ignition::math::Rand::DblUniform(-2.5,2.5);

        double rand_radius = ignition::math::Rand::DblUniform(2,2.5);

        gazebo::physics::EntityPtr leader;

        std::string leader_name; 

        ignition::math::Pose3d last_leader_pose;

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

        void LoadLeader(gazebo::physics::EntityPtr leader);
};

class FlowField{

    private:

        std::vector<std::vector<ignition::math::Vector3d>> field;

        int rows;

        int cols;

        int resolution;

        ignition::math::Box rect;

    public:

        FlowField(double min_x, double min_y, double width, double height, int resolution);

        bool Lookup(ignition::math::Vector3d pos, ignition::math::Vector3d &res);
};

class FlowFollower: public Wanderer{

    protected:

        std::vector<FlowField> fields;

        bool Follow();

    public: 
        
        FlowFollower(gazebo::physics::ActorPtr _actor,
         double _mass,
         double _max_force, 
         double _max_speed, 
         ignition::math::Pose3d initial_pose, 
         ignition::math::Vector3d initial_velocity, 
         std::vector<gazebo::physics::EntityPtr> objects, 
         std::vector<FlowField> _fields);

        void OnUpdate(const gazebo::common::UpdateInfo &_info, double dt, std::vector<boost::shared_ptr<Vehicle>> vehicles, std::vector<gazebo::physics::EntityPtr> objects);
};


#endif