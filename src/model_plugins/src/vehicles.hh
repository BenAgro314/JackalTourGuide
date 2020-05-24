#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

class Vehicle{

    protected: 

        double mass;
        double max_force;
        double max_speed;
        ignition::math::Pose3d pose;
        ignition::math::Vector3d velocity;
        ignition::math::Vector3d acceleration;
        gazebo::physics::ActorPtr actor;
        
        double update_freq = 60;
        gazebo::common::Time last_update;
        double animation_factor{5.1};


    public:

        

        Vehicle(gazebo::physics::ActorPtr _actor, double _mass, double _max_force,double _max_speed, ignition::math::Pose3d initial_pose, ignition::math::Vector3d initial_velocity, std::string animation);

        void OnUpdate(const gazebo::common::UpdateInfo &_inf);

    protected:

        void ApplyForce(ignition::math::Vector3d force);

        void Seek(ignition::math::Vector3d target);

        void UpdatePosition(double dt);

        void UpdateModel();
};