#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <vector>
#include <map>

#define ALI 0 
#define COH 1
#define SEP 2

class Vehicle{

    protected: 

        //Simple Vehicle Parameters 
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

        double slowing_distance = 2;
        double arrival_distance = 0.5;
        double obstacle_margin = 0.5;
        
        std::string building_name;

        std::vector<gazebo::physics::EntityPtr> objects;
        std::vector<gazebo::physics::ActorPtr> actors;
        int initial_model_count;

    public:

        Vehicle(gazebo::physics::ActorPtr _actor, double _mass, double _max_force, double _max_speed, ignition::math::Pose3d initial_pose, ignition::math::Vector3d initial_velocity, std::string animation, std::string _building_name);

        virtual void OnUpdate(const gazebo::common::UpdateInfo &_inf);

    protected:

        void ApplyForce(ignition::math::Vector3d force);

        void Seek(ignition::math::Vector3d target);

        void Arrival(ignition::math::Vector3d target);

        void UpdatePosition(double dt);

        void UpdateModel();

        ignition::math::Vector3d curr_target;

        void AvoidActors(); 

        void AvoidObstacles();

    public: 

        void SetSlowingDistance(double _slowing_distance){
            this->slowing_distance = _slowing_distance;
        }

        void SetArrivalDistance(double _arrival_distance){
            this->arrival_distance = _arrival_distance;
        }

        void SetObstacleMargin(double _obstacle_margin){
            this->obstacle_margin = _obstacle_margin;
        }
};

class Wanderer: public Vehicle{

    private:

        double curr_theta =0;
        double rand_amp = 0.4;

    protected:
        void SetNextTarget(); 

    public:
        using Vehicle::Vehicle;

        void OnUpdate(const gazebo::common::UpdateInfo &_inf);

        void SetRandAmplitude(double angle){
            this->rand_amp = angle;
        }

};


class RandomWalker: public Vehicle{


    protected:

        void SetNextTarget(); 

        

    public:

        using Vehicle::Vehicle;

        void OnUpdate(const gazebo::common::UpdateInfo &_inf);

};


class Boid: public Vehicle{

    protected:

        void Separation();
        void Alignement(double dt);
        void Cohesion();

        double weights[3];

        double FOV_angle = 4;
        double FOV_radius = 3;

        std::map<gazebo::physics::ActorPtr, ignition::math::Vector3d> last_pos;
        

    public:

        Boid(gazebo::physics::ActorPtr _actor, double _mass, double _max_force, double _max_speed, ignition::math::Pose3d initial_pose, ignition::math::Vector3d initial_velocity, std::string animation, std::string _building_name, double _alignement, double _cohesion, double _separation);

        void OnUpdate(const gazebo::common::UpdateInfo &_inf);

        void SetWeights(double _separation, double _alignement, double _cohesion){
            this->weights[ALI] = _alignement;
            this->weights[COH] = _cohesion;
            this->weights[SEP] = _separation;
        }

        void SetFOV(double _angle, double _radius){
            this->FOV_angle = _angle;
            this->FOV_radius = _radius;
        }
};