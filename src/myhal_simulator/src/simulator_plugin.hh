#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

class WorldHander: public gazebo::WorldPlugin{

    
    public: void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr _sdf);

    public: void OnUpdate(const gazebo::common::UpdateInfo &_info);

    public: gazebo::event::ConnectionPtr update_connection;

    public: gazebo::physics::WorldPtr world;
    public: sdf::ElementPtr sdf;

    

};