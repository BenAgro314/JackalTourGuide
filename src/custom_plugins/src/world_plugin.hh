#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

using namespace gazebo; 


class Factory : public WorldPlugin{
    public: void Load(physics::WorldPtr world, sdf::ElementPtr _sdf);

    public: event::ConnectionPtr updateConnection;

    public: physics::WorldPtr world;
    public: sdf::ElementPtr sdf;

    public: void OnUpdate(const gazebo::common::UpdateInfo &_info);

};