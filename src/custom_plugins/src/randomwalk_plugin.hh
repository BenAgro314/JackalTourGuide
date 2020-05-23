

#ifndef SERVICESIM_PLUGINS_WANDERINGACTORPLUGIN_HH_
#define SERVICESIM_PLUGINS_WANDERINGACTORPLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"


namespace servicesim
{
  class ActorPluginPrivate;

  class GAZEBO_VISIBLE ActorPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: ActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
        override;

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const gazebo::common::UpdateInfo &_info);
    
    private: void NetForceUpdate();

  	private: ignition::math::Vector3d ObstacleAvoidance();

	  private: ignition::math::Vector3d ActorAvoidance();

  	private: ignition::math::Vector3d TargetForce();

    private: std::vector<ignition::math::Line3d> get_edges(gazebo::physics::EntityPtr entity);

    private: std::vector<ignition::math::Vector3d> get_corners(gazebo::physics::EntityPtr entity);

    private: ignition::math::Vector3d min_normal(ignition::math::Vector3d pos, gazebo::physics::EntityPtr entity);

	  private: void SelectRandomTarget();

    /// \internal
    public: ActorPluginPrivate *dataPtr;

  };
}
#endif
