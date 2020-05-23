

#ifndef SERVICESIM_PLUGINS_RANDOMPLACEMENT_HH_
#define SERVICESIM_PLUGINS_RANDOMPLACEMENT_HH_

#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"


namespace servicesim
{
  class RandomPlacementPrivate;

  class GAZEBO_VISIBLE RandomPlacement : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: RandomPlacement();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
        override;

    private: std::vector<ignition::math::Line3d> get_edges(gazebo::physics::EntityPtr entity);

    private: std::vector<ignition::math::Vector3d> get_corners(gazebo::physics::EntityPtr entity);

    private: ignition::math::Vector3d min_normal(ignition::math::Vector3d pos, gazebo::physics::EntityPtr entity);

    /// \internal
    public: RandomPlacementPrivate *dataPtr;


    

  };
}
#endif
