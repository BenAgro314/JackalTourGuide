

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
    /// \internal
    public: RandomPlacementPrivate *dataPtr;

  };
}
#endif
