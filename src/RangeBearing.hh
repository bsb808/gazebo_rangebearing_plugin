#ifndef _RANGEBEARING_HH_
#define _RANGEBEARING_HH_


#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

#include <gazebo/common/common.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>


namespace gazebo
{
  /// \class RaySensor RaySensor.hh sensors/sensors.hh
  /// \brief Sensor with one or more rays.
  ///
  /// This sensor cast rays into the world, tests for intersections, and
  /// reports the range to the nearest object.  It is used by ranging
  /// sensor models (e.g., sonars and scanning laser range finders).
  //class GZ_SENSORS_VISIBLE RangeBearing : public Sensor
  class RangeBearing : public ModelPlugin
  {
    /// \brief Constructor
    public: RangeBearing();
    
    /// \brief Destructor
    public: virtual ~RangeBearing() = default;

    // Documentation inherited
    protected: virtual void Load(physics::ModelPtr _model,
				 sdf::ElementPtr _sdf);

    // Documentation inherited
    protected: virtual void Update();

  };
}

#endif
