#ifndef _RANGEBEARING_HH_
#define _RANGEBEARING_HH_


#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors
  namespace sensors
  {
    // Forward declare private data class.
    class RangeBearingPrivate;

    /// \class RaySensor RaySensor.hh sensors/sensors.hh
    /// \brief Sensor with one or more rays.
    ///
    /// This sensor cast rays into the world, tests for intersections, and
    /// reports the range to the nearest object.  It is used by ranging
    /// sensor models (e.g., sonars and scanning laser range finders).
    //class GZ_SENSORS_VISIBLE RangeBearing : public Sensor
    class RangeBearing : public Sensor
    {
      /// \brief Constructor
      public: RangeBearing();

      /// \brief Destructor
      public: virtual ~RangeBearing();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      // Documentation inherited
      public: virtual std::string Topic() const;

      // Documentation inherited
      public: virtual bool IsActive() const;

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<RangeBearingPrivate> dataPtr;
    };
  }
}

#endif
