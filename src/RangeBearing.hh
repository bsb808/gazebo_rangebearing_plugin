#ifndef _RANGEBEARING_HH_
#define _RANGEBEARING_HH_

#include <cmath>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>

#include <gazebo/common/common.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Pose.hh>

#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include "update_timer.h"

#include <algorithm>    // std::min

inline double clamp(double val, double top, double bottom)
{
  return std::max(bottom, std::min(val, top));
}


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

    private:
      /// \brief The parent World
      physics::WorldPtr world;

      /// \brief Pointer to the model
      physics::ModelPtr model;

      /// \brief Pointer to link
      physics::LinkPtr link;

      /// \brief Pointer to the update event connection.
      event::ConnectionPtr updateConnection;

      /// \brief Update timer
      UpdateTimer updateTimer;

      ros::NodeHandle* nodeHandle;
    
      ros::Publisher rbPublisher;
      std::string rbTopic;
      std_msgs::Float32MultiArray rbMsg;
      std::string rosNamespace;
      std::string linkName;
      math::Vector3 beaconPoint;
  };
}

#endif
