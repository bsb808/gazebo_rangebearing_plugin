#include <ros/ros.h>
#include <gazebo/math/Pose.hh>
#include <tf2/LinearMath/Transform.h>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/Noise.hh"


//#include "RangeBearingPrivate.hh"
#include "RangeBearing.hh"


using namespace gazebo;
//using namespace sensors;


//////////////////////////////////////////////////
RangeBearing::RangeBearing()
{
  ROS_WARN("Constructed");

}



//////////////////////////////////////////////////
void RangeBearing::Load(physics::ModelPtr _model,
			sdf::ElementPtr _sdf)
{
  gzwarn << "Load \n";
  ROS_WARN("Loading range bearing plugin");

}

//////////////////////////////////////////////////
void RangeBearing::Update()
{
  return;
}

