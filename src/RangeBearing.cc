#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/Noise.hh"

#include "RangeBearingPrivate.hh"
#include "RangeBearing.hh"


using namespace gazebo;
using namespace sensors;


GZ_REGISTER_STATIC_SENSOR("rangebearing", RangeBearing)

//////////////////////////////////////////////////
RangeBearing::RangeBearing()
: Sensor(sensors::OTHER),
  dataPtr(new RangeBearingPrivate)
{
}


RangeBearing::~RangeBearing()
{
}

//////////////////////////////////////////////////
std::string RangeBearing::Topic() const
{
  std::string topicName = "~/";
  //topicName += this->ParentName() + "/" + this->Name() + "/scan";
  //boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void RangeBearing::Load(const std::string &_worldName)
{
}

/////////////////////////////////////////////////
void RangeBearing::Init()
{
  Sensor::Init();
}


//////////////////////////////////////////////////
void RangeBearing::Fini()
{
  Sensor::Fini();
}


//////////////////////////////////////////////////
bool RangeBearing::UpdateImpl(const bool /*_force*/)
{
   return true;
}

//////////////////////////////////////////////////
bool RangeBearing::IsActive() const
{
  return Sensor::IsActive();
  //||
  //  (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections());
}
