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
  ROS_INFO("Loading range bearing plugin");

  // Parse SDF
  if (!_sdf->HasElement("robotNamespace"))
    this->rosNamespace.clear();
  else
    this->rosNamespace = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (!_sdf->HasElement("bodyName"))
  {
    this->link = _model->GetLink();
    this->linkName = this->link->GetName();
  }
  else {
    this->linkName = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    this->link = _model->GetLink(this->linkName);
  }

  if (!this->link)
  {
    ROS_FATAL("RangeBearing plugin error: bodyName: %s does not exist\n", this->linkName.c_str());
    return;
  }

  // Defaults
  this->rbTopic = "rangebearing";
  this->beaconPoint = math::Vector3(0,0,0);
  
  // More parsing of SDF
  if (_sdf->HasElement("topicName"))
    this->rbTopic = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("beaconPoint"))
  {
    this->beaconPoint =
      _sdf->GetElement("beaconPoint")->Get<math::Vector3>();
  }

  // ROS Setup
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->nodeHandle = new ros::NodeHandle(this->rosNamespace);
  this->rbPublisher = this->nodeHandle->advertise<std_msgs::Float32MultiArray>(this->rbTopic, 10);
  this->rbMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  this->rbMsg.layout.dim[0].size = 3;
  this->rbMsg.layout.dim[0].stride = 1;
  this->rbMsg.layout.dim[0].label="x";
						   
  
  this->world = _model->GetWorld();
  
  this->updateTimer.setUpdateRate(4.0);
  this->updateTimer.Load(this->world, _sdf);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = this->updateTimer.Connect(std::bind(&RangeBearing::Update, this));
  
}

//////////////////////////////////////////////////
void RangeBearing::Update()
{
  ROS_WARN("Update");

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  const math::Pose pose = this->link->GetWorldPose();
  const math::Vector3 euler = pose.rot.GetAsEuler();
  
  // Do some geometry
  double range =  std::sqrt(std::pow(pose.pos.x-beaconPoint.x,2)+
							std::pow(pose.pos.y-beaconPoint.y,2)+
							std::pow(pose.pos.z-beaconPoint.z,2));
  double bearing = std::atan2(beaconPoint.y-pose.pos.y,
							  beaconPoint.x-pose.pos.x);
  bearing -= euler.z;
  double pi = 3.15159;
  bearing = clamp(bearing,pi,-pi);
  	
  double elevation = std::atan2(beaconPoint.z-pose.pos.z,
								std::sqrt(std::pow(beaconPoint.x-pose.pos.x,2)+
										  std::pow(beaconPoint.y-pose.pos.y,2)));
  elevation -= euler.y;
  elevation = clamp(elevation,pi,-pi);
  
  // Publish
  this->rbMsg.data.clear();
  this->rbMsg.data.push_back(range);
  this->rbMsg.data.push_back(bearing);
  this->rbMsg.data.push_back(elevation);
  this->rbPublisher.publish(this->rbMsg);
  return;
}


GZ_REGISTER_MODEL_PLUGIN(RangeBearing);

