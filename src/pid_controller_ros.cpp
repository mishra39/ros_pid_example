#include "pid_example/pid_controller_ros.h"

PidControllerRos::PidControllerRos()
    : reference_(0.0),
      measurement_(0.0)
{

}

PidControllerRos::PidControllerRos(double kp, double ki, double kd)
    : PidControllerBase::PidControllerBase(kp,ki,kd),
      reference_(0.0),
      measurement_(0.0)
{

}

PidControllerRos::~PidControllerRos()
{

}

double PidControllerRos::getReference(void)
{
  return reference_;
}

double PidControllerRos::getMeasurement(void)
{
  return measurement_;
}

void PidControllerRos::referenceCallback(const std_msgs::Float32::ConstPtr &msg)
{
    reference_ = msg->data;
}

void PidControllerRos::measurementCallback(const std_msgs::Float32::ConstPtr &msg)
{
    measurement_ = msg->data;
}