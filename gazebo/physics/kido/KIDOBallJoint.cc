/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/kido/KIDOJointPrivate.hh"
#include "gazebo/physics/kido/KIDOBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOBallJoint::KIDOBallJoint(BasePtr _parent)
  : BallJoint<KIDOJoint>(_parent)
{
  this->dataPtr->dtJoint = new kido::dynamics::BallJoint();
}

//////////////////////////////////////////////////
KIDOBallJoint::~KIDOBallJoint()
{
  // We don't need to delete dtJoint because the world will delete it
}

//////////////////////////////////////////////////
void KIDOBallJoint::Load(sdf::ElementPtr _sdf)
{
  BallJoint<KIDOJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void KIDOBallJoint::Init()
{
  BallJoint<KIDOJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 KIDOBallJoint::GetAnchor(unsigned int /*_index*/) const
{
  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return KIDOTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 KIDOBallJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
void KIDOBallJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
}

//////////////////////////////////////////////////
double KIDOBallJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzerr << "KIDOBallJoint::GetVelocity not implemented" << std::endl;
  return 0;
}

//////////////////////////////////////////////////
math::Angle KIDOBallJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzerr << "KIDOBallJoint::GetAngleImpl not implemented" << std::endl;
  return math::Angle(0);
}

//////////////////////////////////////////////////
void KIDOBallJoint::SetForceImpl(unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "KIDOBallJoint::SetForceImpl not implemented";
}

//////////////////////////////////////////////////
void KIDOBallJoint::SetAxis(unsigned int /*_index*/,
                            const math::Vector3 &/*_axis*/)
{
  gzerr << "KIDOBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
math::Angle KIDOBallJoint::GetHighStop(unsigned int /*_index*/)
{
  gzerr << "KIDOBallJoint::GetHighStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle KIDOBallJoint::GetLowStop(unsigned int /*_index*/)
{
  gzerr << "KIDOBallJoint::GetLowStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
bool KIDOBallJoint::SetHighStop(unsigned int /*_index*/,
                                const math::Angle &/*_angle*/)
{
  gzerr << "KIDOBallJoint::SetHighStop not implemented" << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool KIDOBallJoint::SetLowStop(unsigned int /*_index*/,
                               const math::Angle &/*_angle*/)
{
  gzerr << "KIDOBallJoint::SetLowStop not implemented" << std::endl;
  return false;
}
