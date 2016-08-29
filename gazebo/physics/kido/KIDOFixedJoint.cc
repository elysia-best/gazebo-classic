/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/kido/KIDOJointPrivate.hh"
#include "gazebo/physics/kido/KIDOFixedJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOFixedJoint::KIDOFixedJoint(BasePtr _parent)
  : FixedJoint<KIDOJoint>(_parent)
{
  this->dataPtr->dtJoint = new kido::dynamics::WeldJoint();
}

//////////////////////////////////////////////////
KIDOFixedJoint::~KIDOFixedJoint()
{
  // We don't need to delete dtJoint because the world will delete it
}

//////////////////////////////////////////////////
void KIDOFixedJoint::Load(sdf::ElementPtr _sdf)
{
  FixedJoint<KIDOJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void KIDOFixedJoint::Init()
{
  FixedJoint<KIDOJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 KIDOFixedJoint::GetAnchor(unsigned int /*index*/) const
{
  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return KIDOTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 KIDOFixedJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzwarn << "KIDOFixedJoint: called method "
         << "GetGlobalAxis that is not valid for joints of type fixed.\n";

  return math::Vector3();
}

//////////////////////////////////////////////////
void KIDOFixedJoint::SetAxis(unsigned int /*_index*/,
                             const math::Vector3& /*_axis*/)
{
  gzwarn << "KIDOFixedJoint: called method "
         << "SetAxis that is not valid for joints of type fixed.\n";
  return;
}

//////////////////////////////////////////////////
math::Angle KIDOFixedJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzwarn << "KIDOFixedJoint: called method "
         << "GetAngleImpl that is not valid for joints of type fixed.\n";
  return math::Angle();
}

//////////////////////////////////////////////////
void KIDOFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_vel*/)
{
  gzwarn << "KIDOFixedJoint: called method "
         << "SetVelocity that is not valid for joints of type fixed.\n";
  return;
}

//////////////////////////////////////////////////
double KIDOFixedJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzwarn << "KIDOFixedJoint: called method "
         << "GetVelocity that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void KIDOFixedJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  gzwarn << "KIDOFixedJoint: called method "
         << "SetForceImpl that is not valid for joints of type fixed.\n";
}
