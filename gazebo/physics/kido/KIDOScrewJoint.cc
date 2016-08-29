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

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/kido/KIDOJointPrivate.hh"
#include "gazebo/physics/kido/KIDOScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOScrewJoint::KIDOScrewJoint(BasePtr _parent)
  : ScrewJoint<KIDOJoint>(_parent)
{
  this->dataPtr->dtJoint = new kido::dynamics::ScrewJoint();
}

//////////////////////////////////////////////////
KIDOScrewJoint::~KIDOScrewJoint()
{
  // We don't need to delete dtJoint because the world will delete it
}

//////////////////////////////////////////////////
void KIDOScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<KIDOJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////
void KIDOScrewJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &/*_anchor*/)
{
  gzerr << "KIDOScrewJoint::SetAnchor not implemented.\n";
}

//////////////////////////////////////////////////
void KIDOScrewJoint::Init()
{
  ScrewJoint<KIDOJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 KIDOScrewJoint::GetAnchor(unsigned int /*index*/) const
{
  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
      this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return KIDOTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 KIDOScrewJoint::GetGlobalAxis(unsigned int _index) const
{
  Eigen::Vector3d globalAxis = Eigen::Vector3d::UnitX();

  if (_index < this->GetAngleCount())
  {
    kido::dynamics::ScrewJoint *dtScrewJoint =
        reinterpret_cast<kido::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = dtScrewJoint->getAxis();
    globalAxis = T.linear() * axis;
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494
  // joint-axis-reference-frame-doesnt-match
  return KIDOTypes::ConvVec3(globalAxis);
}

//////////////////////////////////////////////////
void KIDOScrewJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  if (_index == 0)
  {
    kido::dynamics::ScrewJoint *dtScrewJoint =
        reinterpret_cast<kido::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);

    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494
    // joint-axis-reference-frame-doesnt-match
    Eigen::Vector3d kidoVec3 = KIDOTypes::ConvVec3(_axis);
    Eigen::Isometry3d kidoTransfJointLeftToParentLink
        = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
    kidoVec3 = kidoTransfJointLeftToParentLink.linear() * kidoVec3;

    dtScrewJoint->setAxis(kidoVec3);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
double KIDOScrewJoint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (_index == 0)
    result = this->dataPtr->dtJoint->getVelocity(0);
  else
    gzerr << "Invalid index[" << _index << "]\n";

  return result;
}

//////////////////////////////////////////////////
void KIDOScrewJoint::SetVelocity(unsigned int _index, double _vel)
{
  if (_index == 0)
    this->dataPtr->dtJoint->setVelocity(0, _vel);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
void KIDOScrewJoint::SetThreadPitch(unsigned int _index, double _threadPitch)
{
  if (_index >= this->GetAngleCount())
    gzerr << "Invalid index[" << _index << "]\n";

  this->SetThreadPitch(_threadPitch);
}

//////////////////////////////////////////////////
void KIDOScrewJoint::SetThreadPitch(double _threadPitch)
{
  kido::dynamics::ScrewJoint *dtScrewJoint =
      reinterpret_cast<kido::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);

  this->threadPitch = _threadPitch;
  dtScrewJoint->setPitch(KIDOTypes::InvertThreadPitch(_threadPitch));
}

//////////////////////////////////////////////////
double KIDOScrewJoint::GetThreadPitch(unsigned int _index)
{
  if (_index >= this->GetAngleCount())
    gzerr << "Invalid index[" << _index << "]\n";

  return this->GetThreadPitch();
}

//////////////////////////////////////////////////
double KIDOScrewJoint::GetThreadPitch()
{
  kido::dynamics::ScrewJoint *dtScrewJoint =
      reinterpret_cast<kido::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);

  double result = this->threadPitch;
  if (dtScrewJoint)
    result = KIDOTypes::InvertThreadPitch(dtScrewJoint->getPitch());
  else
    gzwarn << "kidoScrewJoint not created yet, returning cached threadPitch.\n";

  return result;
}

//////////////////////////////////////////////////
double KIDOScrewJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_key  == "thread_pitch")
    return this->GetThreadPitch();
  else
    return KIDOJoint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
math::Angle KIDOScrewJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (_index == 0)
  {
    // angular position
    const double radianAngle = this->dataPtr->dtJoint->getPosition(0);
    result.SetFromRadian(radianAngle);
  }
  else if (_index == 1)
  {
    // linear position
    const double radianAngle = this->dataPtr->dtJoint->getPosition(0);
    result.SetFromRadian(-radianAngle /
                         const_cast<KIDOScrewJoint*>(this)->GetThreadPitch());
    // TODO: The ScrewJoint::GetThreadPitch() function is not const. As an
    // workaround, we use const_cast here until #1686 is resolved.
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
void KIDOScrewJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (_index == 0)
    this->dataPtr->dtJoint->setForce(0, _effort);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
math::Angle KIDOScrewJoint::GetHighStop(unsigned int _index)
{
  switch (_index)
  {
  case 0:
    return this->dataPtr->dtJoint->getPositionUpperLimit(0);
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle KIDOScrewJoint::GetLowStop(unsigned int _index)
{
  switch (_index)
  {
  case 0:
    return this->dataPtr->dtJoint->getPositionLowerLimit(0);
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return math::Angle();
}
