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
#include "gazebo/physics/kido/KIDOUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOUniversalJoint::KIDOUniversalJoint(BasePtr _parent)
  : UniversalJoint<KIDOJoint>(_parent)
{
  this->dataPtr->dtJoint = new kido::dynamics::UniversalJoint();
}

//////////////////////////////////////////////////
KIDOUniversalJoint::~KIDOUniversalJoint()
{
  // We don't need to delete dtJoint because the world will delete it
}

//////////////////////////////////////////////////
void KIDOUniversalJoint::Load(sdf::ElementPtr _sdf)
{
  UniversalJoint<KIDOJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void KIDOUniversalJoint::Init()
{
  UniversalJoint<KIDOJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 KIDOUniversalJoint::GetAnchor(unsigned int /*index*/) const
{
  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return KIDOTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
math::Vector3 KIDOUniversalJoint::GetGlobalAxis(unsigned int _index) const
{
  Eigen::Vector3d globalAxis = Eigen::Vector3d::UnitX();

  if (_index == 0)
  {
    kido::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<kido::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getLocalTransform().inverse() *
        this->dataPtr->dtJoint->getTransformFromParentBodyNode();
    Eigen::Vector3d axis = dtUniveralJoint->getAxis1();

    globalAxis = T.linear() * axis;
  }
  else if (_index == 1)
  {
    kido::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<kido::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = dtUniveralJoint->getAxis2();

    globalAxis = T.linear() * axis;
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return KIDOTypes::ConvVec3(globalAxis);
}

//////////////////////////////////////////////////
void KIDOUniversalJoint::SetAxis(unsigned int _index,
    const math::Vector3 &_axis)
{
  Eigen::Vector3d dtAxis = KIDOTypes::ConvVec3(
      this->GetAxisFrameOffset(_index).RotateVector(_axis));
  Eigen::Isometry3d dtTransfJointLeftToParentLink
      = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
  dtAxis = dtTransfJointLeftToParentLink.linear() * dtAxis;

  if (_index == 0)
  {
    kido::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<kido::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);
    dtUniveralJoint->setAxis1(dtAxis);
  }
  else if (_index == 1)
  {
    kido::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<kido::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);
    dtUniveralJoint->setAxis2(dtAxis);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
math::Angle KIDOUniversalJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (_index == 0)
  {
    double radianAngle = this->dataPtr->dtJoint->getPosition(0);
    result.SetFromRadian(radianAngle);
  }
  else if (_index == 1)
  {
    double radianAngle = this->dataPtr->dtJoint->getPosition(1);
    result.SetFromRadian(radianAngle);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
double KIDOUniversalJoint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (_index == 0)
    result = this->dataPtr->dtJoint->getVelocity(0);
  else if (_index == 1)
    result = this->dataPtr->dtJoint->getVelocity(1);
  else
    gzerr << "Invalid index[" << _index << "]\n";

  return result;
}

//////////////////////////////////////////////////
void KIDOUniversalJoint::SetVelocity(unsigned int _index, double _vel)
{
  if (_index < this->GetAngleCount())
  {
    this->dataPtr->dtJoint->setVelocity(_index, _vel);
    this->dataPtr->dtJoint->getSkeleton()->computeForwardKinematics(
          false, true, false);
  }
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
void KIDOUniversalJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (_index == 0)
    this->dataPtr->dtJoint->setForce(0, _effort);
  else if (_index == 1)
    this->dataPtr->dtJoint->setForce(1, _effort);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}
