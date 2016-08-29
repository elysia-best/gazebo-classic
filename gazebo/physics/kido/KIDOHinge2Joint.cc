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
#include "gazebo/physics/kido/KIDOHinge2Joint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOHinge2Joint::KIDOHinge2Joint(BasePtr _parent)
  : Hinge2Joint<KIDOJoint>(_parent)
{
  this->dataPtr->dtJoint = new kido::dynamics::UniversalJoint();
}

//////////////////////////////////////////////////
KIDOHinge2Joint::~KIDOHinge2Joint()
{
  // We don't need to delete dtJoint because the world will delete it
}

//////////////////////////////////////////////////
void KIDOHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<KIDOJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void KIDOHinge2Joint::Init()
{
  Hinge2Joint<KIDOJoint>::Init();
}

//////////////////////////////////////////////////
math::Vector3 KIDOHinge2Joint::GetAnchor(unsigned int /*_index*/) const
{
  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return KIDOTypes::ConvVec3(worldOrigin);
}

//////////////////////////////////////////////////
void KIDOHinge2Joint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  Eigen::Vector3d kidoAxis = KIDOTypes::ConvVec3(_axis);

  if (_index == 0)
  {
    kido::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<kido::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);

    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference
    Eigen::Isometry3d kidoTransfJointLeftToParentLink
        = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
    kidoAxis = kidoTransfJointLeftToParentLink.linear() * kidoAxis;

    dtUniveralJoint->setAxis1(kidoAxis);
  }
  else if (_index == 1)
  {
    kido::dynamics::UniversalJoint *dtUniveralJoint =
        reinterpret_cast<kido::dynamics::UniversalJoint *>(
          this->dataPtr->dtJoint);

    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference
    Eigen::Isometry3d kidoTransfJointLeftToParentLink
        = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
    kidoAxis = kidoTransfJointLeftToParentLink.linear() * kidoAxis;

    dtUniveralJoint->setAxis2(kidoAxis);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
math::Vector3 KIDOHinge2Joint::GetGlobalAxis(unsigned int _index) const
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

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/
  // joint-axis-reference-frame-doesnt-match
  return KIDOTypes::ConvVec3(globalAxis);
}

//////////////////////////////////////////////////
math::Angle KIDOHinge2Joint::GetAngleImpl(unsigned int _index) const
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
double KIDOHinge2Joint::GetVelocity(unsigned int _index) const
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
void KIDOHinge2Joint::SetVelocity(unsigned int _index, double _vel)
{
  if (_index == 0)
    this->dataPtr->dtJoint->setVelocity(0, _vel);
  else if (_index == 1)
    this->dataPtr->dtJoint->setVelocity(1, _vel);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
void KIDOHinge2Joint::SetForceImpl(unsigned int _index, double _effort)
{
  if (_index == 0)
    this->dataPtr->dtJoint->setForce(0, _effort);
  else if (_index == 1)
    this->dataPtr->dtJoint->setForce(1, _effort);
  else
    gzerr << "Invalid index[" << _index << "]\n";
}
