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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/kido/KIDOLink.hh"
#include "gazebo/physics/kido/KIDOModel.hh"
#include "gazebo/physics/kido/KIDOJoint.hh"

#include "gazebo/physics/kido/KIDOJointPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOJoint::KIDOJoint(BasePtr _parent)
  : Joint(_parent),
    dataPtr(new KIDOJointPrivate(boost::dynamic_pointer_cast<KIDOPhysics>(
      this->GetWorld()->GetPhysicsEngine())))
{
}

//////////////////////////////////////////////////
KIDOJoint::~KIDOJoint()
{
  this->Detach();

  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void KIDOJoint::Load(sdf::ElementPtr _sdf)
{
  if (!this->dataPtr->dtJoint)
  {
    gzerr << "dtJoint should be created in each subclass constructor" <<
        std::endl;
  }

  // In Joint::Load(sdf::ElementPtr), this joint stored the information of the
  // parent link and child link.
  Joint::Load(_sdf);
}

//////////////////////////////////////////////////
void KIDOJoint::Init()
{
  Joint::Init();

  // Name
  std::string jointName = this->GetName();
  this->dataPtr->dtJoint->setName(jointName.c_str());

  // Parent and child link information
  KIDOLinkPtr kidoParentLink =
    boost::static_pointer_cast<KIDOLink>(this->parentLink);
  KIDOLinkPtr kidoChildLink =
    boost::static_pointer_cast<KIDOLink>(this->childLink);

  Eigen::Isometry3d dtTransformParentLinkToJoint =
      Eigen::Isometry3d::Identity();
  Eigen::Isometry3d dtTransformChildLinkToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d dtTransformParentBodyNode = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d dtTransformChildBodyNode = Eigen::Isometry3d::Identity();

  // if (theChildLink != NULL)
  GZ_ASSERT(kidoChildLink.get() != NULL, "kidoChildLink pointer is NULL");
  {
    dtTransformChildBodyNode =
        KIDOTypes::ConvPose(kidoChildLink->GetWorldPose());
    this->dataPtr->dtChildBodyNode = kidoChildLink->GetKIDOBodyNode();
    this->dataPtr->dtChildBodyNode->setParentJoint(this->dataPtr->dtJoint);
  }
  dtTransformChildLinkToJoint = KIDOTypes::ConvPose(this->anchorPose);

  if (kidoParentLink.get() != NULL)
  {
    dtTransformParentBodyNode =
        KIDOTypes::ConvPose(kidoParentLink->GetWorldPose());
    kido::dynamics::BodyNode *dtParentBodyNode =
      kidoParentLink->GetKIDOBodyNode();
    dtParentBodyNode->addChildBodyNode(this->dataPtr->dtChildBodyNode);
  }

  dtTransformParentLinkToJoint = dtTransformParentBodyNode.inverse() *
                                 dtTransformChildBodyNode *
                                 dtTransformChildLinkToJoint;

  // We assume that the joint angles are all zero.
  this->dataPtr->dtJoint->setTransformFromParentBodyNode(
        dtTransformParentLinkToJoint);
  this->dataPtr->dtJoint->setTransformFromChildBodyNode(
        dtTransformChildLinkToJoint);
}

//////////////////////////////////////////////////
void KIDOJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr KIDOJoint::GetJointLink(unsigned int _index) const
{
  LinkPtr result;

  if (_index == 0)
  {
    KIDOLinkPtr kidoLink1
        = boost::static_pointer_cast<KIDOLink>(this->parentLink);

    if (kidoLink1 != NULL)
      return this->parentLink;
  }

  if (_index == 1)
  {
    KIDOLinkPtr kidoLink2
        = boost::static_pointer_cast<KIDOLink>(this->childLink);

    if (kidoLink2 != NULL)
      return this->childLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool KIDOJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  if (_one.get() == NULL && _two.get() == NULL)
    return false;

  if ((this->childLink.get() == _one.get() &&
       this->parentLink.get() == _two.get()) ||
      (this->childLink.get() == _two.get() &&
       this->parentLink.get() == _one.get()))
    return true;

  return false;
}

//////////////////////////////////////////////////
void KIDOJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
  Joint::Attach(_parent, _child);

  if (this->AreConnected(_parent, _child))
    return;

  gzerr << "KIDO does not support joint attaching.\n";
}

//////////////////////////////////////////////////
void KIDOJoint::Detach()
{
  if (!this->AreConnected(this->parentLink, this->childLink))
    return;

  this->childLink.reset();
  this->parentLink.reset();

  gzerr << "KIDO does not support joint dettaching.\n";

  Joint::Detach();
}

//////////////////////////////////////////////////
void KIDOJoint::SetAnchor(unsigned int /*_index*/,
    const gazebo::math::Vector3 &/*_anchor*/)
{
  // nothing to do here for KIDO.
}

//////////////////////////////////////////////////
void KIDOJoint::SetDamping(unsigned int _index, double _damping)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
                              _damping);
  }
  else
  {
    gzerr << "Index[" << _index << "] is out of bounds (GetAngleCount() = "
          << this->GetAngleCount() << ").\n";
    return;
  }
}

//////////////////////////////////////////////////
void KIDOJoint::SetStiffness(unsigned int _index, const double _stiffness)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "KIDOJoint::SetStiffness: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void KIDOJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->GetAngleCount())
  {
    this->stiffnessCoefficient[_index] = _stiffness;
    this->dissipationCoefficient[_index] = _damping;
    this->springReferencePosition[_index] = _reference;

    /// \TODO: this check might not be needed?  attaching an object to a static
    /// body should not affect damping application.
    bool parentStatic =
        this->GetParent() ? this->GetParent()->IsStatic() : false;
    bool childStatic =
        this->GetChild() ? this->GetChild()->IsStatic() : false;

    if (!this->applyDamping)
    {
      if (!parentStatic && !childStatic)
      {
        this->dataPtr->dtJoint->setSpringStiffness(
              static_cast<int>(_index), _stiffness);
        this->dataPtr->dtJoint->setRestPosition(
              static_cast<int>(_index), _reference);
        this->dataPtr->dtJoint->setDampingCoefficient(
              static_cast<int>(_index), _damping);
        this->applyDamping = physics::Joint::ConnectJointUpdate(
          boost::bind(&KIDOJoint::ApplyDamping, this));
      }
      else
      {
        gzwarn << "Spring Damper for Joint[" << this->GetName()
               << "] is not initialized because either parent[" << parentStatic
               << "] or child[" << childStatic << "] is static.\n";
      }
    }
  }
  else
  {
    gzerr << "SetStiffnessDamping _index " << _index << " is too large.\n";
  }
}

//////////////////////////////////////////////////
bool KIDOJoint::SetHighStop(unsigned int _index, const math::Angle &_angle)
{
  switch (_index)
  {
    case 0:
    case 1:
    case 2:
      this->dataPtr->dtJoint->setPositionUpperLimit(_index, _angle.Radian());
      return true;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };
}

//////////////////////////////////////////////////
bool KIDOJoint::SetLowStop(unsigned int _index, const math::Angle &_angle)
{
  switch (_index)
  {
  case 0:
  case 1:
  case 2:
    this->dataPtr->dtJoint->setPositionLowerLimit(_index, _angle.Radian());
    return true;
  default:
    gzerr << "Invalid index[" << _index << "]\n";
    return false;
  };
}

//////////////////////////////////////////////////
math::Angle KIDOJoint::GetHighStop(unsigned int _index)
{
  switch (_index)
  {
  case 0:
  case 1:
  case 2:
    return this->dataPtr->dtJoint->getPositionUpperLimit(_index);
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return 0;
}

//////////////////////////////////////////////////
math::Angle KIDOJoint::GetLowStop(unsigned int _index)
{
  switch (_index)
  {
  case 0:
  case 1:
  case 2:
    return this->dataPtr->dtJoint->getPositionLowerLimit(_index);
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return 0;
}

//////////////////////////////////////////////////
math::Vector3 KIDOJoint::GetLinkForce(unsigned int _index) const
{
  math::Vector3 result;

  if (!this->dataPtr->dtJoint)
  {
    gzerr << "KIDO joint is invalid\n";
    return result;
  }

  //---------------------------------------------
  // Parent and child link information
  //---------------------------------------------
  KIDOLinkPtr theChildLink =
    boost::static_pointer_cast<KIDOLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = this->dataPtr->dtJoint->getLocalTransform();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink != NULL)
  {
    kido::dynamics::BodyNode *kidoChildBody = theChildLink->GetKIDOBodyNode();
    GZ_ASSERT(kidoChildBody, "kidoChildBody pointer is NULL");
    F2 = -kido::math::dAdT(
          this->dataPtr->dtJoint->getTransformFromChildBodyNode(),
          kidoChildBody->getBodyForce());
  }

  // JointWrench.body2Force contains
  // the force applied by the child Link on the Joint specified
  // in the child Link frame.
  F1 = -kido::math::dAdInvR(T12, F2);

  if (_index == 0)
    result.Set(F1(3), F1(4), F1(5));
  else
    result.Set(F2(3), F2(4), F2(5));

  return result;
}

//////////////////////////////////////////////////
math::Vector3 KIDOJoint::GetLinkTorque(unsigned int _index) const
{
  math::Vector3 result;

  if (!this->dataPtr->dtJoint)
  {
    gzerr << "KIDO joint is invalid\n";
    return result;
  }

  // Parent and child link information
  KIDOLinkPtr theChildLink =
    boost::static_pointer_cast<KIDOLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = this->dataPtr->dtJoint->getLocalTransform();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink != NULL)
  {
    kido::dynamics::BodyNode *kidoChildBody = theChildLink->GetKIDOBodyNode();
    GZ_ASSERT(kidoChildBody, "kidoChildBody pointer is NULL");
    F2 = -kido::math::dAdT(
      this->dataPtr->dtJoint->getTransformFromChildBodyNode(),
      kidoChildBody->getBodyForce());
  }

  // JointWrench.body2Force contains
  // the force applied by the child Link on the Joint specified
  // in the child Link frame.
  F1 = -kido::math::dAdInvR(T12, F2);

  if (_index == 0)
    result.Set(F1(0), F1(1), F1(2));
  else
    result.Set(F2(0), F2(1), F2(2));

  return result;
}

//////////////////////////////////////////////////
bool KIDOJoint::SetParam(const std::string &_key, unsigned int _index,
                         const boost::any &_value)
{
  // try because boost::any_cast can throw
  try
  {
    if (_key == "hi_stop")
    {
      this->SetHighStop(_index, boost::any_cast<double>(_value));
    }
    else if (_key == "lo_stop")
    {
      this->SetLowStop(_index, boost::any_cast<double>(_value));
    }
    else if (_key == "friction")
    {
      this->dataPtr->dtJoint->setCoulombFriction(_index,
                                        boost::any_cast<double>(_value));
    }
    else
    {
      gzerr << "Unable to handle joint attribute[" << _key << "]\n";
      return false;
    }
  }
  catch(const boost::bad_any_cast &e)
  {
    gzerr << "SetParam(" << _key << ")"
          << " boost any_cast error:" << e.what()
          << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
double KIDOJoint::GetParam(const std::string &_key, unsigned int _index)
{
  try
  {
    if (_key == "friction")
    {
      return this->dataPtr->dtJoint->getCoulombFriction(_index);
    }
  }
  catch(const common::Exception &e)
  {
    gzerr << "GetParam(" << _key << ") error:"
          << e.GetErrorStr()
          << std::endl;
    return 0;
  }
  return Joint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
void KIDOJoint::CacheForceTorque()
{
  // Does nothing for now, will add when recovering pull request #1721
}

//////////////////////////////////////////////////
JointWrench KIDOJoint::GetForceTorque(unsigned int /*_index*/)
{
  JointWrench jointWrench;

  //---------------------------------------------
  // Parent and child link information
  //---------------------------------------------
  KIDOLinkPtr theChildLink =
    boost::static_pointer_cast<KIDOLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = this->dataPtr->dtJoint->getLocalTransform();

  // JointWrench.body2Force (F2) contains
  // the force applied by the child Link on the parent link specified
  // in the child Link orientation frame and with respect to the joint origin
  if (theChildLink != NULL)
  {
    kido::dynamics::BodyNode *kidoChildBody = theChildLink->GetKIDOBodyNode();
    GZ_ASSERT(kidoChildBody, "kidoChildBody pointer is NULL");
    Eigen::Isometry3d TJ2 = Eigen::Isometry3d::Identity();
    TJ2.translation() =
        this->dataPtr->dtJoint->getTransformFromChildBodyNode().translation();
    F2 = -kido::math::dAdT(TJ2,
                           kidoChildBody->getBodyForce());
  }

  // JointWrench.body1Force (F1) contains the
  // force applied by the parent Link on the child Link specified in
  // the parent Link orientation frame and with respect to the joint origin
  F1 = -kido::math::dAdInvR(T12, F2);

  // kind of backwards here, body1 (parent) corresponds go f2, t2
  // and body2 (child) corresponds go f1, t1
  jointWrench.body1Force.Set(F1(3), F1(4), F1(5));
  jointWrench.body1Torque.Set(F1(0), F1(1), F1(2));
  jointWrench.body2Force.Set(F2(3), F2(4), F2(5));
  jointWrench.body2Torque.Set(F2(0), F2(1), F2(2));

  return jointWrench;
}

/////////////////////////////////////////////////
void KIDOJoint::SetForce(unsigned int _index, double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
}

/////////////////////////////////////////////////
double KIDOJoint::GetForce(unsigned int _index)
{
  if (_index < this->GetAngleCount())
  {
    return this->dataPtr->forceApplied[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get force\n";
    return 0;
  }
}

/////////////////////////////////////////////////
unsigned int KIDOJoint::GetAngleCount() const
{
  unsigned int angleCount = 0;

  angleCount = this->dataPtr->dtJoint->getNumDofs();

  return angleCount;
}

/////////////////////////////////////////////////
void KIDOJoint::ApplyDamping()
{
  // rename ApplyDamping to ApplyStiffnessDamping (below) in gazebo 4.0.
  // public: virtual void ApplyStiffnessDamping();

  // KIDO applies stiffness and damping force implicitly itself by setting
  // the stiffness coefficient and the damping coefficient using
  // kido::dynamics::Joint::setSpringStiffness(index, stiffnessCoeff) and
  // kido::dynamics::Joint::setDampingCoefficient(index, dampingCoeff).
  // Therefore, we do nothing here.
}

/////////////////////////////////////////////////
KIDOModelPtr KIDOJoint::GetKIDOModel() const
{
  return boost::dynamic_pointer_cast<KIDOModel>(this->model);
}

/////////////////////////////////////////////////
kido::dynamics::Joint *KIDOJoint::GetKIDOJoint()
{
  return this->dataPtr->dtJoint;
}

/////////////////////////////////////////////////
void KIDOJoint::SaveForce(unsigned int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->GetAngleCount())
  {
    if (this->dataPtr->forceAppliedTime < this->GetWorld()->GetSimTime())
    {
      // reset forces if time step is new
      this->dataPtr->forceAppliedTime = this->GetWorld()->GetSimTime();
      this->dataPtr->forceApplied[0] = this->dataPtr->forceApplied[1] = 0.0;
    }

    this->dataPtr->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->GetName()
          << "] index [" << _index
          << "] out of range.\n";
}
