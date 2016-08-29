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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldPrivate.hh"

#include "gazebo/physics/kido/kido_inc.h"
#include "gazebo/physics/kido/KIDOCollision.hh"
#include "gazebo/physics/kido/KIDOPhysics.hh"
#include "gazebo/physics/kido/KIDOModel.hh"
#include "gazebo/physics/kido/KIDOLink.hh"
#include "gazebo/physics/kido/KIDOJoint.hh"
#include "gazebo/physics/kido/KIDOSurfaceParams.hh"

#include "gazebo/physics/kido/KIDOLinkPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOLink::KIDOLink(EntityPtr _parent)
  : Link(_parent), dataPtr(new KIDOLinkPrivate())
{
}

//////////////////////////////////////////////////
KIDOLink::~KIDOLink()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void KIDOLink::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->kidoPhysics = boost::dynamic_pointer_cast<KIDOPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->dataPtr->kidoPhysics == NULL)
    gzthrow("Not using the kido physics engine");

  // Check if soft_contact element is contained in this link. If so,
  // SoftBodyNode will be created. Otherwise, BodyNode will be created.
  sdf::ElementPtr kidoElem;
  sdf::ElementPtr softCollElem;
  sdf::ElementPtr softGeomElem;

  if (_sdf->HasElement("collision"))
  {
    sdf::ElementPtr collElem = _sdf->GetElement("collision");
    while (collElem)
    {
      // Geometry
      sdf::ElementPtr geomElem = collElem->GetElement("geometry");

      // Surface
      if (collElem->HasElement("surface"))
      {
        sdf::ElementPtr surfaceElem = collElem->GetElement("surface");

        // Soft contact
        if (surfaceElem->HasElement("soft_contact"))
        {
          sdf::ElementPtr softContactElem
              = surfaceElem->GetElement("soft_contact");

          if (softContactElem->HasElement("kido"))
          {
            if (kidoElem != NULL)
            {
              gzerr << "KIDO supports only one deformable body in a link.\n";
              break;
            }

            kidoElem = softContactElem->GetElement("kido");
            softCollElem = collElem;
            softGeomElem = geomElem;
          }
        }
      }

      collElem = collElem->GetNextElement("collision");
    }
  }

  if (kidoElem != NULL)
  {
    // Create KIDO SoftBodyNode
    kido::dynamics::SoftBodyNode *dtSoftBodyNode
        = new kido::dynamics::SoftBodyNode();

    // Mass
    double fleshMassFraction = kidoElem->Get<double>("flesh_mass_fraction");

    // bone_attachment (Kv)
    if (kidoElem->HasElement("bone_attachment"))
    {
      double kv = kidoElem->Get<double>("bone_attachment");
      dtSoftBodyNode->setVertexSpringStiffness(kv);
    }

    // stiffness (Ke)
    if (kidoElem->HasElement("stiffness"))
    {
      double ke = kidoElem->Get<double>("stiffness");
      dtSoftBodyNode->setEdgeSpringStiffness(ke);
    }

    // damping
    if (kidoElem->HasElement("damping"))
    {
      double damping = kidoElem->Get<double>("damping");
      dtSoftBodyNode->setDampingCoefficient(damping);
    }

    // pose
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    gzdbg << "pose" << T.matrix() << std::endl;
    if (softCollElem->HasElement("pose"))
    {
      T = KIDOTypes::ConvPose(softCollElem->Get<math::Pose>("pose"));
    }

    // geometry
    if (softGeomElem->HasElement("box"))
    {
      sdf::ElementPtr boxEle = softGeomElem->GetElement("box");
      Eigen::Vector3d size
          = KIDOTypes::ConvVec3(boxEle->Get<math::Vector3>("size"));
      kido::dynamics::SoftBodyNodeHelper::setBox(
            dtSoftBodyNode, size, T, fleshMassFraction);
      dtSoftBodyNode->addCollisionShape(
            new kido::dynamics::SoftMeshShape(dtSoftBodyNode));
    }
//    else if (geomElem->HasElement("ellipsoid"))
//    {
//      sdf::ElementPtr ellipsoidEle = geomElem->GetElement("ellipsoid");
//      Eigen::Vector3d size
//          = KIDOTypes::ConvVec3(ellipsoidEle->Get<math::Vector3>("size"));
//      double nSlices = ellipsoidEle->Get<double>("num_slices");
//      double nStacks = ellipsoidEle->Get<double>("num_stacks");
//      kido::dynamics::SoftBodyNodeHelper::setEllipsoid(
//            dtSoftBodyNode, size, nSlices, nStacks, fleshMassFraction);
//      dtSoftBodyNode->addCollisionShape(
//            new kido::dynamics::SoftMeshShape(dtSoftBodyNode));
//    }
    else
    {
      gzerr << "Unknown soft shape" << std::endl;
    }

    this->dataPtr->dtBodyNode = dtSoftBodyNode;
  }
  else
  {
    // Create KIDO BodyNode
    this->dataPtr->dtBodyNode = new kido::dynamics::BodyNode();
  }

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void KIDOLink::Init()
{
  Link::Init();

  // Name
  std::string bodyName = this->GetName();
  this->dataPtr->dtBodyNode->setName(bodyName);

  // Mass
  double mass = this->inertial->GetMass();
  this->dataPtr->dtBodyNode->setMass(mass);

  // Inertia
  double Ixx = this->inertial->GetIXX();
  double Iyy = this->inertial->GetIYY();
  double Izz = this->inertial->GetIZZ();
  double Ixy = this->inertial->GetIXY();
  double Ixz = this->inertial->GetIXZ();
  double Iyz = this->inertial->GetIYZ();
  this->dataPtr->dtBodyNode->setMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

  // Visual
  this->visuals;

  // COG offset
  math::Vector3 cog = this->inertial->GetCoG();
  this->dataPtr->dtBodyNode->setLocalCOM(KIDOTypes::ConvVec3(cog));

  // Gravity mode
  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  // Friction coefficient

  /// \todo FIXME: Friction Parameters
  /// Gazebo allows different friction parameters per collision objects,
  /// while KIDO stores the friction parameter per link (BodyNode in KIDO). For
  /// now, the average friction parameter of all the child collision objects is
  /// stored in this->dataPtr->dtBodyNode.
  /// Final friction coefficient is applied in KIDO's constraint solver by
  /// taking the lower of the 2 colliding rigidLink's.
  /// See also:
  /// - https://github.com/kidosim/kido/issues/141
  /// - https://github.com/kidosim/kido/issues/266

  double hackAvgMu1 = 0;
  double hackAvgMu2 = 0;
  int numCollisions = 0;

  for (auto const &child : this->children)
  {
    if (child->HasType(Base::COLLISION))
    {
      CollisionPtr collision =
          boost::static_pointer_cast<Collision>(child);

      SurfaceParamsPtr surface = collision->GetSurface();
      GZ_ASSERT(surface, "Surface pointer for is invalid");
      FrictionPyramidPtr friction = surface->FrictionPyramid();
      GZ_ASSERT(friction, "Friction pointer for is invalid");

      numCollisions++;
      hackAvgMu1 += friction->MuPrimary();
      hackAvgMu2 += friction->MuSecondary();
    }
  }

  hackAvgMu1 /= static_cast<double>(numCollisions);
  hackAvgMu2 /= static_cast<double>(numCollisions);

  this->dataPtr->dtBodyNode->setFrictionCoeff(0.5 * (hackAvgMu1 + hackAvgMu2));

  // We don't add kido body node to the skeleton here because kido body node
  // should be set its parent joint before being added. This body node will be
  // added to the skeleton in KIDOModel::Init().
}

//////////////////////////////////////////////////
void KIDOLink::Fini()
{
  Link::Fini();
}

/////////////////////////////////////////////////////////////////////
void KIDOLink::UpdateMass()
{
  if (this->dataPtr->dtBodyNode && this->inertial)
  {
    this->dataPtr->dtBodyNode->setMass(this->inertial->GetMass());
    auto Ixxyyzz = this->inertial->GetPrincipalMoments();
    auto Ixyxzyz = this->inertial->GetProductsofInertia();
    this->dataPtr->dtBodyNode->setMomentOfInertia(
        Ixxyyzz[0], Ixxyyzz[1], Ixxyyzz[2],
        Ixyxzyz[0], Ixyxzyz[1], Ixyxzyz[2]);
    auto cog = KIDOTypes::ConvVec3(this->inertial->GetCoG());
    this->dataPtr->dtBodyNode->setLocalCOM(cog);
  }
}

//////////////////////////////////////////////////
void KIDOLink::OnPoseChange()
{
  Link::OnPoseChange();

  // KIDO body node always have its parent joint.
  kido::dynamics::Joint *joint = this->dataPtr->dtBodyNode->getParentJoint();

  // This is for the case this function called before KIDOModel::Init() is
  // called.
  if (joint == NULL)
    return;

  kido::dynamics::FreeJoint *freeJoint =
      dynamic_cast<kido::dynamics::FreeJoint*>(joint);
  if (freeJoint)
  {
    // If the parent joint is free joint, set the 6 dof to fit the target pose.
    const Eigen::Isometry3d &W = KIDOTypes::ConvPose(this->GetWorldPose());
    const Eigen::Isometry3d &T1 = joint->getTransformFromParentBodyNode();
    const Eigen::Isometry3d &InvT2 = joint->getTransformFromChildBodyNode();
    Eigen::Isometry3d P = Eigen::Isometry3d::Identity();

    if (this->dataPtr->dtBodyNode->getParentBodyNode())
      P = this->dataPtr->dtBodyNode->getParentBodyNode()->getTransform();

    Eigen::Isometry3d Q = T1.inverse() * P.inverse() * W * InvT2;

    // Convert homogeneous transformation matrix to 6-dimensional generalized
    // coordinates. There are several ways of conversions. Here is the way of
    // KIDO. The orientation part is converted by using logarithm map, which
    // maps SO(3) to so(3), and it takes the first three components of the
    // generalized coordinates. On the other hand, the position part just takes
    // the last three components of the generalized coordinates without any
    // conversion.
    Eigen::Vector6d q;
    q.head<3>() = kido::math::logMap(Q.linear());
    q.tail<3>() = Q.translation();
    freeJoint->setPositions(q);
    // TODO: The above 4 lines will be reduced to single line as:
    // freeJoint->setPositions(FreeJoint::convertToPositions(Q));
    // after the following PR is merged:
    // https://github.com/kidosim/kido/pull/322

    // Update all the transformations of the links in the parent model.
    freeJoint->getSkeleton()->computeForwardKinematics(true, false, false);
    // TODO: This kinematic updating will be done automatically after pull
    // request (https://github.com/kidosim/kido/pull/319) is merged so that
    // we don't need this line anymore.
  }
  else
  {
    gzdbg << "OnPoseChange() doesn't make sense if the parent joint "
          << "is not free joint (6-dof).\n";
  }
}

//////////////////////////////////////////////////
void KIDOLink::SetEnabled(bool /*_enable*/) const
{
  // TODO: KIDO does not support this functionality.
}

//////////////////////////////////////////////////
bool KIDOLink::GetEnabled() const
{
  // TODO: KIDO does not support this functionality.
  return true;
}

//////////////////////////////////////////////////
void KIDOLink::SetLinearVel(const math::Vector3 &_vel)
{
  // KIDO body node always have its parent joint.
  kido::dynamics::Joint *joint = this->dataPtr->dtBodyNode->getParentJoint();

  // This is for the case this function called before KIDOModel::Init() is
  // called.
  if (joint == NULL)
  {
    gzerr << "KIDOModel::Init() should be called first.\n";
    return;
  }

  // Check if the parent joint is free joint
  kido::dynamics::FreeJoint *freeJoint =
      dynamic_cast<kido::dynamics::FreeJoint*>(joint);

  // If the parent joint is free joint, set the proper generalized velocity to
  // fit the linear velocity of the link
  if (freeJoint)
  {
    // Generalized velocities
    Eigen::Vector3d genVel = KIDOTypes::ConvVec3(_vel);

    kido::dynamics::BodyNode *dtBodyNode = this->dataPtr->dtBodyNode;

    // If this link has parent link then subtract the effect of parent link's
    // linear and angular velocities
    if (dtBodyNode->getParentBodyNode())
    {
      // Local transformation from the parent link frame to this link frame
      Eigen::Isometry3d T = freeJoint->getLocalTransform();

      // Parent link's linear and angular velocities
      Eigen::Vector3d parentLinVel =
          dtBodyNode->getParentBodyNode()->getBodyLinearVelocity();
      Eigen::Vector3d parentAngVel =
          dtBodyNode->getParentBodyNode()->getBodyAngularVelocity();

      // The effect of the parent link's velocities
      Eigen::Vector3d propagatedLinVel =
          T.linear().transpose() *
          (parentAngVel.cross(T.translation()) + parentLinVel);

      // Subtract the effect
      genVel -= propagatedLinVel;
    }

    // Rotation matrix from world frame to this link frame
    Eigen::Matrix3d R = dtBodyNode->getTransform().linear();

    // Change the reference frame to world
    genVel = R * genVel;

    // Set the generalized velocities
    freeJoint->setVelocity(3, genVel[0]);
    freeJoint->setVelocity(4, genVel[1]);
    freeJoint->setVelocity(5, genVel[2]);

    // Update spatial velocities of all the links in the model
    freeJoint->getSkeleton()->computeForwardKinematics(false, true, false);
  }
  else
  {
    gzdbg << "KIDOLink::SetLinearVel() doesn't make sense if the parent joint "
          << "is not free joint (6-dof).\n";
  }
}

//////////////////////////////////////////////////
void KIDOLink::SetAngularVel(const math::Vector3 &_vel)
{
  // KIDO body node always have its parent joint.
  kido::dynamics::Joint *joint = this->dataPtr->dtBodyNode->getParentJoint();

  // This is for the case this function called before KIDOModel::Init() is
  // called.
  if (joint == NULL)
  {
    gzerr << "KIDOModel::Init() should be called first.\n";
    return;
  }

  // Check if the parent joint is free joint
  kido::dynamics::FreeJoint *freeJoint =
      dynamic_cast<kido::dynamics::FreeJoint*>(joint);

  // If the parent joint is free joint, set the proper generalized velocity to
  // fit the linear velocity of the link
  if (freeJoint)
  {
    // Generalized velocities
    Eigen::Vector3d genVel = KIDOTypes::ConvVec3(_vel);

    kido::dynamics::BodyNode *dtBodyNode = this->dataPtr->dtBodyNode;

    // If this link has parent link then subtract the effect of parent link's
    // linear and angular velocities
    if (dtBodyNode->getParentBodyNode())
    {
      // Local transformation from the parent link frame to this link frame
      Eigen::Isometry3d T = freeJoint->getLocalTransform();

      // Parent link's linear and angular velocities
      Eigen::Vector3d parentAngVel =
          dtBodyNode->getParentBodyNode()->getBodyAngularVelocity();

      // The effect of the parent link's velocities
      Eigen::Vector3d propagatedAngVel = T.linear().transpose() * parentAngVel;

      // Subtract the effect
      genVel -= propagatedAngVel;
    }

    // Rotation matrix from world frame to this link frame
    Eigen::Matrix3d R = dtBodyNode->getTransform().linear();

    // Change the reference frame to world
    genVel = R * genVel;

    // Set the generalized velocities
    freeJoint->setVelocity(0, genVel[0]);
    freeJoint->setVelocity(1, genVel[1]);
    freeJoint->setVelocity(2, genVel[2]);

    // Update spatial velocities of all the links in the model
    freeJoint->getSkeleton()->computeForwardKinematics(false, true, false);
  }
  else
  {
    gzdbg << "KIDOLink::SetLinearVel() doesn't make sense if the parent joint "
          << "is not free joint (6-dof).\n";
  }
}

//////////////////////////////////////////////////
void KIDOLink::SetForce(const math::Vector3 &_force)
{
  // KIDO assume that _force is external force.
  this->dataPtr->dtBodyNode->setExtForce(KIDOTypes::ConvVec3(_force));
}

//////////////////////////////////////////////////
void KIDOLink::SetTorque(const math::Vector3 &_torque)
{
  // KIDO assume that _torque is external torque.
  this->dataPtr->dtBodyNode->setExtTorque(KIDOTypes::ConvVec3(_torque));
}

//////////////////////////////////////////////////
void KIDOLink::AddForce(const math::Vector3 &_force)
{
  this->dataPtr->dtBodyNode->addExtForce(KIDOTypes::ConvVec3(_force));
}

/////////////////////////////////////////////////
void KIDOLink::AddRelativeForce(const math::Vector3 &_force)
{
  this->dataPtr->dtBodyNode->addExtForce(KIDOTypes::ConvVec3(_force),
                                Eigen::Vector3d::Zero(),
                                true, true);
}

/////////////////////////////////////////////////
void KIDOLink::AddForceAtWorldPosition(const math::Vector3 &_force,
                                        const math::Vector3 &_pos)
{
  this->dataPtr->dtBodyNode->addExtForce(KIDOTypes::ConvVec3(_pos),
                                KIDOTypes::ConvVec3(_force),
                                false, false);
}

/////////////////////////////////////////////////
void KIDOLink::AddForceAtRelativePosition(const math::Vector3 &_force,
                                          const math::Vector3 &_relpos)
{
  this->dataPtr->dtBodyNode->addExtForce(KIDOTypes::ConvVec3(_force),
                                KIDOTypes::ConvVec3(_relpos),
                                true, true);
}

//////////////////////////////////////////////////
void KIDOLink::AddLinkForce(const math::Vector3 &/*_force*/,
    const math::Vector3 &/*_offset*/)
{
  gzlog << "KIDOLink::AddLinkForce not yet implemented (issue #1477)."
        << std::endl;
}

/////////////////////////////////////////////////
void KIDOLink::AddTorque(const math::Vector3 &_torque)
{
  this->dataPtr->dtBodyNode->addExtTorque(KIDOTypes::ConvVec3(_torque));
}

/////////////////////////////////////////////////
void KIDOLink::AddRelativeTorque(const math::Vector3 &_torque)
{
  this->dataPtr->dtBodyNode->addExtTorque(KIDOTypes::ConvVec3(_torque), true);
}

//////////////////////////////////////////////////
gazebo::math::Vector3 KIDOLink::GetWorldLinearVel(
    const math::Vector3 &_offset) const
{
  Eigen::Vector3d linVel =
      this->dataPtr->dtBodyNode->getWorldLinearVelocity(
        KIDOTypes::ConvVec3(_offset));

  return KIDOTypes::ConvVec3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 KIDOLink::GetWorldLinearVel(
    const gazebo::math::Vector3 &_offset,
    const gazebo::math::Quaternion &_q) const
{
  Eigen::Matrix3d R1 = Eigen::Matrix3d(KIDOTypes::ConvQuat(_q));
  Eigen::Vector3d worldOffset = R1 * KIDOTypes::ConvVec3(_offset);
  Eigen::Vector3d bodyOffset =
      this->dataPtr->dtBodyNode->getTransform().linear().transpose() *
      worldOffset;
  Eigen::Vector3d linVel =
      this->dataPtr->dtBodyNode->getWorldLinearVelocity(bodyOffset);

  return KIDOTypes::ConvVec3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 KIDOLink::GetWorldCoGLinearVel() const
{
  Eigen::Vector3d linVel = this->dataPtr->dtBodyNode->getWorldCOMVelocity();

  return KIDOTypes::ConvVec3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 KIDOLink::GetWorldAngularVel() const
{
  Eigen::Vector3d angVel = this->dataPtr->dtBodyNode->getWorldAngularVelocity();

  return KIDOTypes::ConvVec3(angVel);
}

/////////////////////////////////////////////////
math::Vector3 KIDOLink::GetWorldForce() const
{
  Eigen::Vector6d F = this->dataPtr->dtBodyNode->getExternalForceGlobal();
  return KIDOTypes::ConvVec3(F.tail<3>());
}

//////////////////////////////////////////////////
math::Vector3 KIDOLink::GetWorldTorque() const
{
  // TODO: Need verification
  math::Vector3 torque;

  Eigen::Isometry3d W = this->dataPtr->dtBodyNode->getTransform();
  Eigen::Matrix6d G   = this->dataPtr->dtBodyNode->getSpatialInertia();
  Eigen::VectorXd V   = this->dataPtr->dtBodyNode->getBodyVelocity();
  Eigen::VectorXd dV  = this->dataPtr->dtBodyNode->getBodyAcceleration();
  Eigen::Vector6d F   = G * dV - kido::math::dad(V, G * V);

  torque = KIDOTypes::ConvVec3(W.linear() * F.head<3>());

  return torque;
}

//////////////////////////////////////////////////
void KIDOLink::SetGravityMode(bool _mode)
{
  this->sdf->GetElement("gravity")->Set(_mode);
  this->dataPtr->dtBodyNode->setGravityMode(_mode);
}

//////////////////////////////////////////////////
bool KIDOLink::GetGravityMode() const
{
  return this->dataPtr->dtBodyNode->getGravityMode();
}

//////////////////////////////////////////////////
void KIDOLink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);

  kido::dynamics::BodyNode *dtBodyNode = this->dataPtr->dtBodyNode;

  // If this function is called before the body node is not added to a skeleton,
  // the body node does not have parent skeleton. So we just return here. Self
  // collision setting will be done later in KIDOModel::Init().
  if (dtBodyNode->getSkeleton() == NULL)
    return;

  kido::simulation::World *dtWorld = this->dataPtr->kidoPhysics->GetKIDOWorld();
  kido::dynamics::Skeleton *dtSkeleton = dtBodyNode->getSkeleton();
  kido::collision::CollisionDetector *dtCollDet =
      dtWorld->getConstraintSolver()->getCollisionDetector();

  Link_V links = this->GetModel()->GetLinks();

  bool isSkeletonSelfCollidable =
      dtBodyNode->getSkeleton()->isEnabledSelfCollisionCheck();

  if (_collide)
  {
    // If the skeleton is already self collidable, then we enable self
    // collidable pairs those are associated with this link. The links in the
    // pairs should be all and not itself each other.
    if (isSkeletonSelfCollidable)
    {
      for (size_t i = 0; i < links.size(); ++i)
      {
        if (links[i].get() != this && links[i]->GetSelfCollide())
        {
          kido::dynamics::BodyNode *itdtBodyNode =
            boost::dynamic_pointer_cast<KIDOLink>(links[i])->GetKIDOBodyNode();

          // If this->dataPtr->dtBodyNode and itdtBodyNode are connected then
          // don't enable the pair.
          // Please see: https://bitbucket.org/osrf/gazebo/issue/899
          if ((dtBodyNode->getParentBodyNode() == itdtBodyNode) ||
              itdtBodyNode->getParentBodyNode() == this->dataPtr->dtBodyNode)
            continue;

          dtCollDet->enablePair(dtBodyNode, itdtBodyNode);
        }
      }
    }
    // If the skeleton is not self collidable, we first set the skeleton as
    // self collidable. If the skeleton is self collidable, then KIDO regards
    // that all the links in the skeleton is self collidable. So, we disable all
    // the pairs of which both of the links in the pair is not self collidable.
    else
    {
      dtSkeleton->enableSelfCollision();

      for (size_t i = 0; i < links.size() - 1; ++i)
      {
        for (size_t j = i + 1; j < links.size(); ++j)
        {
          kido::dynamics::BodyNode *itdtBodyNode1 =
            boost::dynamic_pointer_cast<KIDOLink>(links[i])->GetKIDOBodyNode();
          kido::dynamics::BodyNode *itdtBodyNode2 =
            boost::dynamic_pointer_cast<KIDOLink>(links[j])->GetKIDOBodyNode();

          // If this->dataPtr->dtBodyNode and itdtBodyNode are connected then
          // don't enable the pair.
          // Please see: https://bitbucket.org/osrf/gazebo/issue/899
          if ((itdtBodyNode1->getParentBodyNode() == itdtBodyNode2) ||
              itdtBodyNode2->getParentBodyNode() == itdtBodyNode1)
            dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);

          if (!links[i]->GetSelfCollide() || !links[j]->GetSelfCollide())
            dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);
        }
      }
    }
  }
  else
  {
    // If the skeleton is self collidable, then we disable all the pairs
    // associated with this link.
    if (isSkeletonSelfCollidable)
    {
      for (size_t i = 0; i < links.size(); ++i)
      {
        if (links[i].get() != this)
        {
          kido::dynamics::BodyNode *itdtBodyNode =
            boost::dynamic_pointer_cast<KIDOLink>(links[i])->GetKIDOBodyNode();
          dtCollDet->disablePair(dtBodyNode, itdtBodyNode);
        }
      }
    }

    // If now all the links are not self collidable, then we set the skeleton
    // as not self collidable.
    bool isAllLinksNotCollidable = true;
    for (size_t i = 0; i < links.size(); ++i)
    {
      if (links[i]->GetSelfCollide())
      {
        isAllLinksNotCollidable = false;
        break;
      }
    }
    if (isAllLinksNotCollidable)
      dtSkeleton->disableSelfCollision();
  }
}

//////////////////////////////////////////////////
void KIDOLink::SetLinearDamping(double /*_damping*/)
{
  // see: https://github.com/kidosim/kido/issues/85
  gzwarn << "KIDO does not support KIDOLink::SetLinearDamping() yet.\n";
}

//////////////////////////////////////////////////
void KIDOLink::SetAngularDamping(double /*_damping*/)
{
  // see: https://github.com/kidosim/kido/issues/85
  gzwarn << "KIDO does not support KIDOLink::SetAngularDamping() yet.\n";
}

//////////////////////////////////////////////////
void KIDOLink::SetKinematic(const bool& _state)
{
  this->sdf->GetElement("kinematic")->Set(_state);

  gzwarn << "KIDO does not support KIDOLink::SetKinematic() yet.\n";
}

//////////////////////////////////////////////////
bool KIDOLink::GetKinematic() const
{
  // KIDO does not support kinematic mode for link.";
  return false;
}

//////////////////////////////////////////////////
void KIDOLink::SetAutoDisable(bool /*_disable*/)
{
  gzwarn << "KIDO does not support KIDOLink::SetAutoDisable() yet.\n";
}

//////////////////////////////////////////////////
void KIDOLink::SetLinkStatic(bool _static)
{
  if (_static == this->dataPtr->staticLink)
    return;

  if (_static == true)
  {
    // Add weld joint constraint to KIDO
    this->dataPtr->dtWeldJointConst =
        new kido::constraint::WeldJointConstraint(this->dataPtr->dtBodyNode);
    GetKIDOWorld()->getConstraintSolver()->addConstraint(
        this->dataPtr->dtWeldJointConst);
  }
  else
  {
    // Remove ball and revolute joint constraints from KIDO
    GetKIDOWorld()->getConstraintSolver()->removeConstraint(
        this->dataPtr->dtWeldJointConst);
    delete this->dataPtr->dtWeldJointConst;
    this->dataPtr->dtWeldJointConst = NULL;
  }

  this->dataPtr->staticLink = _static;
}

//////////////////////////////////////////////////
void KIDOLink::updateDirtyPoseFromKIDOTransformation()
{
  // Step 1: get kido body's transformation
  // Step 2: set gazebo link's pose using the transformation
  math::Pose newPose = KIDOTypes::ConvPose(
                         this->dataPtr->dtBodyNode->getTransform());

  // Set the new pose to this link
  this->dirtyPose = newPose;

  // Set the new pose to the world
  // (Below method can be changed in gazebo code)
  this->world->dataPtr->dirtyPoses.push_back(this);
}

//////////////////////////////////////////////////
KIDOPhysicsPtr KIDOLink::GetKIDOPhysics(void) const
{
  return boost::dynamic_pointer_cast<KIDOPhysics>(
        this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
kido::simulation::World *KIDOLink::GetKIDOWorld(void) const
{
  return GetKIDOPhysics()->GetKIDOWorld();
}

//////////////////////////////////////////////////
KIDOModelPtr KIDOLink::GetKIDOModel() const
{
  return boost::dynamic_pointer_cast<KIDOModel>(this->GetModel());
}

//////////////////////////////////////////////////
kido::dynamics::BodyNode *KIDOLink::GetKIDOBodyNode() const
{
  return this->dataPtr->dtBodyNode;
}

//////////////////////////////////////////////////
void KIDOLink::SetKIDOParentJoint(KIDOJointPtr _kidoParentJoint)
{
  this->dataPtr->kidoParentJoint = _kidoParentJoint;
}

//////////////////////////////////////////////////
void KIDOLink::AddKIDOChildJoint(KIDOJointPtr _kidoChildJoint)
{
  this->dataPtr->kidoChildJoints.push_back(_kidoChildJoint);
}
