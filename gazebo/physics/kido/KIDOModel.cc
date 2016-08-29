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

#include "gazebo/physics/World.hh"

#include "gazebo/physics/kido/KIDOPhysics.hh"
#include "gazebo/physics/kido/KIDOLink.hh"
#include "gazebo/physics/kido/KIDOModel.hh"

#include "gazebo/physics/kido/KIDOModelPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOModel::KIDOModel(BasePtr _parent)
  : Model(_parent), dataPtr(new KIDOModelPrivate())
{
}

//////////////////////////////////////////////////
KIDOModel::~KIDOModel()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void KIDOModel::Load(sdf::ElementPtr _sdf)
{
  if (_sdf->HasElement("model"))
  {
    gzerr << "Nested models are not currently supported in KIDO. ["
      << _sdf->Get<std::string>("name") << "] will not be loaded. "
      << std::endl;
    this->sdf = _sdf;
    return;
  }

  // create skeleton of KIDO
  this->dataPtr->dtSkeleton = new kido::dynamics::Skeleton();

  Model::Load(_sdf);
}

//////////////////////////////////////////////////
void KIDOModel::Init()
{
  // nested models are not supported for now, issue #1833
  if (this->sdf->HasElement("model"))
    return;

  Model::Init();

  //----------------------------------------------
  // Name
  std::string modelName = this->GetName();
  this->dataPtr->dtSkeleton->setName(modelName.c_str());

  //----------------------------------------------
  // Static
  this->dataPtr->dtSkeleton->setMobile(!this->IsStatic());

  //----------------------------------------------
  // Check if this link is free floating body
  // If a link of this model has no parent joint, then we add 6-dof free joint
  // to the link.
  Link_V linkList = this->GetLinks();
  for (unsigned int i = 0; i < linkList.size(); ++i)
  {
    kido::dynamics::BodyNode *dtBodyNode
        = boost::static_pointer_cast<KIDOLink>(linkList[i])->GetKIDOBodyNode();

    if (dtBodyNode->getParentJoint() == NULL)
    {
      kido::dynamics::FreeJoint *newFreeJoint = new kido::dynamics::FreeJoint;

      newFreeJoint->setTransformFromParentBodyNode(
            KIDOTypes::ConvPose(linkList[i]->GetWorldPose()));
      newFreeJoint->setTransformFromChildBodyNode(
        Eigen::Isometry3d::Identity());

      dtBodyNode->setParentJoint(newFreeJoint);
    }

    this->dataPtr->dtSkeleton->addBodyNode(dtBodyNode);
  }

  // Add the skeleton to the world
  this->GetKIDOWorld()->addSkeleton(this->dataPtr->dtSkeleton);

  // Self collision
  // Note: This process should be done after this skeleton is added to the
  //       world.

  // Check whether there exist at least one pair of self collidable links.
  int numSelfCollidableLinks = 0;
  bool hasPairOfSelfCollidableLinks = false;
  for (size_t i = 0; i < linkList.size(); ++i)
  {
    if (linkList[i]->GetSelfCollide())
    {
      ++numSelfCollidableLinks;
      if (numSelfCollidableLinks >= 2)
      {
        hasPairOfSelfCollidableLinks = true;
        break;
      }
    }
  }

  // If the skeleton has at least two self collidable links, then we set the
  // skeleton as self collidable. If the skeleton is self collidable, then
  // KIDO regards that all the links in the skeleton is self collidable. So, we
  // disable all the pairs of which both of the links in the pair is not self
  // collidable.
  if (hasPairOfSelfCollidableLinks)
  {
    this->dataPtr->dtSkeleton->enableSelfCollision();

    kido::simulation::World *dtWorld = this->GetKIDOPhysics()->GetKIDOWorld();
    kido::collision::CollisionDetector *dtCollDet =
        dtWorld->getConstraintSolver()->getCollisionDetector();

    for (size_t i = 0; i < linkList.size() - 1; ++i)
    {
      for (size_t j = i + 1; j < linkList.size(); ++j)
      {
        kido::dynamics::BodyNode *itdtBodyNode1 =
          boost::dynamic_pointer_cast<KIDOLink>(linkList[i])->GetKIDOBodyNode();
        kido::dynamics::BodyNode *itdtBodyNode2 =
          boost::dynamic_pointer_cast<KIDOLink>(linkList[j])->GetKIDOBodyNode();

        // If this->dtBodyNode and itdtBodyNode are connected then don't enable
        // the pair.
        // Please see: https://bitbucket.org/osrf/gazebo/issue/899
        if ((itdtBodyNode1->getParentBodyNode() == itdtBodyNode2) ||
            itdtBodyNode2->getParentBodyNode() == itdtBodyNode1)
        {
          dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);
        }

        if (!linkList[i]->GetSelfCollide() || !linkList[j]->GetSelfCollide())
        {
          dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);
        }
      }
    }
  }

  // Note: This function should be called after the skeleton is added to the
  //       world.
  this->BackupState();
}


//////////////////////////////////////////////////
void KIDOModel::Update()
{
  Model::Update();
}

//////////////////////////////////////////////////
void KIDOModel::Fini()
{
  Model::Fini();
}

//////////////////////////////////////////////////
void KIDOModel::BackupState()
{
  this->dataPtr->dtConfig = this->dataPtr->dtSkeleton->getPositions();
  this->dataPtr->dtVelocity = this->dataPtr->dtSkeleton->getVelocities();
}

//////////////////////////////////////////////////
void KIDOModel::RestoreState()
{
  if (!this->dataPtr->dtSkeleton)
    return;

  GZ_ASSERT(static_cast<size_t>(this->dataPtr->dtConfig.size()) ==
            this->dataPtr->dtSkeleton->getNumDofs(),
            "Cannot RestoreState, invalid size");
  GZ_ASSERT(static_cast<size_t>(this->dataPtr->dtVelocity.size()) ==
            this->dataPtr->dtSkeleton->getNumDofs(),
            "Cannot RestoreState, invalid size");

  this->dataPtr->dtSkeleton->setPositions(this->dataPtr->dtConfig);
  this->dataPtr->dtSkeleton->setVelocities(this->dataPtr->dtVelocity);
  this->dataPtr->dtSkeleton->computeForwardKinematics(true, true, false);
}

//////////////////////////////////////////////////
kido::dynamics::Skeleton *KIDOModel::GetKIDOSkeleton()
{
  return this->dataPtr->dtSkeleton;
}

//////////////////////////////////////////////////
KIDOPhysicsPtr KIDOModel::GetKIDOPhysics(void) const
{
  return boost::dynamic_pointer_cast<KIDOPhysics>(
    this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
kido::simulation::World *KIDOModel::GetKIDOWorld(void) const
{
  return GetKIDOPhysics()->GetKIDOWorld();
}
