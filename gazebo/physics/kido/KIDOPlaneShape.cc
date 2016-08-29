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

#include "gazebo/common/Console.hh"
#include "gazebo/physics/kido/KIDOPhysics.hh"
#include "gazebo/physics/kido/KIDOCollision.hh"
#include "gazebo/physics/kido/KIDOPlaneShape.hh"
#include "gazebo/util/system.hh"

#include "gazebo/physics/kido/KIDOPlaneShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOPlaneShape::KIDOPlaneShape(CollisionPtr _parent)
  : PlaneShape(_parent),
    dataPtr(new KIDOPlaneShapePrivate())
{
}

//////////////////////////////////////////////////
KIDOPlaneShape::~KIDOPlaneShape()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void KIDOPlaneShape::CreatePlane()
{
  PlaneShape::CreatePlane();

  KIDOCollisionPtr kidoCollisionParent =
      boost::dynamic_pointer_cast<KIDOCollision>(this->collisionParent);

  // math::Vector3 n = this->GetNormal();

  kido::dynamics::BodyNode *dtBodyNode =
      kidoCollisionParent->GetKIDOBodyNode();
  kido::dynamics::BoxShape *dtBoxShape =
      new kido::dynamics::BoxShape(Eigen::Vector3d(2100, 2100, 0.01));
  dtBodyNode->addCollisionShape(dtBoxShape);
  dtBoxShape->setOffset(Eigen::Vector3d(0.0, 0.0, -0.005));
  kidoCollisionParent->SetKIDOCollisionShape(dtBoxShape, false);
}

//////////////////////////////////////////////////
void KIDOPlaneShape::SetAltitude(const math::Vector3 &_pos)
{
  PlaneShape::SetAltitude(_pos);
}

