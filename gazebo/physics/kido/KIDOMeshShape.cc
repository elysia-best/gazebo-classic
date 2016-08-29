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
#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/kido/KIDOMesh.hh"
#include "gazebo/physics/kido/KIDOCollision.hh"
#include "gazebo/physics/kido/KIDOMeshShape.hh"
#include "gazebo/physics/kido/KIDOPhysics.hh"

#include "gazebo/physics/kido/KIDOMeshShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOMeshShape::KIDOMeshShape(CollisionPtr _parent)
  : MeshShape(_parent),
    dataPtr(new KIDOMeshShapePrivate())
{
}

//////////////////////////////////////////////////
KIDOMeshShape::~KIDOMeshShape()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void KIDOMeshShape::Update()
{
  MeshShape::Update();
}

//////////////////////////////////////////////////
void KIDOMeshShape::Load(sdf::ElementPtr _sdf)
{
  MeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void KIDOMeshShape::Init()
{
  MeshShape::Init();

  if (this->submesh)
  {
    this->dataPtr->kidoMesh->Init(this->submesh,
        boost::dynamic_pointer_cast<KIDOCollision>(this->collisionParent),
        this->sdf->Get<math::Vector3>("scale"));
  }
  else
  {
    this->dataPtr->kidoMesh->Init(this->mesh,
        boost::dynamic_pointer_cast<KIDOCollision>(this->collisionParent),
        this->sdf->Get<math::Vector3>("scale"));
  }
}
