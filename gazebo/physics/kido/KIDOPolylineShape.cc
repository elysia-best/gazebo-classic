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

#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/kido/KIDOMesh.hh"
#include "gazebo/physics/kido/KIDOCollision.hh"
#include "gazebo/physics/kido/KIDOPhysics.hh"
#include "gazebo/physics/kido/KIDOPolylineShape.hh"

#include "gazebo/physics/kido/KIDOPolylineShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOPolylineShape::KIDOPolylineShape(CollisionPtr _parent)
  : PolylineShape(_parent),
    dataPtr(new KIDOPolylineShapePrivate())
{
}

//////////////////////////////////////////////////
KIDOPolylineShape::~KIDOPolylineShape()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void KIDOPolylineShape::Load(sdf::ElementPtr _sdf)
{
  PolylineShape::Load(_sdf);
}

//////////////////////////////////////////////////
void KIDOPolylineShape::Init()
{
  PolylineShape::Init();
  if (!this->mesh)
  {
    gzerr << "Unable to create polyline in KIDO. Mesh pointer is null.\n";
    return;
  }

  this->dataPtr->kidoMesh->Init(this->mesh,
      boost::static_pointer_cast<KIDOCollision>(this->collisionParent),
      math::Vector3(1, 1, 1));
}
