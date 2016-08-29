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
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/kido/KIDOPhysics.hh"
#include "gazebo/physics/kido/KIDOTypes.hh"
#include "gazebo/physics/kido/KIDOLink.hh"
#include "gazebo/physics/kido/KIDOCollision.hh"
#include "gazebo/physics/kido/KIDORayShape.hh"

#include "gazebo/physics/kido/KIDORayShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDORayShape::KIDORayShape(PhysicsEnginePtr _physicsEngine)
  : RayShape(_physicsEngine),
    dataPtr(new KIDORayShapePrivate())
{
  this->SetName("KIDO_ray_shape");
}

//////////////////////////////////////////////////
KIDORayShape::KIDORayShape(CollisionPtr _parent)
  : RayShape(_parent),
    dataPtr(new KIDORayShapePrivate())
{
  this->SetName("KIDO_ray_shape");
}

//////////////////////////////////////////////////
KIDORayShape::~KIDORayShape()
{
}

//////////////////////////////////////////////////
void KIDORayShape::Update()
{
  // Not implemented yet, please see issue #911
}

//////////////////////////////////////////////////
void KIDORayShape::GetIntersection(double &_dist, std::string &_entity)
{
  _dist = 0;
  _entity = "";

  // Not implemented yet, please see issue #911
}

//////////////////////////////////////////////////
void KIDORayShape::SetPoints(const math::Vector3& _posStart,
                             const math::Vector3& _posEnd)
{
  RayShape::SetPoints(_posStart, _posEnd);

  // Not implemented yet, please see issue #911
}
