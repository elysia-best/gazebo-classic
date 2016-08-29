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

#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/kido/KIDOTypes.hh"
#include "gazebo/physics/kido/KIDOLink.hh"
#include "gazebo/physics/kido/KIDOCollision.hh"
#include "gazebo/physics/kido/KIDOPhysics.hh"
#include "gazebo/physics/kido/KIDORayShape.hh"
#include "gazebo/physics/kido/KIDOMultiRayShape.hh"

#include "gazebo/physics/kido/KIDOMultiRayShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOMultiRayShape::KIDOMultiRayShape(CollisionPtr _parent)
  : MultiRayShape(_parent),
    dataPtr(new KIDOMultiRayShapePrivate())
{
  this->SetName("KIDO_multiray_shape");
}

//////////////////////////////////////////////////
KIDOMultiRayShape::~KIDOMultiRayShape()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void KIDOMultiRayShape::UpdateRays()
{
  std::vector<RayShapePtr>::iterator iter;
  for (iter = this->rays.begin(); iter != this->rays.end(); ++iter)
  {
    (*iter)->Update();
  }
}

//////////////////////////////////////////////////
void KIDOMultiRayShape::AddRay(const math::Vector3& _start,
                               const math::Vector3& _end)
{
  MultiRayShape::AddRay(_start, _end);

  KIDORayShapePtr ray(new KIDORayShape(this->collisionParent));
  ray->SetPoints(_start, _end);

  this->rays.push_back(ray);
}
