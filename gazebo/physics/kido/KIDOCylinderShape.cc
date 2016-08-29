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
#include "gazebo/physics/kido/KIDOCylinderShape.hh"
#include "gazebo/util/system.hh"

#include "gazebo/physics/kido/KIDOCylinderShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOCylinderShape::KIDOCylinderShape(CollisionPtr _parent)
  : CylinderShape(_parent),
    dataPtr(new KIDOCylinderShapePrivate())
{
}

//////////////////////////////////////////////////
KIDOCylinderShape::~KIDOCylinderShape()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void KIDOCylinderShape::SetSize(double _radius, double _length)
{
  if (_radius < 0)
  {
    gzerr << "Cylinder shape does not support negative radius\n";
    return;
  }

  if (_length < 0)
  {
    gzerr << "Cylinder shape does not support negative length\n";
    return;
  }

  if (math::equal(_radius, 0.0))
  {
    // Warn user, but still create shape with very small value
    // otherwise later resize operations using setLocalScaling
    // will not be possible
    gzwarn << "Setting cylinder shape's radius to zero not supported "
           << "in KIDO, using 1e-4.\n";
    _radius = 1e-4;
  }

  if (math::equal(_length, 0.0))
  {
    gzwarn << "Setting cylinder shape's length to zero not supported "
           << "in KIDO, using 1e-4.\n";
    _length = 1e-4;
  }

  CylinderShape::SetSize(_radius, _length);

  KIDOCollisionPtr kidoCollisionParent =
      boost::dynamic_pointer_cast<KIDOCollision>(this->collisionParent);

  if (kidoCollisionParent->GetKIDOCollisionShape() == NULL)
  {
    kido::dynamics::BodyNode *dtBodyNode =
        kidoCollisionParent->GetKIDOBodyNode();
    kido::dynamics::CylinderShape *dtCylinderShape =
        new kido::dynamics::CylinderShape(_radius, _length);
    dtBodyNode->addCollisionShape(dtCylinderShape);
    kidoCollisionParent->SetKIDOCollisionShape(dtCylinderShape);
  }
  else
  {
    kido::dynamics::CylinderShape *dtCylinderShape =
        dynamic_cast<kido::dynamics::CylinderShape*>(
          kidoCollisionParent->GetKIDOCollisionShape());
    dtCylinderShape->setRadius(_radius);
    dtCylinderShape->setHeight(_length);
  }
}
