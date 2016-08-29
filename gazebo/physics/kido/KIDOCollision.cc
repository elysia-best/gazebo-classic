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

#include <sstream>

#include "gazebo/common/Console.hh"
#include "gazebo/math/Box.hh"

#include "gazebo/physics/SurfaceParams.hh"

#include "gazebo/physics/kido/kido_inc.h"
#include "gazebo/physics/kido/KIDOLink.hh"
#include "gazebo/physics/kido/KIDOCollision.hh"
#include "gazebo/physics/kido/KIDOPlaneShape.hh"
#include "gazebo/physics/kido/KIDOSurfaceParams.hh"

#include "gazebo/physics/kido/KIDOCollisionPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
KIDOCollision::KIDOCollision(LinkPtr _link)
  : Collision(_link),
    dataPtr(new KIDOCollisionPrivate(
      boost::static_pointer_cast<KIDOLink>(this->link)->GetKIDOBodyNode()))

{
  this->SetName("KIDO_Collision");
  this->surface.reset(new KIDOSurfaceParams());
}

//////////////////////////////////////////////////
KIDOCollision::~KIDOCollision()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void KIDOCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }
}

//////////////////////////////////////////////////
void KIDOCollision::Init()
{
  Collision::Init();

  // Offset
  if (this->dataPtr->dtCollisionShape)
  {
    boost::shared_ptr<KIDOPlaneShape> planeShape =
        boost::dynamic_pointer_cast<KIDOPlaneShape>(this->shape);

    if (!planeShape)
    {
      math::Pose relativePose = this->GetRelativePose();
      this->dataPtr->dtCollisionShape->setLocalTransform(
            KIDOTypes::ConvPose(relativePose));
    }
    else
    {
      // change ground plane to be near semi-infinite.
      kido::dynamics::BoxShape *dtBoxShape =
          dynamic_cast<kido::dynamics::BoxShape *>(
            this->dataPtr->dtCollisionShape);
      dtBoxShape->setSize(Eigen::Vector3d(2100, 2100, 2100.0));
      dtBoxShape->setOffset(Eigen::Vector3d(0.0, 0.0, -2100.0/2.0));
    }
    // TODO: Remove this specialized code for plane shape once
    // https://github.com/kidosim/kido/issues/114 is resolved.
  }
}

//////////////////////////////////////////////////
void KIDOCollision::Fini()
{
  Collision::Fini();
}

//////////////////////////////////////////////////
void KIDOCollision::OnPoseChange()
{
  // Nothing to do in kido.
}

//////////////////////////////////////////////////
void KIDOCollision::SetCategoryBits(unsigned int _bits)
{
  this->dataPtr->categoryBits = _bits;
}

//////////////////////////////////////////////////
void KIDOCollision::SetCollideBits(unsigned int _bits)
{
  this->dataPtr->collideBits = _bits;
}

//////////////////////////////////////////////////
unsigned int KIDOCollision::GetCategoryBits() const
{
  return this->dataPtr->categoryBits;
}

//////////////////////////////////////////////////
unsigned int KIDOCollision::GetCollideBits() const
{
  return this->dataPtr->collideBits;
}

//////////////////////////////////////////////////
gazebo::math::Box KIDOCollision::GetBoundingBox() const
{
  math::Box result;

  gzerr << "KIDO does not provide bounding box info.\n";

  return result;
}

//////////////////////////////////////////////////
kido::dynamics::BodyNode *KIDOCollision::GetKIDOBodyNode() const
{
  return this->dataPtr->dtBodyNode;
}

//////////////////////////////////////////////////
void KIDOCollision::SetKIDOCollisionShape(kido::dynamics::Shape *_shape,
                                          bool _placeable)
{
  Collision::SetCollision(_placeable);
  this->dataPtr->dtCollisionShape = _shape;
}

//////////////////////////////////////////////////
kido::dynamics::Shape *KIDOCollision::GetKIDOCollisionShape() const
{
  return this->dataPtr->dtCollisionShape;
}

/////////////////////////////////////////////////
KIDOSurfaceParamsPtr KIDOCollision::GetKIDOSurface() const
{
  return boost::dynamic_pointer_cast<KIDOSurfaceParams>(this->surface);
}
