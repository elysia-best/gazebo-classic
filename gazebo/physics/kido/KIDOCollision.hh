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

#ifndef _GAZEBO_KIDOCOLLISION_HH_
#define _GAZEBO_KIDOCOLLISION_HH_

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/kido/kido_inc.h"
#include "gazebo/physics/kido/KIDOTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class KIDOCollisionPrivate;

    /// \addtogroup gazebo_physics_kido
    /// \{

    /// \brief Base class for all KIDO collisions.
    class GZ_PHYSICS_VISIBLE KIDOCollision : public Collision
    {
      /// \brief Constructor.
      /// \param[in] _link Parent Link
      public: explicit KIDOCollision(LinkPtr _parent);

      /// \brief Destructor.
      public: virtual ~KIDOCollision();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Fini();

      // Documentation inherited.
      public: virtual void OnPoseChange();

      // Documentation inherited.
      public: virtual void SetCategoryBits(unsigned int _bits);

      // Documentation inherited.
      public: virtual void SetCollideBits(unsigned int _bits);

      /// \brief Get the category bits, used during collision detection
      /// \return The bits
      public: virtual unsigned int GetCategoryBits() const;

      /// \brief Get the collide bits, used during collision detection
      /// \return The bits
      public: virtual unsigned int GetCollideBits() const;

      // Documentation inherited.
      public: virtual math::Box GetBoundingBox() const;

      /// \brief Get KIDO body node.
      /// \return Pointer to the kido BodyNode.
      public: kido::dynamics::BodyNode *GetKIDOBodyNode() const;

      /// \brief Set KIDO collision shape.
      /// \param[in] _shape KIDO Collision shape
      /// \param[in] _placeable True to make the object movable.
      public: void SetKIDOCollisionShape(kido::dynamics::Shape *_shape,
                                         bool _placeable = true);

      /// \brief Get KIDO collision shape.
      public: kido::dynamics::Shape *GetKIDOCollisionShape() const;

      /// \brief Similar to Collision::GetSurface, but provides dynamically
      ///        casted pointer to KIDOSurfaceParams.
      /// \return Dynamically casted pointer to KIDOSurfaceParams.
      public: KIDOSurfaceParamsPtr GetKIDOSurface() const;

      /// \internal
      /// \brief Pointer to private data
      private: KIDOCollisionPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
