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

#ifndef _GAZEBO_KIDOMULTIRAYSHAPE_HH_
#define _GAZEBO_KIDOMULTIRAYSHAPE_HH_

#include "gazebo/physics/MultiRayShape.hh"
#include "gazebo/physics/kido/KIDOTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_kido
    /// \{

    /// Forward declare private data class
    class KIDOMultiRayShapePrivate;

    /// \brief KIDO specific version of MultiRayShape
    class GZ_PHYSICS_VISIBLE KIDOMultiRayShape : public MultiRayShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit KIDOMultiRayShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~KIDOMultiRayShape();

      // Documentation inherited.
      public: virtual void UpdateRays();

      /// \brief Add a ray to the collision.
      /// \param[in] _start Start location of the ray.
      /// \param[in] _end End location of the ray.
      protected: void AddRay(const math::Vector3 &_start,
                             const math::Vector3 &_end);

      /// \internal
      /// \brief Pointer to private data
      private: KIDOMultiRayShapePrivate *dataPtr;
    };
    /// \}
  }
}
#endif
