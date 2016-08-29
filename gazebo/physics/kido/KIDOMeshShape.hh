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

#ifndef _GAZEBO_KIDOMESHSHAPE_HH_
#define _GAZEBO_KIDOMESHSHAPE_HH_

#include "gazebo/physics/MeshShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class KIDOMeshShapePrivate;

    /// \addtogroup gazebo_physics_kido
    /// \{

    /// \brief Triangle mesh collision.
    class GZ_PHYSICS_VISIBLE KIDOMeshShape : public MeshShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent collision object.
      public: explicit KIDOMeshShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~KIDOMeshShape();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Update();

      /// \internal
      /// \brief Pointer to private data
      private: KIDOMeshShapePrivate *dataPtr;
    };
    /// \}
  }
}
#endif
