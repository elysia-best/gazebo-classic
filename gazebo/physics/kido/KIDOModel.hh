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

#ifndef _GAZEBO_KIDOMODEL_HH_
#define _GAZEBO_KIDOMODEL_HH_

#include "gazebo/physics/kido/kido_inc.h"
#include "gazebo/physics/kido/KIDOTypes.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class KIDOModelPrivate;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_kido KIDO Physics
    /// \brief kido physics engine wrapper
    /// \{

    /// \class KIDOModel
    /// \brief KIDO model class
    class GZ_PHYSICS_VISIBLE KIDOModel : public Model
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent object.
      public: explicit KIDOModel(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~KIDOModel();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Update();

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief
      public: void BackupState();

      /// \brief
      public: void RestoreState();

      /// \brief
      public: kido::dynamics::Skeleton *GetKIDOSkeleton();

      /// \brief
      public: KIDOPhysicsPtr GetKIDOPhysics(void) const;

      /// \brief
      public: kido::simulation::World *GetKIDOWorld(void) const;

      /// \internal
      /// \brief Pointer to private data
      private: KIDOModelPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
