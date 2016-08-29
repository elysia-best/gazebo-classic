/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_KIDOJOINT_PRIVATE_HH_
#define _GAZEBO_KIDOJOINT_PRIVATE_HH_

#include "gazebo/common/Time.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/kido/kido_inc.h"
#include "gazebo/physics/kido/KIDOTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for KIDOJoint
    class KIDOJointPrivate
    {
      /// \brief Constructor
      public: KIDOJointPrivate(const KIDOPhysicsPtr &_kidoPhysicsEngine)
        : forceApplied {0.0, 0.0},
          forceAppliedTime(),
          kidoPhysicsEngine(_kidoPhysicsEngine),
          dtJoint(NULL),
          dtChildBodyNode(NULL)
      {
      }

      /// \brief Default destructor
      public: ~KIDOJointPrivate() = default;

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by KIDO to store external force applied by the user.
      public: double forceApplied[MAX_JOINT_AXIS];

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      public: common::Time forceAppliedTime;

      /// \brief KIDOPhysics engine pointer
      public: KIDOPhysicsPtr kidoPhysicsEngine;

      /// \brief KIDO joint pointer
      public: kido::dynamics::Joint *dtJoint;

      /// \brief KIDO child body node pointer
      public: kido::dynamics::BodyNode *dtChildBodyNode;
    };
  }
}
#endif
