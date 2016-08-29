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

#ifndef _GAZEBO_KIDOLINK_PRIVATE_HH_
#define _GAZEBO_KIDOLINK_PRIVATE_HH_

#include <vector>

#include "gazebo/physics/kido/kido_inc.h"
#include "gazebo/physics/kido/KIDOTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for KIDOLink
    class KIDOLinkPrivate
    {
      /// \brief Constructor
      public: KIDOLinkPrivate()
        : kidoPhysics(NULL),
          dtBodyNode(NULL),
          kidoParentJoint(NULL),
          kidoChildJoints {},
          staticLink(false),
          dtWeldJointConst(NULL)
      {
      }

      /// \brief Default destructor
      public: ~KIDOLinkPrivate()
      {
        // We don't need to delete dtBodyNode because skeleton will delete
        // dtBodyNode if it is registered to the skeleton.

        delete dtWeldJointConst;
      }

      /// \brief Pointer to the KIDO physics engine.
      public: KIDOPhysicsPtr kidoPhysics;

      /// \brief Pointer to the KIDO BodyNode.
      public: kido::dynamics::BodyNode *dtBodyNode;

      /// \brief Pointer to the parent joint.
      public: KIDOJointPtr kidoParentJoint;

      /// \brief List of pointers to the child joints.
      public: std::vector<KIDOJointPtr> kidoChildJoints;

      /// \brief If true, freeze link to world (inertial) frame.
      public: bool staticLink;

      /// \brief Weld joint constraint for SetLinkStatic()
      public: kido::constraint::WeldJointConstraint *dtWeldJointConst;
    };
  }
}
#endif
