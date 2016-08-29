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

#ifndef _GAZEBO_KIDOLINK_HH_
#define _GAZEBO_KIDOLINK_HH_

#include <vector>

#include "gazebo/physics/Link.hh"

#include "gazebo/physics/kido/kido_inc.h"
#include "gazebo/physics/kido/KIDOTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class KIDOLinkPrivate;

    /// \addtogroup gazebo_physics_kido
    /// \{

    /// \brief KIDO Link class
    class GZ_PHYSICS_VISIBLE KIDOLink : public Link
    {
      /// \brief Constructor
      public: explicit KIDOLink(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~KIDOLink();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _ptr);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Fini();

      // Documentation inherited
      public: virtual void OnPoseChange();

      // Documentation inherited
      public: virtual void SetEnabled(bool _enable) const;

      // Documentation inherited
      public: virtual bool GetEnabled() const;

      // Documentation inherited
      public: virtual void SetLinearVel(const math::Vector3 &_vel);

      // Documentation inherited
      public: virtual void SetAngularVel(const math::Vector3 &_vel);

      // Documentation inherited
      public: virtual void SetForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void SetTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual void AddForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void AddRelativeForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                                                   const math::Vector3 &_pos);

      // Documentation inherited
      public: virtual void AddForceAtRelativePosition(
          const math::Vector3 &_force,
          const math::Vector3 &_relpos);

      // Documentation inherited
      public: virtual void AddLinkForce(const math::Vector3 &_force,
          const math::Vector3 &_offset = math::Vector3::Zero);

      // Documentation inherited
      public: virtual void AddTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual void AddRelativeTorque(const math::Vector3& _torque);

      // Documentation inherited
      public: virtual math::Vector3 GetWorldLinearVel(
          const math::Vector3& _offset = math::Vector3(0, 0, 0)) const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldLinearVel(
          const math::Vector3 &_offset,
          const math::Quaternion &_q) const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldCoGLinearVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldAngularVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldForce() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldTorque() const;

      // Documentation inherited
      public: virtual void SetGravityMode(bool _mode);

      // Documentation inherited
      public: virtual bool GetGravityMode() const;

      // Documentation inherited
      public: virtual void SetSelfCollide(bool _collide);

      // Documentation inherited
      public: virtual void SetLinearDamping(double _damping);

      // Documentation inherited
      public: virtual void SetAngularDamping(double _damping);

      // Documentation inherited
      public: virtual void SetKinematic(const bool &_state);

      // Documentation inherited
      public: virtual bool GetKinematic() const;

      // Documentation inherited
      public: virtual void SetAutoDisable(bool _disable);

      // Documentation inherited
      public: virtual void SetLinkStatic(bool _static);

      /// \brief Store KIDO Transformation to Entity::dirtyPose and add this
      ///        link to World::dirtyPoses so that World::Update() trigger
      ///        Entity::SetWorldPose() for this link.
      public: void updateDirtyPoseFromKIDOTransformation();

      /// \brief Get pointer to KIDO Physics engine associated with this link.
      /// \return Pointer to the KIDO Physics engine.
      public: KIDOPhysicsPtr GetKIDOPhysics(void) const;

      /// \brief Get pointer to KIDO World associated with this link.
      /// \return Pointer to the KIDO World.
      public: kido::simulation::World *GetKIDOWorld(void) const;

      /// \brief Get pointer to KIDO Model associated with this link.
      /// \return Pointer to the KIDO Model.
      public: KIDOModelPtr GetKIDOModel() const;

      /// \brief Get pointer to KIDO BodyNode associated with this link.
      /// \return Pointer to KIDO BodyNode.
      public: kido::dynamics::BodyNode *GetKIDOBodyNode() const;

      /// \brief Set parent joint of this link.
      /// \param[in] _kidoParentJoint Pointer to the parent joint.
      public: void SetKIDOParentJoint(KIDOJointPtr _kidoParentJoint);

      /// \brief Set child joint of this link.
      /// \param[in] _kidoChildJoint Pointer to the child joint.
      public: void AddKIDOChildJoint(KIDOJointPtr _kidoChildJoint);

      // Documentation inherited.
      public: virtual void UpdateMass();

      /// \internal
      /// \brief Pointer to private data
      private: KIDOLinkPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
