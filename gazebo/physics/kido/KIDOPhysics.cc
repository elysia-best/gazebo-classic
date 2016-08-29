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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/Vector3.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/MapShape.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/physics/kido/KIDOScrewJoint.hh"
#include "gazebo/physics/kido/KIDOHingeJoint.hh"
#include "gazebo/physics/kido/KIDOHinge2Joint.hh"
#include "gazebo/physics/kido/KIDOSliderJoint.hh"
#include "gazebo/physics/kido/KIDOBallJoint.hh"
#include "gazebo/physics/kido/KIDOUniversalJoint.hh"
#include "gazebo/physics/kido/KIDOFixedJoint.hh"

#include "gazebo/physics/kido/KIDORayShape.hh"
#include "gazebo/physics/kido/KIDOBoxShape.hh"
#include "gazebo/physics/kido/KIDOSphereShape.hh"
#include "gazebo/physics/kido/KIDOCylinderShape.hh"
#include "gazebo/physics/kido/KIDOPlaneShape.hh"
#include "gazebo/physics/kido/KIDOMeshShape.hh"
#include "gazebo/physics/kido/KIDOPolylineShape.hh"
#include "gazebo/physics/kido/KIDOMultiRayShape.hh"
#include "gazebo/physics/kido/KIDOHeightmapShape.hh"

#include "gazebo/physics/kido/KIDOModel.hh"
#include "gazebo/physics/kido/KIDOLink.hh"

#include "gazebo/physics/kido/KIDOPhysics.hh"

#include "gazebo/physics/kido/KIDOPhysicsPrivate.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("kido", KIDOPhysics)

//////////////////////////////////////////////////
KIDOPhysics::KIDOPhysics(WorldPtr _world)
    : PhysicsEngine(_world), dataPtr(new KIDOPhysicsPrivate())
{
}

//////////////////////////////////////////////////
KIDOPhysics::~KIDOPhysics()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void KIDOPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  // Gravity
  auto g = this->world->Gravity();
  // ODEPhysics checks this, so we will too.
  if (g == ignition::math::Vector3d::Zero)
    gzwarn << "Gravity vector is (0, 0, 0). Objects will float.\n";
  this->dataPtr->dtWorld->setGravity(Eigen::Vector3d(g.X(), g.Y(), g.Z()));

  // Time step
  // double timeStep = this->sdf->GetValueDouble("time_step");
  // this->kidoWorld->setTimeStep(timeStep);

  // TODO: Elements for kido settings
  // sdf::ElementPtr kidoElem = this->sdf->GetElement("kido");
  // this->stepTimeDouble = kidoElem->GetElement("dt")->GetValueDouble();
}

//////////////////////////////////////////////////
void KIDOPhysics::Init()
{
  // this->kidoWorld->initialize();
}

//////////////////////////////////////////////////
void KIDOPhysics::Fini()
{
  PhysicsEngine::Fini();
}

//////////////////////////////////////////////////
void KIDOPhysics::Reset()
{
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  // Restore state all the models
  unsigned int modelCount = this->world->GetModelCount();
  KIDOModelPtr kidoModelIt;

  for (unsigned int i = 0; i < modelCount; ++i)
  {
    kidoModelIt =
      boost::dynamic_pointer_cast<KIDOModel>(this->world->GetModel(i));
    GZ_ASSERT(kidoModelIt.get(), "kidoModelIt pointer is NULL");

    kidoModelIt->RestoreState();
  }
}

//////////////////////////////////////////////////
void KIDOPhysics::InitForThread()
{
}

//////////////////////////////////////////////////
void KIDOPhysics::UpdateCollision()
{
  this->contactManager->ResetCount();

  kido::constraint::ConstraintSolver *dtConstraintSolver =
      this->dataPtr->dtWorld->getConstraintSolver();
  kido::collision::CollisionDetector *dtCollisionDetector =
      dtConstraintSolver->getCollisionDetector();
  int numContacts = dtCollisionDetector->getNumContacts();

  for (int i = 0; i < numContacts; ++i)
  {
    const kido::collision::Contact &dtContact =
        dtCollisionDetector->getContact(i);
    kido::dynamics::BodyNode *dtBodyNode1 = dtContact.bodyNode1;
    kido::dynamics::BodyNode *dtBodyNode2 = dtContact.bodyNode2;

    KIDOLinkPtr kidoLink1 = this->FindKIDOLink(dtBodyNode1);
    KIDOLinkPtr kidoLink2 = this->FindKIDOLink(dtBodyNode2);

    GZ_ASSERT(kidoLink1.get() != NULL, "kidoLink1 in collision pare is NULL");
    GZ_ASSERT(kidoLink2.get() != NULL, "kidoLink2 in collision pare is NULL");

    unsigned int colIndex = 0;
    CollisionPtr collisionPtr1 = kidoLink1->GetCollision(colIndex);
    CollisionPtr collisionPtr2 = kidoLink2->GetCollision(colIndex);

    // Add a new contact to the manager. This will return NULL if no one is
    // listening for contact information.
    Contact *contactFeedback = this->GetContactManager()->NewContact(
                                 collisionPtr1.get(), collisionPtr2.get(),
                                 this->world->GetSimTime());

    if (!contactFeedback)
      continue;

    math::Pose body1Pose = kidoLink1->GetWorldPose();
    math::Pose body2Pose = kidoLink2->GetWorldPose();
    math::Vector3 localForce1;
    math::Vector3 localForce2;
    math::Vector3 localTorque1;
    math::Vector3 localTorque2;

    // calculate force in world frame
    Eigen::Vector3d force = dtContact.force;

    // calculate torque in world frame
    Eigen::Vector3d torqueA =
        (dtContact.point -
         dtBodyNode1->getTransform().translation()).cross(force);
    Eigen::Vector3d torqueB =
        (dtContact.point -
         dtBodyNode2->getTransform().translation()).cross(-force);

    // Convert from world to link frame
    localForce1 = body1Pose.rot.RotateVectorReverse(
        KIDOTypes::ConvVec3(force));
    localForce2 = body2Pose.rot.RotateVectorReverse(
        KIDOTypes::ConvVec3(-force));
    localTorque1 = body1Pose.rot.RotateVectorReverse(
        KIDOTypes::ConvVec3(torqueA));
    localTorque2 = body2Pose.rot.RotateVectorReverse(
        KIDOTypes::ConvVec3(torqueB));

    contactFeedback->positions[0] = KIDOTypes::ConvVec3(dtContact.point);
    contactFeedback->normals[0] = KIDOTypes::ConvVec3(dtContact.normal);
    contactFeedback->depths[0] = dtContact.penetrationDepth;

    if (!kidoLink1->IsStatic())
    {
      contactFeedback->wrench[0].body1Force = localForce1;
      contactFeedback->wrench[0].body1Torque = localTorque1;
    }

    if (!kidoLink2->IsStatic())
    {
      contactFeedback->wrench[0].body2Force = localForce2;
      contactFeedback->wrench[0].body2Torque = localTorque2;
    }

    ++contactFeedback->count;
  }
}

//////////////////////////////////////////////////
void KIDOPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  // common::Time currTime =  this->world->GetRealTime();

  this->dataPtr->dtWorld->setTimeStep(this->maxStepSize);
  this->dataPtr->dtWorld->step();

  // Update all the transformation of KIDO's links to gazebo's links
  // TODO: How to visit all the links in the world?
  unsigned int modelCount = this->world->GetModelCount();
  ModelPtr modelItr;

  for (unsigned int i = 0; i < modelCount; ++i)
  {
    modelItr = this->world->GetModel(i);
    // TODO: need to improve speed
    Link_V links = modelItr->GetLinks();
    unsigned int linkCount = links.size();
    KIDOLinkPtr kidoLinkItr;

    for (unsigned int j = 0; j < linkCount; ++j)
    {
      kidoLinkItr
          = boost::dynamic_pointer_cast<KIDOLink>(links.at(j));
      kidoLinkItr->updateDirtyPoseFromKIDOTransformation();
    }
  }

  // this->lastUpdateTime = currTime;
}

//////////////////////////////////////////////////
std::string KIDOPhysics::GetType() const
{
  return "kido";
}

//////////////////////////////////////////////////
void KIDOPhysics::SetSeed(uint32_t /*_seed*/)
{
  gzwarn << "Not implemented yet in KIDO.\n";
}

//////////////////////////////////////////////////
ModelPtr KIDOPhysics::CreateModel(BasePtr _parent)
{
  KIDOModelPtr model(new KIDOModel(_parent));

  return model;
}

//////////////////////////////////////////////////
LinkPtr KIDOPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == NULL)
  {
    gzerr << "Link must have a parent in KIDO.\n";
    return LinkPtr();
  }

  KIDOLinkPtr link(new KIDOLink(_parent));
  link->SetWorld(_parent->GetWorld());

  return link;
}

//////////////////////////////////////////////////
CollisionPtr KIDOPhysics::CreateCollision(const std::string &_type,
                                          LinkPtr _body)
{
  KIDOCollisionPtr collision(new KIDOCollision(_body));
  ShapePtr shape = this->CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_body->GetWorld());
  return collision;
}

//////////////////////////////////////////////////
ShapePtr KIDOPhysics::CreateShape(const std::string &_type,
                                    CollisionPtr _collision)
{
  ShapePtr shape;
  KIDOCollisionPtr collision =
    boost::dynamic_pointer_cast<KIDOCollision>(_collision);

  if (_type == "sphere")
    shape.reset(new KIDOSphereShape(collision));
  else if (_type == "plane")
    shape.reset(new KIDOPlaneShape(collision));
  else if (_type == "box")
    shape.reset(new KIDOBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new KIDOCylinderShape(collision));
  else if (_type == "multiray")
    shape.reset(new KIDOMultiRayShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new KIDOMeshShape(collision));
  else if (_type == "polyline")
    shape.reset(new KIDOPolylineShape(collision));
  else if (_type == "heightmap")
    shape.reset(new KIDOHeightmapShape(collision));
  else if (_type == "map" || _type == "image")
    shape.reset(new MapShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new KIDORayShape(collision));
    else
      shape.reset(new KIDORayShape(this->world->GetPhysicsEngine()));
  else
    gzerr << "Unable to create collision of type[" << _type << "]\n";

  return shape;
}

//////////////////////////////////////////////////
JointPtr KIDOPhysics::CreateJoint(const std::string &_type, ModelPtr _parent)
{
  JointPtr joint;

  if (_type == "prismatic")
    joint.reset(new KIDOSliderJoint(_parent));
  else if (_type == "screw")
    joint.reset(new KIDOScrewJoint(_parent));
  else if (_type == "revolute")
    joint.reset(new KIDOHingeJoint(_parent));
  else if (_type == "revolute2")
    joint.reset(new KIDOHinge2Joint(_parent));
  else if (_type == "ball")
    joint.reset(new KIDOBallJoint(_parent));
  else if (_type == "universal")
    joint.reset(new KIDOUniversalJoint(_parent));
  else if (_type == "fixed")
    joint.reset(new KIDOFixedJoint(_parent));
  else
    gzerr << "Unable to create joint of type[" << _type << "]";

  return joint;
}

//////////////////////////////////////////////////
void KIDOPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  this->world->SetGravitySDF(_gravity.Ign());
  this->dataPtr->dtWorld->setGravity(
    Eigen::Vector3d(_gravity.x, _gravity.y, _gravity.z));
}

//////////////////////////////////////////////////
void KIDOPhysics::DebugPrint() const
{
  gzwarn << "Not implemented in KIDO.\n";
}

//////////////////////////////////////////////////
boost::any KIDOPhysics::GetParam(const std::string &_key) const
{
  boost::any value;
  this->GetParam(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool KIDOPhysics::GetParam(const std::string &_key, boost::any &_value) const
{
  if (!this->sdf->HasElement("kido"))
  {
    return PhysicsEngine::GetParam(_key, _value);
  }
  sdf::ElementPtr kidoElem = this->sdf->GetElement("kido");
  // physics kido element not yet added to sdformat
  // GZ_ASSERT(kidoElem != NULL, "KIDO SDF element does not exist");

  if (_key == "max_contacts")
  {
    _value = kidoElem->GetElement("max_contacts")->Get<int>();
  }
  else if (_key == "min_step_size")
  {
    _value = kidoElem->GetElement("solver")->Get<double>("min_step_size");
  }
  else
  {
    return PhysicsEngine::GetParam(_key, _value);
  }

  return true;
}

//////////////////////////////////////////////////
bool KIDOPhysics::SetParam(const std::string &_key, const boost::any &_value)
{
  /// \TODO fill this out, see issue #1115
  try
  {
    if (_key == "max_contacts")
    {
      int value = boost::any_cast<int>(_value);
      gzerr << "Setting [" << _key << "] in KIDO to [" << value
            << "] not yet supported.\n";
    }
    else if (_key == "min_step_size")
    {
      double value = boost::any_cast<double>(_value);
      gzerr << "Setting [" << _key << "] in KIDO to [" << value
            << "] not yet supported.\n";
    }
    else
    {
      if (_key == "max_step_size")
      {
        this->dataPtr->dtWorld->setTimeStep(boost::any_cast<double>(_value));
      }
      return PhysicsEngine::SetParam(_key, _value);
    }
  }
  catch(boost::bad_any_cast &e)
  {
    gzerr << "KIDOPhysics::SetParam(" << _key << ") boost::any_cast error: "
          << e.what() << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
kido::simulation::World *KIDOPhysics::GetKIDOWorld()
{
  return this->dataPtr->dtWorld;
}

//////////////////////////////////////////////////
void KIDOPhysics::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info")
  {
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::KIDO);
    physicsMsg.mutable_gravity()->CopyFrom(
      msgs::Convert(this->world->Gravity()));
    physicsMsg.mutable_magnetic_field()->CopyFrom(
      msgs::Convert(this->MagneticField()));
    physicsMsg.set_enable_physics(this->world->GetEnablePhysicsEngine());
    physicsMsg.set_real_time_update_rate(this->realTimeUpdateRate);
    physicsMsg.set_real_time_factor(this->targetRealTimeFactor);
    physicsMsg.set_max_step_size(this->maxStepSize);

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
  }
}

//////////////////////////////////////////////////
void KIDOPhysics::OnPhysicsMsg(ConstPhysicsPtr& _msg)
{
  // Parent class handles many generic parameters
  // This should be done first so that the profile settings
  // can be over-ridden by other message parameters.
  PhysicsEngine::OnPhysicsMsg(_msg);

  if (_msg->has_enable_physics())
    this->world->EnablePhysicsEngine(_msg->enable_physics());

  if (_msg->has_gravity())
    this->SetGravity(msgs::ConvertIgn(_msg->gravity()));

  if (_msg->has_real_time_factor())
    this->SetTargetRealTimeFactor(_msg->real_time_factor());

  if (_msg->has_real_time_update_rate())
  {
    this->SetRealTimeUpdateRate(_msg->real_time_update_rate());
  }

  if (_msg->has_max_step_size())
  {
    this->SetMaxStepSize(_msg->max_step_size());
  }

  /// Make sure all models get at least on update cycle.
  this->world->EnableAllModels();
}

//////////////////////////////////////////////////
KIDOLinkPtr KIDOPhysics::FindKIDOLink(
    const kido::dynamics::BodyNode *_dtBodyNode)
{
  KIDOLinkPtr res;

  const Model_V& models = this->world->GetModels();

  for (Model_V::const_iterator itModel = models.begin();
       itModel != models.end(); ++itModel)
  {
    const Link_V& links = (*itModel)->GetLinks();

    for (Link_V::const_iterator itLink = links.begin();
         itLink != links.end(); ++itLink)
    {
      KIDOLinkPtr kidoLink = boost::dynamic_pointer_cast<KIDOLink>(*itLink);

      if (kidoLink->GetKIDOBodyNode() == _dtBodyNode)
      {
        res = kidoLink;
        break;
      }
    }
  }

  return res;
}

