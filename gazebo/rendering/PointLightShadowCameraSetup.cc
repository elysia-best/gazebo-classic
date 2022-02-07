/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

// Code in this file has been adapted from Ogre's OgreShadowCameraSetup.
// The original Ogre's licence and copyright headers are copied below:

/*
-----------------------------------------------------------------------------
This source file is part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2014 Torus Knot Software Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#include "gazebo/rendering/PointLightShadowCameraSetup.hh"
#include "gazebo/rendering/ogre_gazebo.h"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
PointLightShadowCameraSetup::PointLightShadowCameraSetup()
{
}

//////////////////////////////////////////////////
PointLightShadowCameraSetup::~PointLightShadowCameraSetup()
{
}

//////////////////////////////////////////////////
void PointLightShadowCameraSetup::getShadowCamera(
    const Ogre::SceneManager */*_sm*/, const Ogre::Camera *_cam,
    const Ogre::Viewport */*_vp*/, const Ogre::Light *_light,
    Ogre::Camera *_texCam, size_t _iteration) const
{
  Ogre::Vector3 pos, dir;

  // reset custom view / projection matrix in case already set
  _texCam->setCustomViewMatrix(false);
  _texCam->setCustomProjectionMatrix(false);
  _texCam->setNearClipDistance(_light->_deriveShadowNearClipDistance(_cam));
  _texCam->setFarClipDistance(_light->_deriveShadowFarClipDistance(_cam));

  _texCam->setProjectionType(Ogre::PT_PERSPECTIVE);

  // theoretically set x to +-0.25 for accuracy
  // decrease x for quality of shadows
  float x = 0.18f;
  _texCam->setFrustumExtents(-x, x, x, -x);

  // set shadow cube map depending on the iteration
  switch (_iteration) {
    case 0:
      _texCam->lookAt(_texCam->getPosition() + Ogre::Vector3(1, 0, 0));
      break;
    case 1:
      _texCam->lookAt(_texCam->getPosition() + Ogre::Vector3(-1, 0, 0));
      break;
    case 2:
      _texCam->lookAt(_texCam->getPosition() + Ogre::Vector3(0, 1, 0));
      break;
    case 3:
      _texCam->lookAt(_texCam->getPosition() + Ogre::Vector3(0, -1, 0));
      break;
    case 4:
      _texCam->lookAt(_texCam->getPosition() + Ogre::Vector3(0, 0, 1));
      break;
    case 5:
      _texCam->lookAt(_texCam->getPosition() + Ogre::Vector3(0, 0, -1));
      break;
    default:
      break;
  }
}
