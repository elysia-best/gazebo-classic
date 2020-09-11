/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef GAZEBO_REFLECTANCEPLUGIN_HH_
#define GAZEBO_REFLECTANCEPLUGIN_HH_

#include <memory>
#include <string>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  class VisualUtilsPrivate;

  /// \brief This plugin stored the reflectance map in the parent visual Then we
  /// will be able to access this reflectance map from the ogre node to apply
  /// this reflectance to the object using this plugin.
  /// This plugin should have a <reflectance_map> tag. Inside this tag you two
  /// options:
  /// - include <name> with the absolute path of your image
  /// - include <name> with the name of the resource and <uri> could be used:
  ///   - absolute directory
  ///   - model://
  ///
  /// The image could be RGB or black and white. It's recommended to use
  /// .png files.
  class GAZEBO_VISIBLE ReflectancePlugin : public VisualPlugin
  {
    /// \brief Constructor.
    public: ReflectancePlugin();

    /// \brief Destructor.
    public: ~ReflectancePlugin();

    // Documentation inherited
    public: virtual void Load(rendering::VisualPtr _visual,
        sdf::ElementPtr _sdf) override;
  };
}
#endif
