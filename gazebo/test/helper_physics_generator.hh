/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _HELPER_PHYSICS_GENERATOR_HH_
#define _HELPER_PHYSICS_GENERATOR_HH_

#include "gazebo/gazebo_config.h"

#define BULLET_SUPPORT
#define WORLD_STEP_BULLET_LEMKE
#define WORLD_STEP_BULLET_PGS

#ifdef HAVE_BULLET
# undef BULLET_SUPPORT
# define BULLET_SUPPORT , "bullet"
# undef WORLD_STEP_BULLET_PGS
# define WORLD_STEP_BULLET_PGS , "BULLET_PGS"
# ifdef LIBBULLET_VERSION_GT_282
#  undef WORLD_STEP_BULLET_LEMKE
#  define WORLD_STEP_BULLET_LEMKE , "BULLET_LEMKE"
# endif
#endif

#define SIMBODY_SUPPORT
#define KIDO_SUPPORT
#define WORLD_STEP_KIDO_PGS

#ifdef HAVE_SIMBODY
# undef SIMBODY_SUPPORT
# define SIMBODY_SUPPORT , "simbody"
#endif
#ifdef HAVE_KIDO
# undef KIDO_SUPPORT
# define KIDO_SUPPORT , "kido"
# undef WORLD_STEP_KIDO_PGS
# define WORLD_STEP_KIDO_PGS , "KIDO_PGS"
#endif

/// \brief Helper macro to instantiate gtest for different physics engines
#define PHYSICS_ENGINE_VALUES ::testing::Values("ode" \
  BULLET_SUPPORT \
  SIMBODY_SUPPORT \
  KIDO_SUPPORT \
  )

/// \brief Helper macro to instantiate gtest for different solvers
#define WORLD_STEP_SOLVERS ::testing::Values("ODE_DANTZIG" \
  WORLD_STEP_KIDO_PGS \
  WORLD_STEP_BULLET_PGS \
  WORLD_STEP_BULLET_LEMKE \
  )

#endif
