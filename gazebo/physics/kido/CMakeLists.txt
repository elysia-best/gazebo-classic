include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
    kido/KIDOBallJoint.cc
    kido/KIDOBoxShape.cc
    kido/KIDOCollision.cc
    kido/KIDOCylinderShape.cc
    kido/KIDOFixedJoint.cc
    kido/KIDOHeightmapShape.cc
    kido/KIDOHingeJoint.cc
    kido/KIDOHinge2Joint.cc
    kido/KIDOJoint.cc
    kido/KIDOLink.cc
    kido/KIDOMesh.cc
    kido/KIDOMeshShape.cc
    kido/KIDOModel.cc
    kido/KIDOMultiRayShape.cc
    kido/KIDOPhysics.cc
    kido/KIDOPlaneShape.cc
    kido/KIDOPolylineShape.cc
    kido/KIDORayShape.cc
    kido/KIDOScrewJoint.cc
    kido/KIDOSliderJoint.cc
    kido/KIDOSphereShape.cc
    kido/KIDOSurfaceParams.cc
    kido/KIDOUniversalJoint.cc
    PARENT_SCOPE
)

SET (headers
    KIDOBallJoint.hh
    KIDOBoxShape.hh
    KIDOCollision.hh
    KIDOCylinderShape.hh
    KIDOFixedJoint.hh
    KIDOHeightmapShape.hh
    KIDOHingeJoint.hh
    KIDOHinge2Joint.hh
    KIDOJoint.hh
    KIDOLink.hh
    KIDOMesh.hh
    KIDOMeshShape.hh
    KIDOModel.hh
    KIDOMultiRayShape.hh
    KIDOPhysics.hh
    KIDOPlaneShape.hh
    KIDOPolylineShape.hh
    KIDORayShape.hh
    KIDOScrewJoint.hh
    KIDOSliderJoint.hh
    KIDOSphereShape.hh
    KIDOSurfaceParams.hh
    KIDOTypes.hh
    KIDOUniversalJoint.hh
    kido_inc.h
)

gz_install_includes("physics/kido" ${headers})
