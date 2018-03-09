#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_INCLUDE_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_INCLUDE_H

// Runtime
#include <agx/Runtime.h>

// Simulation
#include <agxSDK/Simulation.h>
#include <agxSDK/Assembly.h>
#include <agxSDK/MergeSplitHandler.h>

// Threads
#include <agx/Thread.h>

// IO
#include <agxIO/ReaderWriter.h>

// Collide
#include <agxCollide/Box.h>
#include <agxCollide/Sphere.h>
#include <agxCollide/Capsule.h>
#include <agxCollide/Cylinder.h>
#include <agxCollide/Plane.h>
#include <agxCollide/Line.h>
#include <agxCollide/Mesh.h>
#include <agxCollide/HeightField.h>
#include <agxCollide/Convex.h>
#include <agxCollide/Trimesh.h>

// Constraint
#include <agx/Frame.h>
#include <agx/Hinge.h>
#include <agx/Prismatic.h>
#include <agx/LockJoint.h>
#include <agx/BallJoint.h>
#include <agx/PlaneJoint.h>
#include <agx/VirtualConstraintInertia.h>

// Friction models
#include <agx/OrientedFrictionModels.h>

// agxVehicle
#include <agxVehicle/Track.h>

// agxWire
#include <agxWire/Wire.h>
#include <agxWire/Link.h>
#include <agxWire/WireController.h>

// etc
#include <agxCollide/ConvexBuilder.h>

#endif