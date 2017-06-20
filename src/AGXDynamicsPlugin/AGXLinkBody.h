#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_LINKBODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_LINKBODY_H

#include "AGXInclude.h"

namespace cnoid{
struct AGXRigidBodyDesc
{
	AGXRigidBodyDesc(){}
	agx::RigidBody::MotionControl control = agx::RigidBody::DYNAMICS;
	agx::Vec3 v = agx::Vec3();			// velocity
	agx::Vec3 p = agx::Vec3();			// position(world)
	agx::Real m = 1.0;					// mass
	agx::SPDMatrix3x3 I = agx::SPDMatrix3x3(agx::Vec3(1.0, 1.0, 1.0));	// inertia
	agx::Vec3 c = agx::Vec3();			// center of mass(local)
	agx::MassProperties::AutoGenerateFlags genflags = agx::MassProperties::AutoGenerateFlags::AUTO_GENERATE_ALL;		//
	agx::String name;					// name
};

struct AGXGeometryDesc{
	AGXGeometryDesc(){};
	agx::Vec3f surfacevel;
};

enum AGXShapeType
{
	NONE = 0,
	AGXCOLLIDE_BOX,
	AGXCOLLIDE_SPHERE,
	AGXCOLLIDE_CAPSULE,
	AGXCOLLIDE_CYLINDER,
	AGXCOLLIDE_PLANE,
	AGXCOLLIDE_LINE,
	AGXCOLLIDE_MESH,
	AGXCOLLIDE_HEIGHTFIELD,
	AGXCOLLIDE_CONVEX,
	AGXCOLLIDE_TRIMESH,
	AGXCOLLIDE_TERRAIN
};

struct AGXShapeDesc
{
	AGXShapeDesc(){};
	AGXShapeType shapeType = AGXShapeType::NONE;
};

struct AGXBoxDesc : public AGXShapeDesc
{
	AGXBoxDesc()
	{
		shapeType = AGXShapeType::AGXCOLLIDE_BOX;
	}
	agx::Vec3 halfExtents;
};

struct AGXSphereDesc : public AGXShapeDesc{
	AGXSphereDesc()
	{
		shapeType = AGXShapeType::AGXCOLLIDE_SPHERE;
	}
};

struct AGXTrimeshDesc : public AGXShapeDesc
{
	AGXTrimeshDesc()
	{
		shapeType = AGXShapeType::AGXCOLLIDE_TRIMESH;
	}
	agx::Vec3Vector vertices;
	agx::UIntVector indices;
};

class AGXLinkBody : public agx::Referenced
{
public:
	AGXLinkBody();
	agx::RigidBodyRef getRigidBody();
	agxCollide::GeometryRef getGeometry();
	void createRigidBody(AGXRigidBodyDesc desc);
	void createGeometry(AGXGeometryDesc desc);
	void createShape(const AGXShapeDesc& desc);
private:
	agx::RigidBodyRef _rigid;
	agxCollide::GeometryRef _geometry;
	agxCollide::BoxRef createShapeBox(const AGXBoxDesc& desc);
	agxCollide::MeshRef createShapeTrimesh(const AGXTrimeshDesc & desc);
};
typedef agx::ref_ptr<AGXLinkBody> AGXLinkBodyRef;
}

#endif