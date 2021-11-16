/**
   \file
   \author Ikumi Susa
*/

#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_CONVEXDECOMPOSITION_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_CONVEXDECOMPOSITION_H

#include <cnoid/Referenced>
#include "AGXInclude.h"

namespace cnoid {

class AGXConvexDecomposition : public Referenced {
public:
    AGXConvexDecomposition();
    
    agxCollide::ConvexFactory* getConvexFactory();

#if AGX_VERSION_GREATER_OR_EQUAL(2, 31, 0, 0)
    size_t build(const agx::Vec3Vector& vertices, const agx::UInt32Vector& indices, unsigned int /* numTriangles */){
        return m_factory->build(vertices, indices);
    }
#else
    size_t build(const agx::Vec3Vector& vertices, const agx::UInt32Vector& indices, unsigned int numTriangles){
        return m_factory->build(vertices, indices, numTriangles);
    }
#endif

private:
    agxCollide::ConvexFactoryRef m_factory;
};

typedef ref_ptr<AGXConvexDecomposition> AGXConvexDecompositionPtr;

}

#endif
