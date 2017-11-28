/**
   \file
   \author Ikumi Susa
*/

#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_CONVEXDECOMPOSITION_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_CONVEXDECOMPOSITION_H

#include <cnoid/Referenced>
#include <agxCollide/ConvexBuilder.h>

namespace cnoid{

class AGXConvexDecomposition : public Referenced {
public:
    AGXConvexDecomposition();
    agxCollide::ConvexBuilder* getConvexBuilder();

private:
    agxCollide::ConvexBuilderRef m_builder;
};
typedef ref_ptr<AGXConvexDecomposition> AGXConvexDecompositionPtr;

}

#endif
