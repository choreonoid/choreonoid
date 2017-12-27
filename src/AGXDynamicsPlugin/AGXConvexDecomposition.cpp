/**
   \file
   \author Ikumi Susa
*/

#include "AGXConvexDecomposition.h"
#include "AGXObjectFactory.h"

namespace cnoid{

AGXConvexDecomposition::AGXConvexDecomposition(){
    m_builder = AGXObjectFactory::createConvexBuilder();
}

agxCollide::ConvexBuilder* AGXConvexDecomposition::getConvexBuilder(){
    return m_builder;
}

}
