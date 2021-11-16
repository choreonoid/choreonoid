/**
   \file
   \author Ikumi Susa
*/

#include "AGXConvexDecomposition.h"
#include "AGXObjectFactory.h"

namespace cnoid{

AGXConvexDecomposition::AGXConvexDecomposition(){
    m_factory = AGXObjectFactory::createConvexFactory();
}

agxCollide::ConvexFactory* AGXConvexDecomposition::getConvexFactory(){
    return m_factory;
}

}
