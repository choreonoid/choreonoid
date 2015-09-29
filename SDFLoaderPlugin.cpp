/*!
  @file
  @author 
*/

#include <cnoid/Plugin>
#include <cnoid/BodyLoader>
#include <iostream>
#include <boost/make_shared.hpp>
#include "SDFBodyLoader.h"

using namespace std;

using namespace cnoid;

namespace {
    
AbstractBodyLoaderPtr sdfBodyLoaderFactory()
{
    return boost::make_shared<SDFBodyLoader>();
}

class SDFLoaderPlugin : public Plugin
{
public:
    SDFLoaderPlugin() : Plugin("SDFLoader") {
    }
        
    virtual ~SDFLoaderPlugin() {
    }
    
    virtual bool initialize() {
        BodyLoader::registerLoader("sdf", sdfBodyLoaderFactory);
        BodyLoader::registerLoader("urdf", sdfBodyLoaderFactory);
        return true;
    }
        
    virtual bool finalize() {
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SDFLoaderPlugin);
