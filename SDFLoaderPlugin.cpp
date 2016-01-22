/*!
  @file
  @author 
*/

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/BodyLoader>
#include <cnoid/BodyItem>
#include <iostream>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
#include "SDFBodyLoader.h"

using namespace std;

using namespace cnoid;

namespace {
    
bool loadBodyItem(BodyItem* item, const std::string& filename)
{
    if(item->loadModelFile(filename)){
        if(item->name().empty()){
            item->setName(item->body()->modelName());
        }
        item->setEditable(!item->body()->isStaticModel());
        return true;
    }
    return false;
}
    
AbstractBodyLoaderPtr sdfBodyLoaderFactory()
{
    return boost::make_shared<SDFBodyLoader>();
}

class SDFLoaderPlugin : public Plugin
{
public:
    SDFLoaderPlugin() : Plugin("SDFLoader") {
        require("Body");
    }
        
    virtual ~SDFLoaderPlugin() {
    }
    
    virtual bool initialize() {
        BodyLoader::registerLoader("sdf", sdfBodyLoaderFactory);
        BodyLoader::registerLoader("urdf", sdfBodyLoaderFactory);
        itemManager().addLoader<BodyItem>(
            "Gazebo Model File", "GAZEBO-MODEL", "sdf;urdf", boost::bind(loadBodyItem, _1, _2));
        return true;
    }
        
    virtual bool finalize() {
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SDFLoaderPlugin);
