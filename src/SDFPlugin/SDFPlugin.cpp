/*!
  @file
  @author Shizuko Hattori, Shin'ichiro Nakaoka
*/

#include "SDFBodyLoader.h"
#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/ItemFileIO>  
#include <cnoid/BodyLoader>
#include <cnoid/BodyItem>
#include <sdf/sdf.hh>
#include <OGRE/OgreRoot.h>
#include <memory>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
    
AbstractBodyLoaderPtr sdfBodyLoaderFactory()
{
    return make_shared<SDFBodyLoader>();
}

class UrdfFileIO : public ItemFileIOBase<BodyItem>
{
    unique_ptr<SDFBodyLoader> loader;

public:
    UrdfFileIO()
        : ItemFileIOBase<BodyItem>("URDF", Load)
    {
        setCaption(_("ROS / Gazebo model"));
        setFileTypeCaption(_("URDF"));
        setExtensions({ "urdf", "xacro" });
        addFormatIdAlias("GAZEBO-MODEL"); // for the backward compatibility
        setInterfaceLevel(Conversion);
    }

    virtual bool load(BodyItem* item, const std::string& filename) override
    {
        if(!loader){
            loader.reset(new SDFBodyLoader);
            loader->setMessageSink(os());
        }
        return loader->load(item, filename);
    }
};

class SdfFileIO : public ItemFileIOBase<BodyItem>
{
    unique_ptr<SDFBodyLoader> loader;

public:
    SdfFileIO()
        : ItemFileIOBase<BodyItem>("SDF", Load)
    {
        setCaption(_("ROS / Gazebo model"));
        setFileTypeCaption(_("SDF"));
        setExtensions({ "sdf", "xacro" });
        addFormatIdAlias("GAZEBO-MODEL"); // for the backward compatibility
        setInterfaceLevel(Conversion);
    }

    virtual bool load(BodyItem* item, const std::string& filename) override
    {
        if(!loader){
            loader.reset(new SDFBodyLoader);
            loader->setMessageSink(os());
        }
        return loader->load(item, filename);
    }
};


class SDFPlugin : public Plugin
{
    Ogre::Root* ogreRoot;

public:
    SDFPlugin() : Plugin("SDF")
    {
        require("Body");
        require("Assimp");
    }
        
    virtual bool initialize()
    {
        BodyLoader::registerLoader("sdf", sdfBodyLoaderFactory);
        BodyLoader::registerLoader("urdf", sdfBodyLoaderFactory);

        auto& im = itemManager();
        im.registerFileIO<BodyItem>(new UrdfFileIO);
        im.registerFileIO<BodyItem>(new SdfFileIO);

        addModelSearchPath("GAZEBO_MODEL_PATH");
        addModelSearchPath("ROS_PACKAGE_PATH");
        addModelSearchPath("HOME");

        ogreRoot = new Ogre::Root();

        return true;
    }
        
    virtual bool finalize()
    {
        delete ogreRoot;
        return true;
    }

    void addModelSearchPath(const char *envname)
    {
        std::list<std::string> paths;
        std::string path;
        char *p;

        if (envname != NULL && (p = getenv(envname)) != NULL) {
            if (envname != "HOME") {
                boost::split(paths, p, boost::is_any_of(":"));
                for(auto& path : paths){
                    if (path != "") {
                        sdf::addURIPath("model://", path);
                    }
                }
            } else {
                path = p;
                sdf::addURIPath("model://", path + "/.gazebo/models");
            }
        }

        return;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SDFPlugin);
