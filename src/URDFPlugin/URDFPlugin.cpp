#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/URDFBodyLoader>
#include <cnoid/BodyItem>
#include <cnoid/BodyItemFileIO>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyItemUrdfLoader : public ItemFileIoBase<BodyItem>
{
    unique_ptr<URDFBodyLoader> urdfLoader;
    
public:
    BodyItemUrdfLoader()
        : ItemFileIoBase("URDF", Load)
    {
        setCaption(_("Body"));
        setFileTypeCaption("URDF");
        setExtensions({ "urdf", "xacro" });
    }

    URDFBodyLoader* ensureUrdfLoader()
    {
        if(!urdfLoader){
            urdfLoader.reset(new URDFBodyLoader);
            urdfLoader->setMessageSink(os());
        }
        return urdfLoader.get();
    }
    
    virtual bool load(BodyItem* item, const std::string& filename) override
    {
        BodyPtr newBody = new Body;
        if(!ensureUrdfLoader()->load(newBody, filename)){
            return false;
        }
        item->setBody(newBody);
    
        auto itype = invocationType();
        if(itype == Dialog || itype == DragAndDrop){
            item->setChecked(true);
        }
    
        return true;
    }
};


class URDFPlugin : public Plugin
{
public:
    URDFPlugin() : Plugin("URDF")
    {
        require("Body");
    }
        
    virtual bool initialize()
    {
        constexpr bool MAKE_URDF_LOADER_INDEPENDENT = true;

        if(MAKE_URDF_LOADER_INDEPENDENT){
            itemManager().registerFileIO<BodyItem>(new BodyItemUrdfLoader);
        } else {
            if(auto io = dynamic_cast<BodyItemBodyFileIO*>(BodyItem::bodyFileIO())){
                io->addExtensions({ "urdf", "xacro" });
            }
        }

        return true;
    }
        
    virtual bool finalize()
    {
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(URDFPlugin);
