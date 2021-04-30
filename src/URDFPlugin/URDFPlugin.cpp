#include "gettext.h"

#include <cnoid/BodyItem>
#include <cnoid/BodyItemFileIO>
#include <cnoid/ItemManager>
#include <cnoid/Plugin>
#include <cnoid/URDFBodyLoader>

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
        setExtensions({"urdf", "xacro"});
    }

    URDFBodyLoader* ensureUrdfLoader()
    {
        if (!urdfLoader) {
            urdfLoader.reset(new URDFBodyLoader);
            urdfLoader->setMessageSink(os());
        }
        return urdfLoader.get();
    }

    virtual bool load(BodyItem* item, const std::string& filename) override
    {
        BodyPtr newBody = new Body;
        if (!ensureUrdfLoader()->load(newBody, filename)) {
            return false;
        }
        item->setBody(newBody);

        auto itype = currentInvocationType();
        if (itype == Dialog || itype == DragAndDrop) {
            item->setChecked(true);
        }

        return true;
    }
};


class URDFPlugin : public Plugin
{
public:
    URDFPlugin()
        : Plugin("URDF")
    {
        require("Body");
    }

    virtual bool initialize()
    {
        constexpr bool MAKE_URDF_LOADER_INDEPENDENT = true;

        if (MAKE_URDF_LOADER_INDEPENDENT) {
            itemManager().addFileIO<BodyItem>(new BodyItemUrdfLoader);
        } else {
            if (auto io = dynamic_cast<BodyItemBodyFileIO*>(
                    BodyItem::bodyFileIO())) {
                io->addFormatAlias({"urdf", "xacro"});
            }
        }

        return true;
    }

    virtual bool finalize() { return true; }
};

}  // namespace

CNOID_IMPLEMENT_PLUGIN_ENTRY(URDFPlugin);
