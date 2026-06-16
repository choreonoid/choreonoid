#include <cnoid/BodyItem>
#include <cnoid/BodyItemFileIO>
#include <cnoid/ItemManager>
#include <cnoid/Plugin>
#include <cnoid/SDFBodyLoader>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyItemSdfLoader : public ItemFileIoBase<BodyItem>
{
    unique_ptr<SDFBodyLoader> sdfLoader;

public:
    BodyItemSdfLoader()
        : ItemFileIoBase("SDF", Load)
    {
        setCaption(_("Body"));
        setFileTypeCaption("SDF");
        setExtensionsForLoading({"sdf", "world"});
    }

    SDFBodyLoader* ensureSdfLoader()
    {
        if (!sdfLoader) {
            sdfLoader.reset(new SDFBodyLoader);
            sdfLoader->setMessageSink(os());
        }
        return sdfLoader.get();
    }

    virtual bool load(BodyItem* item, const std::string& filename) override
    {
        BodyPtr newBody = new Body;
        if (!ensureSdfLoader()->load(newBody, filename)) {
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


class SDFPlugin : public Plugin
{
public:
    SDFPlugin()
        : Plugin("SDF")
    {
        require("Body");
    }

    virtual bool initialize()
    {
        itemManager().addFileIO<BodyItem>(new BodyItemSdfLoader);
        return true;
    }

    virtual bool finalize()
    {
        return true;
    }
};

}  // namespace

CNOID_IMPLEMENT_PLUGIN_ENTRY(SDFPlugin);
