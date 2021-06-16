#include "SceneItemFileIO.h"
#include "GeneralSceneFileImporterBase.h"
#include "ItemManager.h"
#include <cnoid/StdSceneWriter>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

SceneItemStdSceneFileExporter* stdSceneFileExporter = nullptr;

class GeneralSceneFileImporter : public GeneralSceneFileImporterBase
{
public:
    GeneralSceneFileImporter()
    {
        setCaption(_("Scene"));
        setFileTypeCaption(_("Scene / Mesh"));

        addFormatAlias("CHOREONOID-SCENE");
        addFormatAlias("AVAILABLE-SCENE-FILE");
        addFormatAlias("VRML-FILE");
        addFormatAlias("STL-FILE");
    }
    
    virtual Item* createItem() override
    {
        return new SceneItem;
    }
    
    virtual bool load(Item* item, const std::string& filename) override
    {
        SgNode* scene = loadScene(filename);
        if(!scene){
            return false;
        }
        auto sceneItem = static_cast<SceneItem*>(item);
        auto topNode = sceneItem->topNode();
        topNode->clearChildren();
        topNode->addChild(scene);
        if(sceneItem->isLightweightRenderingEnabled()){
            sceneItem->setLightweightRenderingEnabled(true);
        }
        return true;
    }
};

}


SceneItemStdSceneFileExporter::SceneItemStdSceneFileExporter()
    : ItemFileIoBase<SceneItem>("CHOREONOID-SCENE", Save)
{
    setCaption(_("Standard scene file"));
    setExtension("scen");
}


StdSceneWriter* SceneItemStdSceneFileExporter::ensureSceneWriter()
{
    if(!sceneWriter_){
        sceneWriter_.reset(new StdSceneWriter);
        sceneWriter_->setMessageSink(os());
    }
    return sceneWriter_.get();
}


bool SceneItemStdSceneFileExporter::save(SceneItem* item, const std::string& filename)
{
    auto topNode = item->topNode();
    if(topNode->empty()){
        os() << _("Empty scene") << endl;
        return false;
    }
    
    SgNodePtr node;
    if(topNode->numChildren() == 1){
        node = topNode->child(0);
    } else {
        auto group = new SgGroup;
        topNode->copyChildrenTo(group);
        node = group;
    }

    return ensureSceneWriter()->writeScene(filename, node);
}


void SceneItem::registerSceneItemFileIoSet(ItemManager* im)
{
    im->addFileIO<SceneItem>(new GeneralSceneFileImporter);

    ::stdSceneFileExporter = new SceneItemStdSceneFileExporter;
    im->addFileIO<SceneItem>(::stdSceneFileExporter);
}


// Defined in SceneItem.h
ItemFileIO* SceneItem::stdSceneFileExporter()
{
    return ::stdSceneFileExporter;
}
