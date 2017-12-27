/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneItem.h"
#include "ItemManager.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include <cnoid/SceneLoader>
#include <cnoid/FileUtil>
#include <cnoid/EigenArchive>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

bool loadScene(SceneItem* item, const std::string& filename, std::ostream& os)
{
    static SceneLoader* loader = 0;
    if(!loader){
        loader = new SceneLoader;
        loader->setMessageSink(mvout(true));
    }
    auto scene = loader->load(filename);
    if(scene){
        auto invariant = new SgInvariantGroup;
        invariant->addChild(scene);
        auto node = item->topNode();
        node->clearChildren();
        node->addChild(invariant);
        return true;
    }
    return false;
}

}


void SceneItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<SceneItem>(N_("SceneItem"));

        ext->itemManager().addLoader<SceneItem>(
            "Scene", "AVAILABLE-SCENE-FILE", SceneLoader::availableFileExtensions,
            std::bind(::loadScene, _1, _2, _3), ItemManager::PRIORITY_CONVERSION);

        ext->itemManager().addLoader<SceneItem>(
            "VRML", "VRML-FILE", "wrl",
            std::bind(::loadScene, _1, _2, _3), ItemManager::PRIORITY_COMPATIBILITY);

        ext->itemManager().addLoader<SceneItem>(
            "Stereolithography (STL)", "STL-FILE", "stl",
            std::bind(::loadScene, _1, _2, _3), ItemManager::PRIORITY_COMPATIBILITY);

        initialized = true;
    }
}


SceneItem::SceneItem()
    : topNode_(new SgPosTransform())
{

}


SceneItem::SceneItem(const SceneItem& org)
    : Item(org)
{
    // shallow copy
    topNode_ = new SgPosTransform(*org.topNode());
}


SceneItem::~SceneItem()
{
    
}


void SceneItem::setName(const std::string& name)
{
    topNode_->setName(name);
    Item::setName(name);
}


SgNode* SceneItem::getScene()
{
    return topNode_;
}


Item* SceneItem::doDuplicate() const
{
    return new SceneItem(*this);
}


void SceneItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("File"), getFilename(filePath()));
    putProperty(_("Translation"), str(Vector3(topNode_->translation())),
                std::bind(&SceneItem::onTranslationChanged, this, _1));
    Vector3 rpy(TO_DEGREE * rpyFromRot(topNode_->rotation()));
    putProperty("RPY", str(rpy), std::bind(&SceneItem::onRotationChanged, this, _1));
}


bool SceneItem::onTranslationChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        topNode_->setTranslation(p);
        topNode_->notifyUpdate();
        return true;
    }
    return false;
}


bool SceneItem::onRotationChanged(const std::string& value)
{
    Vector3 rpy;
    if(toVector3(value, rpy)){
        topNode_->setRotation(rotFromRpy(TO_RADIAN * rpy));
        topNode_->notifyUpdate();
        return true;
    }
    return false;
}


bool SceneItem::store(Archive& archive)
{
    if(!filePath().empty()){
        archive.writeRelocatablePath("file", filePath());
        archive.write("format", fileFormat());
        write(archive, "translation", topNode_->translation());
        write(archive, "rotation", AngleAxis(topNode_->rotation()));
    }
    return true;
}


bool SceneItem::restore(const Archive& archive)
{
    std::string filename, formatId;
    if(archive.readRelocatablePath("file", filename) && archive.read("format", formatId)){
        Vector3 translation;
        if(read(archive, "translation", translation)){
            topNode_->setTranslation(translation);
        }
        AngleAxis rot;
        if(read(archive, "rotation", rot)){
            topNode_->setRotation(rot);
        }
        if(load(filename, archive.currentParentItem(), formatId)){
            return true;
        }
    }
    return false;
}
