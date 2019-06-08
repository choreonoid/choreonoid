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
#include <cnoid/SceneEffects>
#include "gettext.h"

using namespace std;
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
        auto group = new SgInvariantGroup;
        group->addChild(scene);
        auto topNode = item->topNode();
        topNode->clearChildren();
        topNode->addChild(group);
        item->setSimplifiedRenderingEnabled(item->isSimplifiedRenderingEnabled());
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
            [&](SceneItem* item, const std::string& filename, std::ostream& os, Item* parentItem){
                return ::loadScene(item, filename, os);
            },
            ItemManager::PRIORITY_CONVERSION);

        ext->itemManager().addLoader<SceneItem>(
            "VRML", "VRML-FILE", "wrl",
            [&](SceneItem* item, const std::string& filename, std::ostream& os, Item* parentItem){
                return ::loadScene(item, filename, os);
            },
            ItemManager::PRIORITY_COMPATIBILITY);

        ext->itemManager().addLoader<SceneItem>(
            "Stereolithography (STL)", "STL-FILE", "stl",
            [&](SceneItem* item, const std::string& filename, std::ostream& os, Item* parentItem){
                return ::loadScene(item, filename, os);
            },
            ItemManager::PRIORITY_COMPATIBILITY);

        initialized = true;
    }
}


SceneItem::SceneItem()
{
    topNode_ = new SgPosTransform;
    isSimplifiedRenderingEnabled_ = false;
}


SceneItem::SceneItem(const SceneItem& org)
    : Item(org)
{
    // shallow copy
    topNode_ = new SgPosTransform(*org.topNode());
    isSimplifiedRenderingEnabled_ = org.isSimplifiedRenderingEnabled_;
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

void SceneItem::setSimplifiedRenderingEnabled(bool on)
{
    if(on){
        if(!topNode_->findNodeOfType<SgSimplifiedRenderingGroup>(1)){
            auto simplified = new SgSimplifiedRenderingGroup;
            topNode_->moveChildrenTo(simplified);
            topNode_->addChild(simplified, true);
        }
    } else {
        auto simplified = topNode_->findNodeOfType<SgSimplifiedRenderingGroup>(1);
        if(simplified){
            topNode_->removeChild(simplified);
            simplified->moveChildrenTo(topNode_, true);
        }
    }
    isSimplifiedRenderingEnabled_ = on;
}       


void SceneItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("File"), getFilename(filePath()));

    putProperty(_("Translation"), str(Vector3(topNode_->translation())),
                [&](const std::string& value){
                    Vector3 p;
                    if(toVector3(value, p)){
                        topNode_->setTranslation(p);
                        topNode_->notifyUpdate();
                        return true;
                    }
                    return false;
                });

    Vector3 rpy(TO_DEGREE * rpyFromRot(topNode_->rotation()));
    
    putProperty("RPY", str(rpy),
                [&](const std::string& value){
                    Vector3 rpy;
                    if(toVector3(value, rpy)){
                        topNode_->setRotation(rotFromRpy(TO_RADIAN * rpy));
                        topNode_->notifyUpdate();
                        return true;
                    }
                    return false;
                });
    
    putProperty("Simplified rendering", isSimplifiedRenderingEnabled_,
                [&](bool on){ setSimplifiedRenderingEnabled(on); return true; });
}


bool SceneItem::store(Archive& archive)
{
    if(!filePath().empty()){
        archive.writeRelocatablePath("file", filePath());
        archive.write("format", fileFormat());
        write(archive, "translation", topNode_->translation());
        write(archive, "rotation", AngleAxis(topNode_->rotation()));
        archive.write("simplifiedRendering", isSimplifiedRenderingEnabled_);
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

        archive.read("simplifiedRendering", isSimplifiedRenderingEnabled_);
        
        if(load(filename, archive.currentParentItem(), formatId)){
            return true;
        }
    }
    return false;
}
