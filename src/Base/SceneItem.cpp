/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneItem.h"
#include "ItemManager.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include <cnoid/SceneLoader>
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
        item->setLightweightRenderingEnabled(item->isLightweightRenderingEnabled());
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
            _("Scene"), "AVAILABLE-SCENE-FILE", SceneLoader::availableFileExtensions,
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
    isLightweightRenderingEnabled_ = false;
}


SceneItem::SceneItem(const SceneItem& org)
    : Item(org)
{
    // shallow copy
    topNode_ = new SgPosTransform(*org.topNode());
    isLightweightRenderingEnabled_ = org.isLightweightRenderingEnabled_;
}


SceneItem::~SceneItem()
{
    
}


Item* SceneItem::doDuplicate() const
{
    return new SceneItem(*this);
}


bool SceneItem::setName(const std::string& name)
{
    topNode_->setName(name);
    return Item::setName(name);
}


SgNode* SceneItem::getScene()
{
    return topNode_;
}


void SceneItem::setTranslation(const Vector3f& translation)
{
    topNode_->setTranslation(translation);
    topNode_->notifyUpdate();
}


void SceneItem::setRotation(const AngleAxisf& rotation)
{
    topNode_->setRotation(rotation);
    topNode_->notifyUpdate();
}


void SceneItem::setLightweightRenderingEnabled(bool on)
{
    if(on){
        if(!topNode_->findNodeOfType<SgLightweightRenderingGroup>(1)){
            auto lightweight = new SgLightweightRenderingGroup;
            topNode_->moveChildrenTo(lightweight);
            topNode_->addChild(lightweight, true);
        }
    } else {
        auto lightweight = topNode_->findNodeOfType<SgLightweightRenderingGroup>(1);
        if(lightweight){
            lightweight->moveChildrenTo(topNode_);
            topNode_->removeChild(lightweight, true);
        }
    }
    isLightweightRenderingEnabled_ = on;
}       


void SceneItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("File"), fileName());

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
    
    putProperty(_("Rotation"), str(rpy),
                [&](const std::string& value){
                    Vector3 rpy;
                    if(toVector3(value, rpy)){
                        topNode_->setRotation(rotFromRpy(TO_RADIAN * rpy));
                        topNode_->notifyUpdate();
                        return true;
                    }
                    return false;
                });
    
    putProperty(_("Lightweight rendering"), isLightweightRenderingEnabled_,
                [&](bool on){ setLightweightRenderingEnabled(on); return true; });
}


bool SceneItem::store(Archive& archive)
{
    if(archive.writeFileInformation(this)){
        write(archive, "translation", topNode_->translation());

        writeDegreeAngleAxis(archive, "rotation", AngleAxis(topNode_->rotation()));
        // The following element is used to distinguish the value type from the old one using radian.
        // The old format is deprecated, and writing the following element should be omitted in the future.
        archive.write("angle_unit", "degree");
        
        archive.write("lightweight_rendering", isLightweightRenderingEnabled_);
    }
    return true;
}


bool SceneItem::restore(const Archive& archive)
{
    if(archive.loadFileTo(this)){
        Vector3 translation;
        if(read(archive, "translation", translation)){
            topNode_->setTranslation(translation);
        }
        AngleAxis rot;
        string unit;
        bool hasRot = false;
        if(archive.read("angle_unit", unit) && unit == "degree"){
            hasRot = readDegreeAngleAxis(archive, "rotation", rot);
        } else { // for the backward compatibility
            hasRot = readAngleAxis(archive, "rotation", rot);
        }
        if(hasRot){
            topNode_->setRotation(rot);
        }
        archive.read("lightweight_rendering", isLightweightRenderingEnabled_);
        return true;
    }
    return false;
}
