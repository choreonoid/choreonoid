/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneItem.h"
#include "ItemManager.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include <cnoid/FileUtil>
#include <cnoid/VRMLParser>
#include <cnoid/STLSceneLoader>
#include <cnoid/EasyScanner>
#include <cnoid/VRMLToSGConverter>
#include <cnoid/EigenArchive>
#include <cnoid/Exception>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

std::unique_ptr<VRMLParser> vrmlParser;
std::unique_ptr<VRMLToSGConverter> vrmlConverter;

bool loadVRML(SceneItem* item, const std::string& filename, std::ostream& os)
{
    item->topNode()->clearChildren(true);
    
    if(!vrmlParser){
        vrmlParser.reset(new VRMLParser);
        vrmlConverter.reset(new VRMLToSGConverter);
    }
    vrmlConverter->setMessageSink(os);
    vrmlConverter->clearConvertedNodeMap();

    SgInvariantGroupPtr group = new SgInvariantGroup;

    try {
        vrmlParser->load(filename);
        while(VRMLNodePtr vrml = vrmlParser->readNode()){
            SgNodePtr node = vrmlConverter->convert(vrml);
            if(node){
                group->addChild(node);
            }
        }
        vrmlParser->checkEOF();

    } catch(EasyScanner::Exception& ex){
        os << ex.getFullMessage();
        return false;
    }
        
    if(group->empty()){
        os << _("The VRML file does not have any valid entity.") << endl;
    } else {
        item->topNode()->addChild(group, true);
        return true;
    }
    return false;
}

bool loadSTL(SceneItem* item, const std::string& filename, std::ostream& os)
{
    STLSceneLoader loader;
    SgNode* scene = loader.load(filename);
    if(!scene){
        os << _("The STL file cannot be loaded.") << endl;
    } else {
        item->topNode()->addChild(scene);
    }
    return (scene != 0);
}

}


void SceneItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<SceneItem>(N_("SceneItem"));

        ext->itemManager().addLoader<SceneItem>(
            "VRML", "VRML-FILE", "wrl",
            std::bind(::loadVRML, _1, _2, _3), ItemManager::PRIORITY_CONVERSION);

        ext->itemManager().addLoader<SceneItem>(
            "Stereolithography (STL)", "STL-FILE", "stl",
            std::bind(::loadSTL, _1, _2, _3), ItemManager::PRIORITY_CONVERSION);
        
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
    return topNode_.get();
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
    Vector3 rpy(rpyFromRot(topNode_->rotation()));
    putProperty("RPY", str(TO_DEGREE * rpy), std::bind(&SceneItem::onRotationChanged, this, _1));
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
