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
#include <cnoid/EasyScanner>
#include <cnoid/VRMLToSGConverter>
#include <cnoid/EigenArchive>
#include <cnoid/Exception>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

scoped_ptr<VRMLParser> vrmlParser;
scoped_ptr<VRMLToSGConverter> vrmlConverter;
    
bool loadVRML(SceneItem* item, const std::string& filename, std::ostream& os)
{
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
        item->topNode()->addChild(group);
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
            _("VRML"), "VRML-FILE", "wrl",
            bind(::loadVRML, _1, _2, _3), ItemManager::PRIORITY_CONVERSION);
        
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


SgNode* SceneItem::scene()
{
    return topNode_.get();
}


ItemPtr SceneItem::doDuplicate() const
{
    return new SceneItem(*this);
}


void SceneItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("File"), getFilename(filesystem::path(lastAccessedFilePath())));
    putProperty(_("Translation"), str(Vector3(topNode_->translation())),
                bind(&SceneItem::onTranslationChanged, this, _1));
    Vector3 rpy(rpyFromRot(topNode_->rotation()));
    putProperty(_("RPY"), str(TO_DEGREE * rpy), bind(&SceneItem::onRotationChanged, this, _1));
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
    if(!lastAccessedFilePath().empty()){
        archive.writeRelocatablePath("file", lastAccessedFilePath());
        archive.write("format", lastAccessedFileFormatId());
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
