#include "BodyPoseListItem.h"
#include "BodyItem.h"
#include "BodyPoseItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemFileIO>
#include <cnoid/ItemTreeView>
#include <cnoid/MenuManager>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Selection>
#include <cnoid/Archive>
#include <fmt/format.h>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class BodyPoseListFileIO : public ItemFileIoBase<BodyPoseListItem>
{
public:
    BodyPoseListFileIO();
    virtual bool load(BodyPoseListItem* listItem, const std::string& filename) override;
    virtual bool save(BodyPoseListItem* listItem, const std::string& filename) override;
};

}


void BodyPoseListItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<BodyPoseListItem>(N_("BodyPoseListItem"))
        .addCreationPanel<BodyPoseListItem>()
        .addFileIO<BodyPoseListItem>(new BodyPoseListFileIO);

    ItemTreeView::customizeContextMenu<BodyPoseListItem>(
        [](BodyPoseListItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.addItem(_("Record pose"))->sigTriggered().connect(
                [item](){ item->recordCurrentPose(); });
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
}


BodyPoseListItem::BodyPoseListItem()
{
    setName("PoseList");
    targetBodyItem_ = nullptr;
    poseListArchiveMode_ = PoseItemArchiveMode;
}


BodyPoseListItem::BodyPoseListItem(const BodyPoseListItem& org)
    : Item(org)
{
    targetBodyItem_ = nullptr;
    poseListArchiveMode_ = org.poseListArchiveMode_;
}


BodyPoseListItem::~BodyPoseListItem()
{

}


Item* BodyPoseListItem::doDuplicate() const
{
    return new BodyPoseListItem(*this);
}


std::string BodyPoseListItem::displayName() const
{
    if(hasDisplayNameModifier()){
        return Item::displayName();
    }
    if(name() == "PoseList"){
        return _("PoseList");
    }
    return name();
}


void BodyPoseListItem::onTreePathChanged()
{
    auto newBodyItem = findOwnerItem<BodyItem>();
    if(newBodyItem != targetBodyItem_){
        targetBodyItem_ = newBodyItem;
    }
}


void BodyPoseListItem::setPoseListArchiveMode(int mode)
{
    if(mode != poseListArchiveMode_){
        poseListArchiveMode_ = mode;
        if(mode == PoseItemArchiveMode){
            clearFileInformation();
        } else if(mode == PoseListFileArchiveMode){
            suggestFileUpdate();
        }
    }
}


void BodyPoseListItem::addPoseItem(BodyPoseItem* item)
{
    if(item->name().empty()){
        int maxPoseNumber = 0;
        static regex re("^(.+)(\\d+)$");
        smatch match;
        for(auto& poseItem : childItems<BodyPoseItem>()){
            if(regex_match(poseItem->name(), match, re)){
                int number = std::stoi(match.str(2));
                if(number > maxPoseNumber){
                    maxPoseNumber = number;
                }
            }
        }
        item->setName(format("Pose {0}", maxPoseNumber + 1));
    }
    addChildItem(item);

    if(poseListArchiveMode_ == PoseListFileArchiveMode){
        suggestFileUpdate();
    }
}


void BodyPoseListItem::recordCurrentPose()
{
    auto poseItem = new BodyPoseItem;
    addPoseItem(poseItem);
    poseItem->fetchBodyPose();
}


bool BodyPoseListItem::loadPoseListFile(const std::string& filename)
{
    bool loaded = false;

    YAMLReader reader;
    loaded = reader.load(filename);
    if(loaded){
        if(reader.numDocuments() > 0){
            auto archive = reader.document()->toMapping();
            auto poseNodes = archive->findListing("poses");
            if(!poseNodes->empty()){
                if(auto bodyItem = findOwnerItem<BodyItem>()){
                    auto body = bodyItem->body();
                    auto existingPoseItems = childItems<BodyPoseItem>();
                    int index = 0;
                    while(index < poseNodes->size()){
                        BodyPoseItemPtr poseItem;
                        if(index < existingPoseItems.size()){
                            poseItem = existingPoseItems[index];
                        } else {
                            poseItem = new BodyPoseItem;
                        }
                        auto poseNode = poseNodes->at(index)->toMapping();
                        if(!poseItem->readBodyPose(poseNode, body, true)){
                            loaded = false;
                            break;
                        }
                        if(!poseItem->parentItem()){
                            addChildItem(poseItem);
                        }
                        ++index;
                    }
                    while(index < existingPoseItems.size()){
                        existingPoseItems[index]->removeFromParentItem();
                    }
                }
            }
        }
    }

    return loaded;
}


bool BodyPoseListItem::savePosesAsPoseListFile(const std::string& filename)
{
    YAMLWriter writer(filename);
    writer.setKeyOrderPreservationMode(true);

    MappingPtr archive = new Mapping;
    archive->setFloatingNumberFormat("%.9g");

    auto poseItems = descendantItems<BodyPoseItem>();
    if(!poseItems.empty()){
        auto poseNodes = archive->createListing("poses");
        for(auto& poseItem : poseItems){
            MappingPtr node = new Mapping;
            if(poseItem->writeBodyPose(node, true)){
                poseNodes->append(node);
            }
        }
    }

    writer.putNode(archive);

    return true;
}


void BodyPoseListItem::doPutProperties(PutPropertyFunction& putProperty)
{
    Selection mode = { _("Pose item"), _("Pose list file") };
    mode.select(poseListArchiveMode_);
    putProperty(_("Archive mode"), mode, [this](int mode){ setPoseListArchiveMode(mode); return true; });
}


bool BodyPoseListItem::store(Archive& archive)
{
    if(poseListArchiveMode_ == PoseItemArchiveMode){
        archive.write("archive_mode", "pose_item");
    } else if(poseListArchiveMode_ == PoseListFileArchiveMode){
        if(!overwriteOrSaveWithDialog()){
            return false;
        }
        archive.write("archive_mode", "pose_list_file");
        archive.writeFileInformation(this);
    }
    return true;
}


bool BodyPoseListItem::restore(const Archive& archive)
{
    bool restored = false;
    
    string mode;
    archive.read("archive_mode", mode);
    if(mode == "pose_item"){
        setPoseListArchiveMode(PoseItemArchiveMode);
        restored = true;
    } else if(mode == "pose_list_file"){
        setPoseListArchiveMode(PoseListFileArchiveMode);
        if(auto bodyItem = archive.currentParentItem()->findOwnerItem<BodyItem>(true)){
            archive.addProcessOnSubTreeRestored(
                bodyItem,
                [this, &archive](){ archive.loadFileTo(this); });
            restored = true;
        }
    }
        
    return restored;
}


BodyPoseListFileIO::BodyPoseListFileIO()
    : ItemFileIoBase("BODY-POSE-LIST-YAML", Load | Save)
{
    setCaption(_("Body Pose List"));
    setFileTypeCaption(_("Body Pose List File"));
    setExtension("yaml");
}


bool BodyPoseListFileIO::load(BodyPoseListItem* listItem, const std::string& filename)
{
    return listItem->loadPoseListFile(filename);
}


bool BodyPoseListFileIO::save(BodyPoseListItem* listItem, const std::string& filename)
{
    if(listItem->savePosesAsPoseListFile(filename)){
        listItem->setPoseListArchiveMode(BodyPoseListItem::PoseListFileArchiveMode);
        return true;
    }
    return false;
}
