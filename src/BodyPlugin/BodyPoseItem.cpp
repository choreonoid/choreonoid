#include "BodyPoseItem.h"
#include "BodyPoseListItem.h"
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/Archive>
#include <cnoid/ValueTree>
#include <cnoid/MathUtil>
#include <fmt/format.h>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


void BodyPoseItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BodyPoseItem>("BodyPoseItem");

    ItemTreeView::customizeContextMenu<BodyPoseItem>(
        [](BodyPoseItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.addItem(_("Apply pose"))->sigTriggered().connect(
                [item](){ item->applyBodyPose(); });
            menuManager.addItem(_("Fetch pose"))->sigTriggered().connect(
                [item](){ item->fetchBodyPose(); });
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
}


BodyPoseItem::BodyPoseItem()
{
    poseListItem_ = nullptr;
}


BodyPoseItem::BodyPoseItem(const BodyPoseItem& org)
    : Item(org),
      jointDisplacements(org.jointDisplacements)
{
    poseListItem_ = nullptr;
}


Item* BodyPoseItem::doDuplicate() const
{
    return new BodyPoseItem(*this);
}


std::string BodyPoseItem::displayName() const
{
    if(hasDisplayNameModifier()){
        return Item::displayName();
    }
    static regex re("^Pose( ?)( *)(\\d+)$");
    smatch match;
    if(regex_match(name(), match, re)){
        return format(_("Pose{0}{1}{2}"), match.str(1), match.str(2), match.str(3));
    }
    return name();
}


void BodyPoseItem::onTreePathChanged()
{
    auto newPoseListItem = findOwnerItem<BodyPoseListItem>();
    if(newPoseListItem != poseListItem_){
        poseListItem_ = newPoseListItem;
    }
}


bool BodyPoseItem::onNewTreePositionCheck(bool isManualOperation, std::function<void()>& out_callbackWhenAdded)
{
    bool accepted = dynamic_cast<BodyPoseListItem*>(parentItem()) != nullptr;
    return accepted;
}


BodyItem* BodyPoseItem::targetBodyItem() const
{
    if(poseListItem_){
        return poseListItem_->targetBodyItem();
    }
    return nullptr;
}


void BodyPoseItem::onDoubleClicked()
{
    applyBodyPose();
}


bool BodyPoseItem::fetchBodyPose()
{
    bool fetched = false;
    
    if(auto bodyItem = targetBodyItem()){
        jointDisplacements.clear();
        auto body = bodyItem->body();
        int nj = body->numAllJoints();
        for(int i=0; i < nj; ++i){
            jointDisplacements.push_back(body->joint(i)->q());
        }
        if(poseListItem_ && poseListItem_->poseListArchiveMode() == BodyPoseListItem::PoseListFileArchiveMode){
            poseListItem_->suggestFileUpdate();
        }
        fetched = true;
    }

    return fetched;
}


bool BodyPoseItem::applyBodyPose()
{
    bool restored = false;
    
    if(auto bodyItem = targetBodyItem()){
        auto body = bodyItem->body();
        for(size_t i=0; i < jointDisplacements.size(); ++i){
            body->joint(i)->q() = jointDisplacements[i];
        }
        bodyItem->notifyKinematicStateChange(true);
    }

    return restored;
}


bool BodyPoseItem::readBodyPose(const Mapping* archive, const Body* body, bool doReadMetaData)
{
    bool result = false;

    string name_;
    if(archive->read("name", name_)){
        setName(name_);
    }
    
    auto& jdlist = *archive->findListing("joint_displacements");
    if(jdlist.isValid()){
        int n = jdlist.size();
        jointDisplacements.resize(n);
        result = true;
        for(int i=0; i < n; ++i){
            double q;
            if(jdlist[i].read(q)){
                if(auto joint = body->joint(i)){
                    if(joint->isRevoluteJoint()){
                        q = radian(q);
                    }
                }
                jointDisplacements[i] = q;
            } else {
                result = false;
                break;
            }
        }
        if(!result){
            jointDisplacements.clear();
        }
    }
    return result;
}


bool BodyPoseItem::writeBodyPose(Mapping* archive, bool doWriteMetaData) const
{
    bool result = false;

    if(auto bodyItem = targetBodyItem()){
        if(doWriteMetaData){
            archive->write("name", name(), DOUBLE_QUOTED);
        }

        auto body = bodyItem->body();
        auto jdlist = archive->createFlowStyleListing("joint_displacements");
        int n = jointDisplacements.size();
        for(int i=0; i < n; ++i){
            double q = jointDisplacements[i];
            if(body->joint(i)->isRevoluteJoint()){
                q = degree(q);
            }
            jdlist->append(q, 10, n);
        }

        result = true;
    }

    return result;
}


bool BodyPoseItem::store(Archive& archive)
{
    if(poseListItem_){
        if(poseListItem_->poseListArchiveMode() == BodyPoseListItem::PoseItemArchiveMode){
            writeBodyPose(&archive, false);
        }
    }
    return true;
}


bool BodyPoseItem::restore(const Archive& archive)
{
    if(auto poseListItem = archive.currentParentItem()->findOwnerItem<BodyPoseListItem>(true)){
        if(poseListItem->poseListArchiveMode() == BodyPoseListItem::PoseItemArchiveMode){
            if(auto bodyItem = poseListItem->targetBodyItem()){
                const Mapping* pArchive = &archive;
                archive.addProcessOnSubTreeRestored(
                    bodyItem,
                    [this, pArchive, bodyItem](){ restorePose(pArchive, bodyItem); });
            }
        }
    }
    return true;
}


void BodyPoseItem::restorePose(const Mapping* archive, BodyItem* bodyItem)
{
    readBodyPose(archive, bodyItem->body(), false);
}

