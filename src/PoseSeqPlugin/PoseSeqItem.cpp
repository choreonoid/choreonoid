/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "PoseSeqItem.h"
#include "BodyMotionGenerationBar.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/LinkPath>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/Archive>
#include <cnoid/PutPropertyFunction>
#include <fmt/format.h>
#include <set>
#include <deque>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class PoseSeqItem::Impl
{
public:
    PoseSeqItem* self;
    
    BodyItem* ownerBodyItem;
    PoseSeqPtr seq;
    PoseSeqInterpolatorPtr interpolator;
    BodyMotionItemPtr bodyMotionItem;
    Connection sigInterpolationParametersChangedConnection;

    ConnectionSet editConnections;

    struct EditHistory {
        /*
          Unify these containers into one which contains elements
          in the operated orders and restore them in the same order
          when undo or redo is carried out.
        */
        PoseSeqPtr removed;
        PoseSeqPtr added;
        EditHistory(){
            removed = new PoseSeq();
            added = new PoseSeq();
        }
        bool empty(){
            return removed->empty() && added->empty();
        }
        void clear(){
            if(!empty()){
                removed = new PoseSeq();
                added = new PoseSeq();
            }
        }
    };

    struct PoseIterComp {
        bool operator()(const PoseSeq::iterator it1, const PoseSeq::iterator it2) const {
            return &(*it1) < &(*it2);
        }
    };
    std::set<PoseSeq::iterator, PoseIterComp> inserted;
    std::set<PoseSeq::iterator, PoseIterComp> modified;
            
    double modifyingPoseTime;
    double modifyingPoseTTime;
    PoseUnitPtr modifyingPoseUnitOrg;
    PoseSeq::iterator modifyingPoseIter;

    std::deque<EditHistory> editHistories;
    EditHistory newHistory;
    int currentHistory;

    BodyMotionGenerationBar* generationBar;
    TimeBar* timeBar;

    bool isSelectedPoseMoving;

    double barLength;

    Impl(PoseSeqItem* self);
    Impl(PoseSeqItem* self, const PoseSeqItem::Impl& org);
    void init();
    ~Impl();
    void onTreePathChanged();
    void convert(BodyPtr orgBody);
    bool convertSub(BodyPtr orgBody, const Mapping& convInfo);
    void updateInterpolationParameters();
    bool updateInterpolation();
    bool updateTrajectory(bool putMessages);
    void beginEditing();
    bool endEditing(bool actuallyModified);
    void onInserted(PoseSeq::iterator p, bool isMoving);
    void onRemoving(PoseSeq::iterator p, bool isMoving);
    void onModifying(PoseSeq::iterator p);
    void onModified(PoseSeq::iterator p);
    void clearEditHistory();
    PoseSeq::iterator removeSameElement(PoseSeq::iterator current, PoseSeq::iterator p);
    bool undo();
    bool redo();
    bool updateKeyPosesWithBalancedTrajectories(std::ostream& os);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}

namespace {

bool loadPoseSeqItem(PoseSeqItem* item, const std::string& filename, std::ostream& os, Item* parentItem)
{
    bool loaded = false;
    BodyItem* bodyItem = parentItem->findOwnerItem<BodyItem>(true);
    if(bodyItem){
        item->clearEditHistory();
        loaded = item->poseSeq()->load(filename, bodyItem->body());
        if(loaded){
            const string& name = item->poseSeq()->name();
            if(!name.empty()){
                item->setName(name);
            }
            if(item->poseSeq()->targetBodyName() != bodyItem->body()->name()){
                os<< format( _("Warning: the original target body {0} of \"{1}\" is"
                               "different from the current target {2}."),
                             item->poseSeq()->targetBodyName(), item->displayName(), bodyItem->body()->name());
            }
            item->notifyUpdate();
        } else {
            os << item->poseSeq()->errorMessage();
        }
    } else {
        os << _("PoseSeqItem must be loaded as a child of a BodyItem");
    }
    return loaded;
}
    
bool savePoseSeqItem(PoseSeqItem* item, const std::string& filename, std::ostream& os, Item* parentItem)
{
    BodyItem* bodyItem = parentItem->findOwnerItem<BodyItem>(true);
    if(bodyItem){
        return item->poseSeq()->save(filename, bodyItem->body());
    } else {
        os << "PoseSeqItem to save must be a child of a BodyItem ";
    }
    return false;
}

}


void PoseSeqItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    
    im.registerClass<PoseSeqItem>(N_("PoseSeqItem"));

    im.addCreationPanel<PoseSeqItem>();

    im.addLoaderAndSaver<PoseSeqItem>(
        _("Pose Sequence"), "POSE-SEQ-YAML", "pseq", loadPoseSeqItem, savePoseSeqItem);

    im.addSaver<PoseSeqItem>(
        _("Talk Plugin File"), "TALK-PLUGIN-FORMAT", "talk",
        [](PoseSeqItem* item, const std::string& filename, std::ostream&, Item*){
            return item->poseSeq()->exportTalkPluginFile(filename); },
        ItemManager::PRIORITY_CONVERSION);
        
    im.addSaver<PoseSeqItem>(
        _("Seq File for the Face Controller"), "FACE-CONTROLLER-SEQ-FORMAT", "poseseq",
        [](PoseSeqItem* item, const std::string& filename, std::ostream&, Item*){
            return item->poseSeq()->exportSeqFileForFaceController(filename); },
        ItemManager::PRIORITY_CONVERSION);

    ItemTreeView::customizeContextMenu<PoseSeqItem>(
        [](PoseSeqItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.addItem(_("Generate"))->sigTriggered().connect([item](){ item->updateTrajectory(true); });
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
}


PoseSeqItem::PoseSeqItem()
{
    impl = new Impl(this);
}


PoseSeqItem::Impl::Impl(PoseSeqItem* self)
    : self(self)
{
    seq = new PoseSeq;
    
    init();

    barLength = 1.0;
}


PoseSeqItem::PoseSeqItem(const PoseSeqItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


PoseSeqItem::Impl::Impl(PoseSeqItem* self, const PoseSeqItem::Impl& org)
    : self(self)
{
    seq = new PoseSeq(*org.seq);
    
    init();

    barLength = org.barLength;
}


void PoseSeqItem::Impl::init()
{
    ownerBodyItem = nullptr;
    
    interpolator.reset(new PoseSeqInterpolator);
    interpolator->setPoseSeq(seq);
    
    bodyMotionItem = new BodyMotionItem;
    bodyMotionItem->setName("motion");
    self->addSubItem(bodyMotionItem);

    clearEditHistory();

    generationBar = BodyMotionGenerationBar::instance();

    isSelectedPoseMoving = false;
}


PoseSeqItem::~PoseSeqItem()
{
    delete impl;
}


PoseSeqItem::Impl::~Impl()
{
    editConnections.disconnect();
    sigInterpolationParametersChangedConnection.disconnect();
}


Item* PoseSeqItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new PoseSeqItem(*this);
}


bool PoseSeqItem::setName(const std::string& name)
{
    impl->seq->setName(name);
    suggestFileUpdate();
    return Item::setName(name);
}


PoseSeq* PoseSeqItem::poseSeq()
{
    return impl->seq;
}


PoseSeqInterpolatorPtr PoseSeqItem::interpolator()
{
    return impl->interpolator;
}


BodyMotionItem* PoseSeqItem::bodyMotionItem()
{
    return impl->bodyMotionItem;
}


double PoseSeqItem::barLength() const
{
    return impl->barLength;
}


void PoseSeqItem::onTreePathChanged()
{
    impl->onTreePathChanged();
}


void PoseSeqItem::Impl::onTreePathChanged()
{
    if(!sigInterpolationParametersChangedConnection.connected()){
        sigInterpolationParametersChangedConnection =
            generationBar->sigInterpolationParametersChanged().connect(
                [&](){ updateInterpolationParameters(); });
        updateInterpolationParameters();
    }

    BodyItemPtr prevBodyItem = ownerBodyItem;
    ownerBodyItem = self->findOwnerItem<BodyItem>();
    if(ownerBodyItem == prevBodyItem){
        return;
    }
        
    if(!ownerBodyItem){
        interpolator->setBody(nullptr);

    } else {
        Body* body = ownerBodyItem->body();

        if(seq->targetBodyName().empty()){
            seq->setTargetBodyName(body->name());
        } else if(prevBodyItem && (seq->targetBodyName() != body->name())){
            convert(prevBodyItem->body());
        }

        interpolator->setBody(body);

        const Listing& linearInterpolationJoints =
            *ownerBodyItem->body()->info()->findListing("linearInterpolationJoints");
        if(linearInterpolationJoints.isValid()){
            for(int i=0; i < linearInterpolationJoints.size(); ++i){
                Link* link = body->link(linearInterpolationJoints[i].toString());
                if(link){
                    interpolator->setLinearInterpolationJoint(link->jointId());
                }
            }
        }

        LeggedBodyHelperPtr legged = getLeggedBodyHelper(ownerBodyItem->body());
        if(legged->isValid()){
            for(int i=0; i < legged->numFeet(); ++i){
                interpolator->addFootLink(legged->footLink(i)->index(), legged->centerOfSoleLocal(i));
            }
        }

        interpolator->setLipSyncShapes(*ownerBodyItem->body()->info()->findMapping("lipSyncShapes"));
        bodyMotionItem->motion()->setNumJoints(interpolator->body()->numJoints());

        if(generationBar->isAutoGenerationForNewBodyEnabled()){
            updateTrajectory(true);
        } else {
            bodyMotionItem->notifyUpdate();
        }
    }
}


void PoseSeqItem::Impl::convert(BodyPtr orgBody)
{
    if(!orgBody){
        return;
    }
    
    const Listing& convInfoTop = *ownerBodyItem->body()->info()->findListing("poseConversionInfo");
    if(convInfoTop.isValid()){
        for(int i=0; i < convInfoTop.size(); ++i){
            const Mapping& convInfo = *convInfoTop[i].toMapping();
            const Listing& targets = *convInfo["targetBodies"].toListing();
            for(int j=0; j < targets.size(); ++j){
                if(targets[j].toString() == orgBody->name()){

                    beginEditing();
                    if(endEditing(convertSub(orgBody, convInfo))){
                        
                        self->clearFileInformation();
                        BodyPtr body = ownerBodyItem->body();
                        seq->setTargetBodyName(body->name());
                        MessageView::mainInstance()->notify(
                            format(_("Pose seq \"{0}\" has been converted. Its target has been changed from {1} to {2}"),
                                   self->displayName(), orgBody->name(), body->name()));
                        
                        return;
                    }
                }
            }
        }
    }
}


bool PoseSeqItem::Impl::convertSub(BodyPtr orgBody, const Mapping& convInfo)
{
    bool converted = false;
    
    const Listing& jointMap = *convInfo.findListing("jointMap");
    const Mapping& linkMap = *convInfo.findMapping("linkMap");
    BodyPtr body = ownerBodyItem->body();
    
    for(PoseSeq::iterator p = seq->begin(); p != seq->end(); ++p){
        PosePtr pose = p->get<Pose>();
        if(pose){

            bool modified = false;
            seq->beginPoseModification(p);
                
            PosePtr orgPose = dynamic_cast<Pose*>(pose->duplicate());
            if(jointMap.isValid()){
                modified = true;
                pose->setNumJoints(0);
                int n = orgPose->numJoints();
                for(int i=0; i < n; ++i){
                    if(orgPose->isJointValid(i)){
                        if(i < jointMap.size()){
                            int newJointId = jointMap[i].toInt();
                            if(newJointId >= 0){
                                pose->setJointPosition(newJointId, orgPose->jointPosition(i));
                                pose->setJointStationaryPoint(newJointId, orgPose->isJointStationaryPoint(i));
                            }
                        }
                    }
                }
            }

            if(linkMap.isValid()){
                modified = true;
                pose->clearIkLinks();
                int baseLinkIndex = -1;
                for(Pose::LinkInfoMap::const_iterator q = orgPose->ikLinkBegin(); q != orgPose->ikLinkEnd(); ++q){
                    Link* orgLink = orgBody->link(q->first);
                    string linkName;
                    ValueNode* linkNameNode = linkMap.find(orgLink->name());
                    if(linkNameNode->isValid()){
                        linkName = linkNameNode->toString();
                    } else {
                        linkName = orgLink->name();
                    }
                    Link* link = body->link(linkName);
                    if(link){
                        const Pose::LinkInfo& orgLinkInfo = q->second;
                        Pose::LinkInfo* linkInfo = pose->addIkLink(link->index());
                        linkInfo->p = orgLinkInfo.p;
                        linkInfo->R = orgLinkInfo.R;
                        linkInfo->setStationaryPoint(orgLinkInfo.isStationaryPoint());
                        if(orgLinkInfo.isTouching()){
                            linkInfo->setTouching(orgLinkInfo.partingDirection(), orgLinkInfo.contactPoints());
                        }
                        linkInfo->setSlave(orgLinkInfo.isSlave());
                        if(orgLinkInfo.isBaseLink()){
                            baseLinkIndex = link->index();
                        }
                    }
                }
                if(baseLinkIndex >= 0){
                    pose->setBaseLink(baseLinkIndex);
                }
            }

            if(modified){
                seq->endPoseModification(p);
                converted = true;
            }
        }
    }

    return converted;
}


void PoseSeqItem::Impl::updateInterpolationParameters()
{
    interpolator->setTimeScaleRatio(generationBar->timeScaleRatio());

    interpolator->enableStealthyStepMode(generationBar->isStealthyStepMode());
    interpolator->setStealthyStepParameters(
        generationBar->stealthyHeightRatioThresh(),
        generationBar->flatLiftingHeight(), generationBar->flatLandingHeight(),
        generationBar->impactReductionHeight(), generationBar->impactReductionTime());

    interpolator->enableAutoZmpAdjustmentMode(generationBar->isAutoZmpAdjustmentMode());
    interpolator->setZmpAdjustmentParameters(
        generationBar->minZmpTransitionTime(),
        generationBar->zmpCenteringTimeThresh(),
        generationBar->zmpTimeMarginBeforeLifting(),
        generationBar->zmpMaxDistanceFromCenter());

    interpolator->enableLipSyncMix(generationBar->isLipSyncMixMode());
}        


bool PoseSeqItem::updateInterpolation()
{
    return impl->updateInterpolation();
}


bool PoseSeqItem::Impl::updateInterpolation()
{
    updateInterpolationParameters();
    return interpolator->update();
}


bool PoseSeqItem::updateTrajectory(bool putMessages)
{
    return impl->updateTrajectory(putMessages);
}


bool PoseSeqItem::Impl::updateTrajectory(bool putMessages)
{
    bool result = false;

    if(ownerBodyItem){
        result = generationBar->shapeBodyMotion(
            ownerBodyItem->body(), interpolator.get(), bodyMotionItem, putMessages);
    }

    return result;
}


void PoseSeqItem::beginEditing()
{
    impl->beginEditing();
}


void PoseSeqItem::Impl::beginEditing()
{
    newHistory.clear();
    inserted.clear();
    modified.clear();
    modifyingPoseIter = seq->end();

    if(editConnections.empty()){
        editConnections = seq->connectSignalSet(
            [&](PoseSeq::iterator p, bool isMoving){ onInserted(p, isMoving); },
            [&](PoseSeq::iterator p, bool isMoving){ onRemoving(p, isMoving); },
            [&](PoseSeq::iterator p){ onModifying(p); },
            [&](PoseSeq::iterator p){ onModified(p); });
    }
}


bool PoseSeqItem::endEditing(bool actuallyModified)
{
    return impl->endEditing(actuallyModified);
}


bool PoseSeqItem::Impl::endEditing(bool actuallyModified)
{
    if(actuallyModified){
        for(std::set<PoseSeq::iterator, PoseIterComp>::iterator p = inserted.begin(); p != inserted.end(); ++p){
            newHistory.added->insert(newHistory.added->end(), (*p)->time(), (*p)->poseUnit()->duplicate())
                ->setMaxTransitionTime((*p)->maxTransitionTime());
        }
        for(std::set<PoseSeq::iterator, PoseIterComp>::iterator p = modified.begin(); p != modified.end(); ++p){
            newHistory.added->insert(newHistory.added->end(), (*p)->time(), (*p)->poseUnit()->duplicate())
                ->setMaxTransitionTime((*p)->maxTransitionTime());
        }
        if(!newHistory.empty()){
            editHistories.resize(currentHistory);
            editHistories.push_back(newHistory);
            currentHistory = editHistories.size();
            self->suggestFileUpdate();
        } 
    }
    modifyingPoseIter = seq->end();
    inserted.clear();
    modified.clear();
    newHistory.clear();
    editConnections.disconnect();

    return actuallyModified;
}


void PoseSeqItem::Impl::onInserted(PoseSeq::iterator p, bool isMoving)
{
    if(isSelectedPoseMoving && isMoving){
        modified.insert(p);
        isSelectedPoseMoving = false;
    }
    inserted.insert(p);
}


void PoseSeqItem::Impl::onRemoving(PoseSeq::iterator p, bool isMoving)
{
    if(isMoving){
        if(modified.find(p) != modified.end()){
            modified.erase(p);
            isSelectedPoseMoving = true;
        }
    }

    if(inserted.find(p) != inserted.end()){
        inserted.erase(p);
    } else {
        newHistory.removed->insert(newHistory.removed->end(), p->time(), p->poseUnit()->duplicate())
            ->setMaxTransitionTime(p->maxTransitionTime());
    }
}


void PoseSeqItem::Impl::onModifying(PoseSeq::iterator p)
{
    modifyingPoseTime = p->time();
    modifyingPoseTTime = p->maxTransitionTime();
    modifyingPoseUnitOrg = p->poseUnit()->duplicate();
    modifyingPoseIter = p;
}


void PoseSeqItem::Impl::onModified(PoseSeq::iterator p)
{
    if(p == modifyingPoseIter){
        if(modified.find(p) == modified.end()){
            newHistory.removed->insert(newHistory.removed->end(), modifyingPoseTime, modifyingPoseUnitOrg)
                ->setMaxTransitionTime(modifyingPoseTTime);
            modified.insert(p);
        }
    }
    modifyingPoseIter = seq->end();
}


void PoseSeqItem::clearEditHistory()
{
    impl->clearEditHistory();
}


void PoseSeqItem::Impl::clearEditHistory()
{
    currentHistory = 0;
    editHistories.clear();
}


PoseSeq::iterator PoseSeqItem::Impl::removeSameElement(PoseSeq::iterator current, PoseSeq::iterator p)
{
    current = seq->seek(current, p->time());
    while(current->time() == p->time()){
        if(current->poseUnit()->hasSameParts(p->poseUnit())){
            return seq->erase(current);
        }
        ++current;
    }
    return current;
}
    

bool PoseSeqItem::undo()
{
    return impl->undo();
}


bool PoseSeqItem::Impl::undo()
{
    if(currentHistory > 0){
        editConnections.block();
        EditHistory& history = editHistories[--currentHistory];
        PoseSeqPtr added = history.added;
        PoseSeq::iterator current = seq->begin();
        for(PoseSeq::iterator p = added->begin(); p != added->end(); ++p){
            current = removeSameElement(current, p);
        }
        PoseSeqPtr removed = history.removed;
        for(PoseSeq::iterator p = removed->begin(); p != removed->end(); ++p){
            current = seq->insert(current, p->time(), p->poseUnit()->duplicate());
            current->setMaxTransitionTime(p->maxTransitionTime());
        }
        editConnections.unblock();
        self->suggestFileUpdate();
        return true;
    }
    return false;
}


bool PoseSeqItem::redo()
{
    return impl->redo();
}


bool PoseSeqItem::Impl::redo()
{
    if(currentHistory < (int)editHistories.size()){
        editConnections.block();
        EditHistory& history = editHistories[currentHistory++];
        PoseSeqPtr removed = history.removed;
        PoseSeq::iterator current = seq->begin();
        for(PoseSeq::iterator p = removed->begin(); p != removed->end(); ++p){
            current = removeSameElement(current, p);
        }
        PoseSeqPtr added = history.added;
        for(PoseSeq::iterator p = added->begin(); p != added->end(); ++p){
            current = seq->insert(current, p->time(), p->poseUnit()->duplicate());
            current->setMaxTransitionTime(p->maxTransitionTime());
        }
        editConnections.unblock();
        self->suggestFileUpdate();
        return true;
    }
    return false;
}


bool PoseSeqItem::updateKeyPosesWithBalancedTrajectories(std::ostream& os)
{
    return impl->updateKeyPosesWithBalancedTrajectories(os);
}


bool PoseSeqItem::Impl::updateKeyPosesWithBalancedTrajectories(std::ostream& os)
{
    auto motion = bodyMotionItem->motion();
    auto qseq = motion->jointPosSeq();
    auto pseq = motion->linkPosSeq();

    double length = seq->endingTime();
    
    if(qseq->timeLength() < length || pseq->timeLength() < length){
        os << "Length of the interpolated trajectories is shorter than key pose sequence.";
        return false;
    }
    if(pseq->numParts() < ownerBodyItem->body()->numLinks()){
        os << "Not all link positions are available. Please do interpolate with \"Put all link positions\"";
        return false;
    }

    beginEditing();
    
    for(PoseSeq::iterator p = seq->begin(); p != seq->end(); ++p){
        PosePtr pose = p->get<Pose>();
        if(pose){

            seq->beginPoseModification(p);
            
            int nj = pose->numJoints();
            int frame = qseq->frameOfTime(p->time());
            MultiValueSeq::Frame q = qseq->frame(frame);
            for(int i=0; i < nj; ++i){
                if(pose->isJointValid(i)){
                    pose->setJointPosition(i, q[i]);
                }
            }
            MultiSE3Seq::Frame pos = pseq->frame(frame);
            for(Pose::LinkInfoMap::iterator q = pose->ikLinkBegin(); q != pose->ikLinkEnd(); ++q){
                int linkIndex = q->first;
                Pose::LinkInfo& info = q->second;
                const Vector3& p = pos[linkIndex].translation();
                // only update horizontal position
                info.p[0] = p[0];
                info.p[1] = p[1];
            }

            seq->endPoseModification(p);
        }
    }

    endEditing(true);

    updateInterpolation();
    
    return true;
}


void PoseSeqItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void PoseSeqItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Target body"), seq->targetBodyName());
    putProperty(_("Bar length"), barLength, changeProperty(barLength));
}


bool PoseSeqItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool PoseSeqItem::Impl::store(Archive& archive)
{
    if(self->overwriteOrSaveWithDialog()){
        archive.writeFileInformation(self);
        archive.write("bar_length", barLength);
        if(bodyMotionItem->isSelected()){
            archive.write("is_body_motion_selected", true);
        }
        if(bodyMotionItem->isChecked()){
            archive.write("is_body_motion_checked", true);
        }
        return true;
    }
    return false;
}


bool PoseSeqItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool PoseSeqItem::Impl::restore(const Archive& archive)
{
    if(archive.loadFileTo(self)){
        archive.read({ "bar_length", "barLength" }, barLength);
        if(archive.get("is_body_motion_selected", false)){
            bodyMotionItem->setSelected(true);
        }
        if(archive.get("is_body_motion_checked", false)){
            bodyMotionItem->setChecked(true);
        }
        return true;
    }
    return false;
}
