#include "PoseSeqItem.h"
#include "BodyKeyPose.h"
#include "BodyMotionGenerationBar.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemFileIO>
#include <cnoid/ItemTreeView>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/LinkPath>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/Archive>
#include <cnoid/PutPropertyFunction>
#include <cnoid/ConnectionSet>
#include <cnoid/CheckBox>
#include <fmt/format.h>
#include <set>
#include <deque>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class PoseSeqItem::Impl
{
public:
    PoseSeqItem* self;
    
    BodyItem* targetBodyItem;
    PoseSeqPtr seq;
    PoseSeqInterpolatorPtr interpolator;
    BodyMotionItemPtr bodyMotionItem;
    Connection sigInterpolationParametersChangedConnection;

    struct PoseSeqIteratorComp {
        bool operator()(const PoseSeq::iterator& it1, const PoseSeq::iterator& it2) const {
            return &(*it1) < &(*it2);
        }
    };

    std::vector<PoseSeq::iterator> selectedPoses;
    // Used to check the duplication
    std::set<PoseSeq::iterator, PoseSeqIteratorComp> selectedPoseSet;
    Signal<void(const std::vector<PoseSeq::iterator>& poses)> sigPoseSelectionChanged;

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

    std::set<PoseSeq::iterator, PoseSeqIteratorComp> inserted;
    std::set<PoseSeq::iterator, PoseSeqIteratorComp> modified;
            
    PoseSeq::iterator modifyingPoseIter;
    AbstractPosePtr preModifiedPose;
    double preModifiedPoseTime;
    double preModifiedPoseTransitionTime;

    std::deque<EditHistory> editHistories;
    EditHistory newHistory;
    int currentHistory;

    BodyMotionGenerationBar* generationBar;

    bool isSelectedPoseBeingMoved;
    bool isPoseSelectionChangedByEditing;

    double barLength;

    Impl(PoseSeqItem* self);
    Impl(PoseSeqItem* self, const PoseSeqItem::Impl& org);
    void init();
    ~Impl();
    bool clearPoseSelection(bool doNotify);
    void selectPose(PoseSeq::iterator pose, bool doNotify, bool doSort);
    void selectAllPoses(bool doNotify);
    bool deselectPose(PoseSeq::iterator pose, bool doNotify);
    void onTreePathChanged();
    void convert(BodyPtr orgBody);
    void convertSub(BodyPtr orgBody, const Mapping& convInfo);
    void updateInterpolationParameters();
    bool updateInterpolation();
    bool updateTrajectory(bool putMessages);
    void beginEditing();
    bool endEditing(bool actuallyModified);
    void onPoseInserted(PoseSeq::iterator pose, bool isMoving);
    void onPoseAboutToBeRemoved(PoseSeq::iterator pose, bool isMoving);
    void onPoseAboutToBeModified(PoseSeq::iterator pose);
    void onPoseModified(PoseSeq::iterator pose);
    void clearEditHistory();
    PoseSeq::iterator removeSameElement(PoseSeq::iterator current, PoseSeq::iterator p);
    bool undo();
    bool redo();
    bool updatePosesWithBalancedTrajectories(std::ostream& os);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}

namespace {

class PoseSeqFileIO : public ItemFileIO
{
    CheckBox* contactPointOutputCheck;
public:
    PoseSeqFileIO();
    virtual Item* createItem() override;
    virtual bool load(Item* item, const std::string& filename) override;
    virtual bool save(Item* item, const std::string& filename) override;
    virtual QWidget* getOptionPanelForSaving(Item* item) override;
};

}


PoseSeqFileIO::PoseSeqFileIO()
    : ItemFileIO("POSE-SEQ-YAML", Load | Save | Options | OptionPanelForSaving)
{
    setCaption(_("Pose Sequence"));
    setFileTypeCaption(_("Pose Sequence File"));
    setExtension("pseq");

    contactPointOutputCheck = nullptr;
}


Item* PoseSeqFileIO::createItem()
{
    return new PoseSeqItem;
}


bool PoseSeqFileIO::load(Item* item, const std::string& filename)
{
    bool loaded = false;

    auto bodyItem = parentItem()->findOwnerItem<BodyItem>(true);
    if(!bodyItem){
        putError(_("PoseSeqItem must be loaded as a child of a BodyItem"));
    } else {
        auto pseqItem = static_cast<PoseSeqItem*>(item);
        pseqItem->clearEditHistory();
        auto pseq = pseqItem->poseSeq();
        loaded = pseq->load(filename, bodyItem->body());
        if(!loaded){
            putError(pseq->errorMessage());
        } else {
            const string& name = pseq->name();
            if(!name.empty()){
                pseqItem->setName(name);
            }
            if(pseq->targetBodyName() != bodyItem->body()->name()){
                putWarning(
                    format(_("The original target body {0} of \"{1}\" is different from the current target {2}."),
                           pseq->targetBodyName(), pseqItem->displayName(), bodyItem->body()->name()));
            }
            pseqItem->notifyUpdate();
        }
    }
    
    return loaded;
}


bool PoseSeqFileIO::save(Item* item, const std::string& filename)
{
    bool saved = false;
    auto bodyItem = parentItem()->findOwnerItem<BodyItem>(true);
    if(bodyItem){
        saved = static_cast<PoseSeqItem*>(item)->poseSeq()->save(filename, bodyItem->body());
    } else {
        putError(_("PoseSeqItem to save must be a child of a BodyItem."));
    }
    return saved;
}


QWidget* PoseSeqFileIO::getOptionPanelForSaving(Item* /* item */)
{
    if(!contactPointOutputCheck){
        contactPointOutputCheck = new CheckBox(_("Output contact points"));
        contactPointOutputCheck->sigToggled().connect(
            [](bool on){ BodyKeyPose::setContactPointOutputEnabled(on); });
    }
    contactPointOutputCheck->setChecked(BodyKeyPose::isContactPointOutputEnabled());
    return contactPointOutputCheck;
}


void PoseSeqItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    
    im.registerClass<PoseSeqItem>(N_("PoseSeqItem"));

    im.addCreationPanel<PoseSeqItem>();

    im.addFileIO<PoseSeqItem>(new PoseSeqFileIO);

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
    targetBodyItem = nullptr;
    
    interpolator.reset(new PoseSeqInterpolator);
    interpolator->setPoseSeq(seq);
    
    bodyMotionItem = new BodyMotionItem;
    bodyMotionItem->setName("motion");
    self->addSubItem(bodyMotionItem);

    clearEditHistory();

    generationBar = BodyMotionGenerationBar::instance();

    isSelectedPoseBeingMoved = false;
    isPoseSelectionChangedByEditing = false;
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


BodyItem* PoseSeqItem::targetBodyItem()
{
    return impl->targetBodyItem;
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


const std::vector<PoseSeq::iterator>& PoseSeqItem::selectedPoses() const
{
    return impl->selectedPoses;
}


bool PoseSeqItem::clearPoseSelection(bool doNotify)
{
    return impl->clearPoseSelection(doNotify);
}


bool PoseSeqItem::Impl::clearPoseSelection(bool doNotify)
{
    bool cleared = false;
    if(!selectedPoses.empty()){
        selectedPoses.clear();
        selectedPoseSet.clear();
        if(doNotify){
            self->notifyPoseSelectionChange();
        }
        cleared = true;
    }
    return cleared;
}


void PoseSeqItem::selectPose(PoseSeq::iterator pose, bool doNotify, bool doSort)
{
    impl->selectPose(pose, doNotify, doSort);
}


void PoseSeqItem::Impl::selectPose(PoseSeq::iterator pose, bool doNotify, bool doSort)
{
    auto inserted = selectedPoseSet.insert(pose);
    if(inserted.second){
        selectedPoses.push_back(pose);
        if(doSort){
            std::stable_sort(
                selectedPoses.begin(),
                selectedPoses.end(),
                [](const PoseSeq::iterator& a, const PoseSeq::iterator& b){
                    return a->time() < b->time();
                });
        }
        if(doNotify){
            self->notifyPoseSelectionChange();
        }
    }
}


void PoseSeqItem::selectAllPoses(bool doNotify)
{
    impl->selectAllPoses(doNotify);
}


void PoseSeqItem::Impl::selectAllPoses(bool doNotify)
{
    if(seq->empty() && selectedPoses.empty()){
        return;
    }
    clearPoseSelection(false);
    for(auto it = seq->begin(); it != seq->end(); ++it){
        selectPose(it, false, false);
    }
    if(doNotify){
        self->notifyPoseSelectionChange();
    }
}


bool PoseSeqItem::deselectPose(PoseSeq::iterator pose, bool doNotify)
{
    return impl->deselectPose(pose, doNotify);
}


bool PoseSeqItem::Impl::deselectPose(PoseSeq::iterator pose, bool doNotify)
{
    bool deselected = false;
    if(selectedPoseSet.erase(pose) > 0){
        auto found =
            std::equal_range(
            selectedPoses.begin(),
            selectedPoses.end(),
            pose,
            [](const PoseSeq::iterator& a, const PoseSeq::iterator& b){
                return a->time() < b->time();
            });
        selectedPoses.erase(found.first, found.second);
        deselected = true;
        if(doNotify){
            self->notifyPoseSelectionChange();
        }
    }
    return deselected;
}


bool PoseSeqItem::checkSelected(PoseSeq::iterator pose) const
{
    return (impl->selectedPoseSet.find(pose) != impl->selectedPoseSet.end());
}


void PoseSeqItem::notifyPoseSelectionChange()
{
    impl->sigPoseSelectionChanged(impl->selectedPoses);
    
}


SignalProxy<void(const std::vector<PoseSeq::iterator>& poses)> PoseSeqItem::sigPoseSelectionChanged()
{
    return impl->sigPoseSelectionChanged;
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

    BodyItemPtr prevBodyItem = targetBodyItem;
    targetBodyItem = self->findOwnerItem<BodyItem>();
    if(targetBodyItem == prevBodyItem){
        return;
    }
        
    if(!targetBodyItem){
        interpolator->setBody(nullptr);

    } else {
        Body* body = targetBodyItem->body();

        if(seq->targetBodyName().empty()){
            seq->setTargetBodyName(body->name());
        } else if(prevBodyItem && (seq->targetBodyName() != body->name())){
            convert(prevBodyItem->body());
        }

        interpolator->setBody(body);

        const Listing& linearInterpolationJoints =
            *targetBodyItem->body()->info()->findListing("linearInterpolationJoints");
        if(linearInterpolationJoints.isValid()){
            for(int i=0; i < linearInterpolationJoints.size(); ++i){
                Link* link = body->link(linearInterpolationJoints[i].toString());
                if(link){
                    interpolator->setLinearInterpolationJoint(link->jointId());
                }
            }
        }

        interpolator->setLipSyncShapes(*targetBodyItem->body()->info()->findMapping("lipSyncShapes"));
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
    
    const Listing& convInfoTop = *targetBodyItem->body()->info()->findListing("pose_conversion_info");
    if(convInfoTop.isValid()){
        for(int i=0; i < convInfoTop.size(); ++i){
            const Mapping& convInfo = *convInfoTop[i].toMapping();
            const Listing& targets = *convInfo["target_body_models"].toListing();
            for(int j=0; j < targets.size(); ++j){
                if(targets[j].toString() == orgBody->name()){
                    beginEditing();
                    convertSub(orgBody, convInfo);
                    endEditing(true);
                    self->clearFileInformation();
                    BodyPtr body = targetBodyItem->body();
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


void PoseSeqItem::Impl::convertSub(BodyPtr orgBody, const Mapping& convInfo)
{
    const Listing& jointMap = *convInfo.findListing("joint_map");
    const Mapping& linkMap = *convInfo.findMapping("link_map");
    BodyPtr body = targetBodyItem->body();

    for(auto it = seq->begin(); it != seq->end(); ++it){
        auto pose = it->get<BodyKeyPose>();
        if(!pose){
            continue;
        }
        seq->beginPoseModification(it);
        BodyKeyPosePtr orgPose = pose->clone();

        if(jointMap.isValid()){
            pose->setNumJoints(0);
            int n = orgPose->numJoints();
            for(int i=0; i < n; ++i){
                if(orgPose->isJointValid(i)){
                    if(i < jointMap.size()){
                        int newJointId = jointMap[i].toInt();
                        if(newJointId >= 0){
                            pose->setJointDisplacement(newJointId, orgPose->jointDisplacement(i));
                            pose->setJointStationaryPoint(newJointId, orgPose->isJointStationaryPoint(i));
                        }
                    }
                }
            }
        }

        pose->clearIkLinks();
        int baseLinkIndex = -1;
        for(auto ikLinkIter = orgPose->ikLinkBegin(); ikLinkIter != orgPose->ikLinkEnd(); ++ikLinkIter){
            Link* orgLink = orgBody->link(ikLinkIter->first);
            string linkName = orgLink->name();
            if(linkMap.isValid()){
                ValueNode* linkNameNode = linkMap.find(orgLink->name());
                if(linkNameNode->isValid()){
                    linkName = linkNameNode->toString();
                }
            }
            Link* link = body->link(linkName);
            if(link){
                const BodyKeyPose::LinkInfo& orgLinkInfo = ikLinkIter->second;
                BodyKeyPose::LinkInfo* linkInfo = pose->getOrCreateIkLink(link->index());
                linkInfo->setPosition(orgLinkInfo.position());
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
        
        seq->endPoseModification(it);
    }
}


void PoseSeqItem::Impl::updateInterpolationParameters()
{
    interpolator->setTimeScaleRatio(generationBar->timeScaleRatio());

    /*
    interpolator->enableStealthyStepMode(
        generationBar->stepTrajectoryAdjustmentMode() == PoseSeqInterpolator::StealthyStepMode);
    */

    interpolator->setStepTrajectoryAdjustmentMode(generationBar->stepTrajectoryAdjustmentMode());
    
    interpolator->setStealthyStepParameters(
        generationBar->stealthyHeightRatioThresh(),
        generationBar->flatLiftingHeight(), generationBar->flatLandingHeight(),
        generationBar->impactReductionHeight(), generationBar->impactReductionTime());

    interpolator->setToeStepParameters(
        generationBar->toeContactAngle(), generationBar->toeContactTime());

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

    if(targetBodyItem){
        result = generationBar->shapeBodyMotion(
            targetBodyItem, interpolator.get(), bodyMotionItem, putMessages);
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
    isSelectedPoseBeingMoved = false;
    isPoseSelectionChangedByEditing = false;

    if(editConnections.empty()){
        editConnections.add(
            seq->sigPoseInserted().connect(
                [this](PoseSeq::iterator pose, bool isMoving){
                    onPoseInserted(pose, isMoving);
                }));
        editConnections.add(
            seq->sigPoseAboutToBeRemoved().connect(
                [this](PoseSeq::iterator pose, bool isMoving){
                    onPoseAboutToBeRemoved(pose, isMoving);
                }));
        editConnections.add(
            seq->sigPoseAboutToBeModified().connect(
                [this](PoseSeq::iterator pose){
                    onPoseAboutToBeModified(pose);
                }));
        editConnections.add(
            seq->sigPoseModified().connect(
                [this](PoseSeq::iterator pose){
                    onPoseModified(pose);
                }));
    }
}


bool PoseSeqItem::endEditing(bool actuallyModified)
{
    return impl->endEditing(actuallyModified);
}


bool PoseSeqItem::Impl::endEditing(bool actuallyModified)
{
    if(actuallyModified){
        for(auto& it : inserted){
            newHistory.added->insert(newHistory.added->end(), it->time(), it->pose()->clone())
                ->setMaxTransitionTime(it->maxTransitionTime());
        }
        for(auto& it : modified){
            newHistory.added->insert(newHistory.added->end(), it->time(), it->pose()->clone())
                ->setMaxTransitionTime(it->maxTransitionTime());
        }
        if(!newHistory.empty()){
            editHistories.resize(currentHistory);
            editHistories.push_back(newHistory);
            currentHistory = editHistories.size();
            self->suggestFileUpdate();
        }

        // This does not necessarily have to be done here.
        // The interpolation information the following function updates
        // is automatically updated on demand when a interpolation is actually done.
        updateInterpolation();
    
        if(BodyMotionGenerationBar::instance()->isAutoGenerationMode()){
            updateTrajectory(false);
        }
    }
    
    modifyingPoseIter = seq->end();
    inserted.clear();
    modified.clear();
    newHistory.clear();
    editConnections.disconnect();

    isSelectedPoseBeingMoved = false;
    if(isPoseSelectionChangedByEditing){
        self->notifyPoseSelectionChange();
        isPoseSelectionChangedByEditing = false;
    }

    return actuallyModified;
}


void PoseSeqItem::Impl::onPoseInserted(PoseSeq::iterator pose, bool isMoving)
{
    if(isSelectedPoseBeingMoved){
        if(isMoving){
            modified.insert(pose);
            selectPose(pose, false, true);
            isPoseSelectionChangedByEditing = true;
        }
        isSelectedPoseBeingMoved = false;
    }
    inserted.insert(pose);
}


void PoseSeqItem::Impl::onPoseAboutToBeRemoved(PoseSeq::iterator pose, bool isMoving)
{
    if(deselectPose(pose, false)){
        if(isMoving){
            isSelectedPoseBeingMoved = true;
        }
        isPoseSelectionChangedByEditing = true;
    }
    
    if(modified.find(pose) != modified.end()){
        modified.erase(pose);
    }

    if(inserted.find(pose) != inserted.end()){
        inserted.erase(pose);
    } else {
        newHistory.removed->insert(newHistory.removed->end(), pose->time(), pose->pose()->clone())
            ->setMaxTransitionTime(pose->maxTransitionTime());
    }
}


void PoseSeqItem::Impl::onPoseAboutToBeModified(PoseSeq::iterator pose)
{
    modifyingPoseIter = pose;
    preModifiedPose = pose->pose()->clone();
    preModifiedPoseTime = pose->time();
    preModifiedPoseTransitionTime = pose->maxTransitionTime();
}


void PoseSeqItem::Impl::onPoseModified(PoseSeq::iterator pose)
{
    if(pose == modifyingPoseIter){
        if(modified.find(pose) == modified.end()){
            modified.insert(pose);
            auto removed =
                newHistory.removed->insert(
                    newHistory.removed->end(),
                    preModifiedPoseTime,
                    preModifiedPose);
            removed->setMaxTransitionTime(preModifiedPoseTransitionTime);
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


PoseSeq::iterator PoseSeqItem::Impl::removeSameElement(PoseSeq::iterator current, PoseSeq::iterator it)
{
    current = seq->seek(current, it->time());
    while(current->time() == it->time()){
        if(current->pose()->hasSameParts(it->pose())){
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
        auto current = seq->begin();
        for(auto it = added->begin(); it != added->end(); ++it){
            current = removeSameElement(current, it);
        }
        PoseSeqPtr removed = history.removed;
        for(auto it = removed->begin(); it != removed->end(); ++it){
            current = seq->insert(current, it->time(), it->pose()->clone());
            current->setMaxTransitionTime(it->maxTransitionTime());
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
        auto current = seq->begin();
        for(auto it = removed->begin(); it != removed->end(); ++it){
            current = removeSameElement(current, it);
        }
        PoseSeqPtr added = history.added;
        for(auto it = added->begin(); it != added->end(); ++it){
            current = seq->insert(current, it->time(), it->pose()->clone());
            current->setMaxTransitionTime(it->maxTransitionTime());
        }
        editConnections.unblock();
        self->suggestFileUpdate();
        return true;
    }
    return false;
}


bool PoseSeqItem::updatePosesWithBalancedTrajectories(std::ostream& os)
{
    return impl->updatePosesWithBalancedTrajectories(os);
}


bool PoseSeqItem::Impl::updatePosesWithBalancedTrajectories(std::ostream& os)
{
    auto motion = bodyMotionItem->motion();
    auto pseq = motion->positionSeq();
    double length = seq->endingTime();
    
    if(pseq->timeLength() < length){
        os << "Length of the interpolated trajectories is shorter than key pose sequence.";
        return false;
    }
    if(pseq->numLinkPositionsHint() < targetBodyItem->body()->numLinks()){
        os << "Not all link positions are available. Please do interpolate with \"Put all link positions\"";
        return false;
    }

    beginEditing();
    
    for(auto it = seq->begin(); it != seq->end(); ++it){
        if(auto pose = it->get<BodyKeyPose>()){
            seq->beginPoseModification(it);

            int frameIndex = pseq->frameOfTime(it->time());
            auto& frame = pseq->frame(frameIndex);
            int nj = std::min(pose->numJoints(), frame.numJointDisplacements());
            auto displacements = frame.jointDisplacements();
            for(int i=0; i < nj; ++i){
                if(pose->isJointValid(i)){
                    pose->setJointDisplacement(i, displacements[i]);
                }
            }

            for(auto ikLinkIter = pose->ikLinkBegin(); ikLinkIter != pose->ikLinkEnd(); ++ikLinkIter){
                int linkIndex = ikLinkIter->first;
                BodyKeyPose::LinkInfo& info = ikLinkIter->second;
                auto p = frame.linkPosition(linkIndex).translation();
                // only update horizontal position
                info.p()[0] = p[0];
                info.p()[1] = p[1];
            }

            seq->endPoseModification(it);
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
