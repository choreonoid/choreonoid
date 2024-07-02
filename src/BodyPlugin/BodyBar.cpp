#include "BodyBar.h"
#include "BodyItem.h"
#include "BodyPoseItem.h"
#include "BodyPoseListItem.h"
#include "BodySelectionManager.h"
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Format>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyBar::Impl
{
public:
    BodySelectionManager* bodySelectionManager;

    Impl(BodyBar* self);
    void applyBodyItemOperation(std::function<void(BodyItem* bodyItem)> func);
    void onOriginButtonClicked();
    void onPoseButtonClicked(BodyItem::PresetPoseID id);
    void onPoseRecButtonClicked();
    void onPoseUpdateButtonClicked();
    void onPoseRecallButtonClicked();
    void onSymmetricCopyButtonClicked(int direction, bool doMirrorCopy);
    void doSymmetricCopy(BodyItem* bodyItem, int direction, bool doMirrorCopy);
};

}


BodyBar* BodyBar::instance()
{
    static BodyBar* instance = new BodyBar;
    return instance;
}


BodyBar::BodyBar()
    : ToolBar(N_("BodyBar"))
{
    impl = new Impl(this);
}


BodyBar::Impl::Impl(BodyBar* self)
{
    ToolButton* button;
    
    button = self->addButton(":/Body/icon/origin.svg");
    button->setToolTip(_("Move the selected bodies to the origin"));
    button->sigClicked().connect([this](){ onOriginButtonClicked(); });

    button = self->addButton(":/Body/icon/initialpose.svg");
    button->setToolTip(_("Set the preset initial pose to the selected bodies"));
    button->sigClicked().connect([this](){ onPoseButtonClicked(BodyItem::INITIAL_POSE); });

    button = self->addButton(":/Body/icon/stdpose.svg");
    button->setToolTip(_("Set the preset standard pose to the selected bodies"));
    button->sigClicked().connect([this](){ onPoseButtonClicked(BodyItem::STANDARD_POSE); });

    button = self->addButton(":/Body/icon/poserec.svg");
    button->setToolTip(_("Record pose"));
    button->sigClicked().connect([this](){ onPoseRecButtonClicked(); });

    button = self->addButton(":/Body/icon/storepose.svg");
    button->setToolTip(_("Update recoreded pose"));
    button->sigClicked().connect([this](){ onPoseUpdateButtonClicked(); });

    button = self->addButton(":/Body/icon/restorepose.svg");
    button->setToolTip(_("Recall recorded pose"));
    button->sigClicked().connect([this](){ onPoseRecallButtonClicked(); });
    
    button = self->addButton(":/Body/icon/right-to-left.svg");
    button->setToolTip(_("Copy the right side pose to the left side"));
    button->sigClicked().connect([this](){ onSymmetricCopyButtonClicked(1, false); });

    button = self->addButton(":/Body/icon/flip.svg");
    button->setToolTip(_("Mirror copy"));
    button->sigClicked().connect([this](){ onSymmetricCopyButtonClicked(0, true); });

    button = self->addButton(":/Body/icon/left-to-right.svg");
    button->setToolTip(_("Copy the left side pose to the right side"));
    button->sigClicked().connect([this](){ onSymmetricCopyButtonClicked(0, false); });

    bodySelectionManager = BodySelectionManager::instance();
}


BodyBar::~BodyBar()
{
    delete impl;
}


void BodyBar::Impl::applyBodyItemOperation(std::function<void(BodyItem* bodyItem)> func)
{
    auto& selected = bodySelectionManager->selectedBodyItems();
    if(!selected.empty()){
        for(auto& bodyItem: selected){
            func(bodyItem);
        }
    } else if(auto current = bodySelectionManager->currentBodyItem()){
        func(current);
    }
}


void BodyBar::Impl::onOriginButtonClicked()
{
    applyBodyItemOperation([](BodyItem* bodyItem){ bodyItem->moveToOrigin(); });
}


void BodyBar::Impl::onPoseButtonClicked(BodyItem::PresetPoseID id)
{
    applyBodyItemOperation([id](BodyItem* bodyItem){ bodyItem->setPresetPose(id); });
}


void BodyBar::Impl::onPoseRecButtonClicked()
{
    auto selected = RootItem::instance()->selectedItems();

    ItemList<BodyPoseListItem> poseListItems(selected);
    ItemList<BodyItem> bodyItems(selected);
    if(bodyItems.empty() && poseListItems.empty()){
        if(auto bodyItem = bodySelectionManager->currentBodyItem()){
            bodyItems.push_back(bodyItem);
        }
    }

    bool canceled = false;

    for(auto& bodyItem : bodyItems){
        if(auto poseListItem = bodyItem->findItem<BodyPoseListItem>()){
            auto found = std::find(poseListItems.begin(), poseListItems.end(), poseListItem);
            if(found == poseListItems.end()){
                poseListItems.push_back(poseListItem);
            }
        } else {
            bool doCreate = showConfirmDialog(
                _("Pose list item creation"),
                formatR(_("Do you want to create a pose list item for {0}? "
                          "The pose list is necessary for recording poses."),
                        bodyItem->displayName()));
            if(doCreate){
                auto poseListItem = new BodyPoseListItem;
                bodyItem->addChildItem(poseListItem);
                poseListItems.push_back(poseListItem);
            } else {
                canceled = true;
            }
        }
    }

    if(!poseListItems.empty()){
        for(auto& poseListItem : poseListItems){
            poseListItem->recordCurrentPose();
        }
    } else if(!canceled){
        showWarningDialog(
            _("Unable to record a pose as a pose item because no target body item or pose list item is specified."));
    }
}


void BodyBar::Impl::onPoseUpdateButtonClicked()
{
    ItemList<BodyPoseItem> poseItems(RootItem::instance()->selectedItems());

    if(poseItems.empty()){

    } else {

    }
}


void BodyBar::Impl::onPoseRecallButtonClicked()
{
    ItemList<BodyPoseItem> poseItems(RootItem::instance()->selectedItems());

    if(poseItems.empty()){

    } else {
        for(auto& poseItem : poseItems){
            poseItem->applyBodyPose();
        }
    }
}


void BodyBar::Impl::onSymmetricCopyButtonClicked(int direction, bool doMirrorCopy)
{
    applyBodyItemOperation(
        [this, direction, doMirrorCopy](BodyItem* bodyItem){
            doSymmetricCopy(bodyItem, direction, doMirrorCopy);
        });
}


void BodyBar::Impl::doSymmetricCopy(BodyItem* bodyItem, int direction, bool doMirrorCopy)
{
    const Listing& slinks =
        *bodyItem->body()->info()->findListing({ "symmetric_joints", "symmetricJoints" });
    if(slinks.isValid() && !slinks.empty()){
        int from = direction;
        int to = 1 - direction;
        BodyPtr body = bodyItem->body();
        for(int j=0; j < slinks.size(); ++j){
            const Listing& linkPair = *slinks[j].toListing();
            if(linkPair.size() == 1 && doMirrorCopy){
                Link* link = body->link(linkPair[0].toString());
                if(link){
                    link->q() = -link->q();
                }
            } else if(linkPair.size() >= 2){
                Link* link1 = body->link(linkPair[from].toString());
                Link* link2 = body->link(linkPair[to].toString());
                if(link1 && link2){
                    double sign = 1.0;
                    if(linkPair.size() >= 3){
                        sign = linkPair[2].toDouble();
                    }
                    if(doMirrorCopy){
                        double q1 = link1->q();
                        link1->q() = sign * link2->q();
                        link2->q() = sign * q1;
                    } else {
                        link2->q() = sign * link1->q();
                    }
                }
            }
        }
        bodyItem->calcForwardKinematics();
        bodyItem->notifyKinematicStateUpdate();
    }
}
