/**
   @author Shin'ichiro Nakaoka
*/

#include "BodyBar.h"
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include "gettext.h"

using namespace cnoid;

BodyBar* BodyBar::instance()
{
    static BodyBar* instance = new BodyBar();
    return instance;
}


BodyBar::BodyBar()
    : ToolBar(N_("BodyBar")),
      mes(*MessageView::mainInstance())
{
    using boost::bind;
    
    setVisibleByDefault(true);

    addButton(QIcon(":/Body/icons/storepose.png"), _("Memory the current pose"))
        ->sigClicked().connect(bind(&BodyBar::onCopyButtonClicked, this));

    addButton(QIcon(":/Body/icons/restorepose.png"), _("Recall the memorized pose"))
        ->sigClicked().connect(bind(&BodyBar::onPasteButtonClicked, this));
    
    addButton(QIcon(":/Body/icons/origin.png"), _("Move the selected bodies to the origin"))
        ->sigClicked().connect(bind(&BodyBar::onOriginButtonClicked, this));

    addButton(QIcon(":/Body/icons/initialpose.png"), _("Set the preset initial pose to the selected bodies"))
        ->sigClicked().connect(bind(&BodyBar::onPoseButtonClicked, this, BodyItem::INITIAL_POSE));

    addButton(QIcon(":/Body/icons/stdpose.png"), _("Set the preset standard pose to the selected bodies"))
        ->sigClicked().connect(bind(&BodyBar::onPoseButtonClicked, this, BodyItem::STANDARD_POSE));

    addSeparator();

    addButton(QIcon(":/Body/icons/right-to-left.png"), _("Copy the right side pose to the left side"))
        ->sigClicked().connect(bind(&BodyBar::onSymmetricCopyButtonClicked, this, 1, false));

    addButton(QIcon(":/Body/icons/flip.png"), _("Mirror copy"))
        ->sigClicked().connect(bind(&BodyBar::onSymmetricCopyButtonClicked, this, 0, true));

    addButton(QIcon(":/Body/icons/left-to-right.png"), _("Copy the left side pose to the right side"))
        ->sigClicked().connect(bind(&BodyBar::onSymmetricCopyButtonClicked, this, 0, false));

    
    addSeparator();

    addButton(QIcon(":/Body/icons/center-cm.png"), _("Move the center of mass to the position where its projection corresponds to the support feet cener"))->
        sigClicked().connect(bind(&BodyBar::moveCM, this, BodyItem::HOME_COP));
    
    addButton(QIcon(":/Body/icons/zmp-to-cm.png"), _("Move the center of mass to fit its projection to ZMP"))->
        sigClicked().connect(bind(&BodyBar::moveCM, this, BodyItem::ZERO_MOMENT_POINT));
    
    addSeparator();

    addButton(QIcon(":/Body/icons/cm-to-zmp.png"), _("Set ZMP to the projection of the center of mass"))
        ->sigClicked().connect(bind(&BodyBar::setZmp, this, BodyItem::CM_PROJECTION));

    addButton(QIcon(":/Body/icons/right-zmp"), _("Set ZMP under the right foot"))
        ->sigClicked().connect(bind(&BodyBar::setZmp, this, BodyItem::RIGHT_HOME_COP));

    addButton(QIcon(":/Body/icons/center-zmp.png"), _("Set ZMP at the center of the feet"))
        ->sigClicked().connect(bind(&BodyBar::setZmp, this, BodyItem::HOME_COP));

    addButton(QIcon(":/Body/icons/left-zmp.png"), _("Set ZMP under the left foot"))
        ->sigClicked().connect(bind(&BodyBar::setZmp, this, BodyItem::LEFT_HOME_COP));

    addSeparator();

    addButton(QIcon(":/Body/icons/stancelength.png"), _("Adjust the width between the feet"))
        ->sigClicked().connect(bind(&BodyBar::setStance, this));

    stanceWidthSpin = new DoubleSpinBox();
    stanceWidthSpin->setAlignment(Qt::AlignCenter);
    stanceWidthSpin->setToolTip(_("Width between the feet [m]"));
    stanceWidthSpin->setDecimals(4);
    stanceWidthSpin->setRange(0.0001, 9.9999);
    stanceWidthSpin->setSingleStep(0.001);
    stanceWidthSpin->setValue(0.15);
    addWidget(stanceWidthSpin);

    connectionOfItemSelectionChanged = 
        ItemTreeView::mainInstance()->sigSelectionChanged().connect(
            bind(&BodyBar::onItemSelectionChanged, this, _1));
}


BodyBar::~BodyBar()
{
    connectionOfItemSelectionChanged.disconnect();
    connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
}


/**
   \todo ItemTreeView::sigSelectionChanged() should be emitted
   after the final selection state has been determined.
*/
bool BodyBar::makeSingleSelection(BodyItemPtr bodyItem)
{
    ItemTreeView* tree = ItemTreeView::mainInstance()->mainInstance();

    ItemList<BodyItem> prevSelected = selectedBodyItems_;

    for(size_t i=0; i < prevSelected.size(); ++i){
        BodyItem* item = prevSelected.get(i);
        if(item != bodyItem && tree->isItemSelected(item)){
            tree->selectItem(item, false);
        }
    }

    bool selected = tree->isItemSelected(bodyItem);
    if(!selected){
        selected = tree->selectItem(bodyItem, true);
    }
    return selected;
}


void BodyBar::onItemSelectionChanged(const ItemList<BodyItem>& bodyItems)
{
    bool selectedBodyItemsChanged = false;
    
    if(selectedBodyItems_ != bodyItems){
        selectedBodyItems_ = bodyItems;
        selectedBodyItemsChanged = true;
    }

    BodyItemPtr firstItem = bodyItems.toSingle();

    if(firstItem && firstItem != currentBodyItem_){
        currentBodyItem_ = firstItem;
        connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
        connectionOfCurrentBodyItemDetachedFromRoot = currentBodyItem_->sigDetachedFromRoot().connect(
            boost::bind(&BodyBar::onBodyItemDetachedFromRoot, this));
        sigCurrentBodyItemChanged_(currentBodyItem_.get());
    }

    if(selectedBodyItemsChanged){
        sigBodyItemSelectionChanged_(selectedBodyItems_);
    }

    targetBodyItems.clear();
    if(selectedBodyItems_.empty()){
        if(currentBodyItem_){
            targetBodyItems.push_back(currentBodyItem_);
        }
    } else {
        targetBodyItems = selectedBodyItems_;
    }
}


void BodyBar::onCopyButtonClicked()
{
    if(currentBodyItem_){
        currentBodyItem_->copyKinematicState();
    }
}


void BodyBar::onPasteButtonClicked()
{
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        targetBodyItems[i]->pasteKinematicState();
    }
}


void BodyBar::onBodyItemDetachedFromRoot()
{
    currentBodyItem_ = 0;
    connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
    sigCurrentBodyItemChanged_(0);
}


void BodyBar::onOriginButtonClicked()
{
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        targetBodyItems[i]->moveToOrigin();
    }
}


void BodyBar::onPoseButtonClicked(BodyItem::PresetPoseID id)
{
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        targetBodyItems[i]->setPresetPose(id);
    }
}


void BodyBar::onSymmetricCopyButtonClicked(int direction, bool doMirrorCopy)
{
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        const Listing& slinks = *targetBodyItems[i]->body()->info()->findListing("symmetricJoints");
        if(slinks.isValid() && !slinks.empty()){
            targetBodyItems[i]->beginKinematicStateEdit();
            int from = direction;
            int to = 1 - direction;
            BodyPtr body = targetBodyItems[i]->body();
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
            targetBodyItems[i]->notifyKinematicStateChange(true);
            targetBodyItems[i]->acceptKinematicStateEdit();
        }
    }
}


void BodyBar::moveCM(BodyItem::PositionType position)
{
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        BodyItem* bodyItem = targetBodyItems.get(i);
        Vector3 c = bodyItem->centerOfMass();
        boost::optional<Vector3> p = bodyItem->getParticularPosition(position);
        if(p){
            c[0] = (*p)[0];
            c[1] = (*p)[1];
        }
        if(!bodyItem->doLegIkToMoveCm(c, true)){
            static boost::format f(_("The center of mass of %1% cannt be moved to the target position\n"));
            mes.notify(str(f % bodyItem->name()));
        }
    }
}


void BodyBar::setZmp(BodyItem::PositionType position)
{
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        boost::optional<Vector3> p = targetBodyItems[i]->getParticularPosition(position);
        if(p){
            targetBodyItems[i]->editZmp(*p);
        }
    }
}


void BodyBar::setStance()
{
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        targetBodyItems[i]->setStance(stanceWidthSpin->value());
    }
}


bool BodyBar::storeState(Archive& archive)
{
    if(currentBodyItem_){
        archive.writeItemId("current", currentBodyItem_);
    }
    archive.write("stanceWidth", stanceWidthSpin->value());

    return true;
}


bool BodyBar::restoreState(const Archive& archive)
{
    stanceWidthSpin->setValue(archive.get("stanceWidth", stanceWidthSpin->value()));
    
    if(!currentBodyItem_){
        currentBodyItem_ = archive.findItem<BodyItem>("current");
        if(currentBodyItem_){
            if(targetBodyItems.empty()){
                targetBodyItems.push_back(currentBodyItem_);
            }
            sigCurrentBodyItemChanged_(currentBodyItem_.get());
        }
    }
    return true;
}
