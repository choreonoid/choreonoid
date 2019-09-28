/**
   @author Shin'ichiro Nakaoka
*/

#include "LeggedBodyBar.h"
#include "BodyBar.h"
#include "BodyItem.h"
#include <cnoid/SpinBox>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <fmt/format.h>
#include "gettext.h"

using namespace cnoid;

namespace cnoid {

class LeggedBodyBarImpl
{
public:
    BodyBar* bodyBar;
    DoubleSpinBox* stanceWidthSpin;

    LeggedBodyBarImpl(LeggedBodyBar* self);
    void moveCM(BodyItem::PositionType position);
    void setZmp(BodyItem::PositionType position);
    void setStance();
};

}


LeggedBodyBar* LeggedBodyBar::instance()
{
    static LeggedBodyBar* instance = new LeggedBodyBar();
    return instance;
}


LeggedBodyBar::LeggedBodyBar()
    : ToolBar(N_("LeggedBodyBar"))
{
    impl = new LeggedBodyBarImpl(this);
}


LeggedBodyBarImpl::LeggedBodyBarImpl(LeggedBodyBar* self)
{
    bodyBar = BodyBar::instance();
    
    self->addButton(QIcon(":/Body/icons/center-cm.png"), _("Move the center of mass to the position where its projection corresponds to the support feet cener"))->
        sigClicked().connect(std::bind(&LeggedBodyBarImpl::moveCM, this, BodyItem::HOME_COP));
    
    self->addButton(QIcon(":/Body/icons/zmp-to-cm.png"), _("Move the center of mass to fit its projection to ZMP"))->
        sigClicked().connect(std::bind(&LeggedBodyBarImpl::moveCM, this, BodyItem::ZERO_MOMENT_POINT));
    
    self->addButton(QIcon(":/Body/icons/cm-to-zmp.png"), _("Set ZMP to the projection of the center of mass"))
        ->sigClicked().connect(std::bind(&LeggedBodyBarImpl::setZmp, this, BodyItem::CM_PROJECTION));

    self->addButton(QIcon(":/Body/icons/right-zmp"), _("Set ZMP under the right foot"))
        ->sigClicked().connect(std::bind(&LeggedBodyBarImpl::setZmp, this, BodyItem::RIGHT_HOME_COP));

    self->addButton(QIcon(":/Body/icons/center-zmp.png"), _("Set ZMP at the center of the feet"))
        ->sigClicked().connect(std::bind(&LeggedBodyBarImpl::setZmp, this, BodyItem::HOME_COP));

    self->addButton(QIcon(":/Body/icons/left-zmp.png"), _("Set ZMP under the left foot"))
        ->sigClicked().connect(std::bind(&LeggedBodyBarImpl::setZmp, this, BodyItem::LEFT_HOME_COP));

    self->addSeparator();

    self->addButton(QIcon(":/Body/icons/stancelength.png"), _("Adjust the width between the feet"))
        ->sigClicked().connect(std::bind(&LeggedBodyBarImpl::setStance, this));

    stanceWidthSpin = new DoubleSpinBox();
    stanceWidthSpin->setAlignment(Qt::AlignCenter);
    stanceWidthSpin->setToolTip(_("Width between the feet [m]"));
    stanceWidthSpin->setDecimals(4);
    stanceWidthSpin->setRange(0.0001, 9.9999);
    stanceWidthSpin->setSingleStep(0.001);
    stanceWidthSpin->setValue(0.15);
    self->addWidget(stanceWidthSpin);
}


LeggedBodyBar::~LeggedBodyBar()
{
    delete impl;
}


void LeggedBodyBarImpl::moveCM(BodyItem::PositionType position)
{
    const ItemList<BodyItem>& targetBodyItems = bodyBar->targetBodyItems();
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        BodyItem* bodyItem = targetBodyItems[i];
        Vector3 c = bodyItem->centerOfMass();
        auto p = bodyItem->getParticularPosition(position);
        if(p){
            c[0] = (*p)[0];
            c[1] = (*p)[1];
        }
        if(!bodyItem->doLegIkToMoveCm(c, true)){
            MessageView::instance()->notify(
                fmt::format(_("The center of mass of {} cannt be moved to the target position\n"),
                            bodyItem->name()));
        }
    }
}


void LeggedBodyBarImpl::setZmp(BodyItem::PositionType position)
{
    const ItemList<BodyItem>& targetBodyItems = bodyBar->targetBodyItems();
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        auto p = targetBodyItems[i]->getParticularPosition(position);
        if(p){
            targetBodyItems[i]->editZmp(*p);
        }
    }
}


void LeggedBodyBarImpl::setStance()
{
    const ItemList<BodyItem>& targetBodyItems = bodyBar->targetBodyItems();    
    for(size_t i=0; i < targetBodyItems.size(); ++i){
        targetBodyItems[i]->setStance(stanceWidthSpin->value());
    }
}


bool LeggedBodyBar::storeState(Archive& archive)
{
    archive.write("stanceWidth", impl->stanceWidthSpin->value());
    return true;
}


bool LeggedBodyBar::restoreState(const Archive& archive)
{
    impl->stanceWidthSpin->setValue(archive.get("stanceWidth", impl->stanceWidthSpin->value()));
    return true;
}
