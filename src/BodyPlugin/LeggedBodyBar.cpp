/**
   @author Shin'ichiro Nakaoka
*/

#include "LeggedBodyBar.h"
#include "BodySelectionManager.h"
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
    BodySelectionManager* bodySelectionManager;
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
    bodySelectionManager = BodySelectionManager::instance();
    
    self->addButton(QIcon(":/Body/icons/center-cm.svg"), _("Move the center of mass to the position where its projection corresponds to the support feet cener"))->
        sigClicked().connect(std::bind(&LeggedBodyBarImpl::moveCM, this, BodyItem::HOME_COP));
    
    self->addButton(QIcon(":/Body/icons/zmp-to-cm.svg"), _("Move the center of mass to fit its projection to ZMP"))->
        sigClicked().connect(std::bind(&LeggedBodyBarImpl::moveCM, this, BodyItem::ZERO_MOMENT_POINT));
    
    self->addButton(QIcon(":/Body/icons/cm-to-zmp.svg"), _("Set ZMP to the projection of the center of mass"))
        ->sigClicked().connect(std::bind(&LeggedBodyBarImpl::setZmp, this, BodyItem::CM_PROJECTION));

    self->addButton(QIcon(":/Body/icons/right-zmp.svg"), _("Set ZMP under the right foot"))
        ->sigClicked().connect(std::bind(&LeggedBodyBarImpl::setZmp, this, BodyItem::RIGHT_HOME_COP));

    self->addButton(QIcon(":/Body/icons/center-zmp.svg"), _("Set ZMP at the center of the feet"))
        ->sigClicked().connect(std::bind(&LeggedBodyBarImpl::setZmp, this, BodyItem::HOME_COP));

    self->addButton(QIcon(":/Body/icons/left-zmp.svg"), _("Set ZMP under the left foot"))
        ->sigClicked().connect(std::bind(&LeggedBodyBarImpl::setZmp, this, BodyItem::LEFT_HOME_COP));

    self->addSeparator();

    self->addButton(QIcon(":/Body/icons/stancelength.svg"), _("Adjust the width between the feet"))
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
    for(auto& bodyItem : bodySelectionManager->selectedBodyItems()){
        Vector3 c = bodyItem->centerOfMass();
        if(auto p = bodyItem->getParticularPosition(position)){
            c[0] = (*p)[0];
            c[1] = (*p)[1];
        }
        if(!bodyItem->doLegIkToMoveCm(c, true)){
            MessageView::instance()->notify(
                fmt::format(_("The center of mass of {} cannt be moved to the target position\n"),
                            bodyItem->displayName()));
        }
    }
}


void LeggedBodyBarImpl::setZmp(BodyItem::PositionType position)
{
    for(auto& bodyItem : bodySelectionManager->selectedBodyItems()){
        if(auto p = bodyItem->getParticularPosition(position)){
            bodyItem->editZmp(*p);
        }
    }
}


void LeggedBodyBarImpl::setStance()
{
    for(auto& bodyItem : bodySelectionManager->selectedBodyItems()){
        bodyItem->setStance(stanceWidthSpin->value());
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
