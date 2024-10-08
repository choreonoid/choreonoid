#include "LeggedBodyBar.h"
#include "BodySelectionManager.h"
#include "BodyItem.h"
#include <cnoid/LeggedBodyHelper>
#include <cnoid/SpinBox>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Format>
#include "gettext.h"

using namespace cnoid;

namespace cnoid {

class LeggedBodyBar::Impl
{
public:
    BodySelectionManager* bodySelectionManager;
    DoubleSpinBox* stanceWidthSpin;

    Impl(LeggedBodyBar* self);
    void applyBodyItemOperation(std::function<void(BodyItem* bodyItem)> func);
    void onCmButtonClicked(BodyItem::PositionType position);
    void moveCm(BodyItem* bodyItem, BodyItem::PositionType position);
    void onZmpButtonClicked(BodyItem::PositionType position);
    void onStanceButtonClicked();
};

}


LeggedBodyBar* LeggedBodyBar::instance()
{
    static LeggedBodyBar* instance = new LeggedBodyBar;
    return instance;
}


LeggedBodyBar::LeggedBodyBar()
    : ToolBar(N_("LeggedBodyBar"))
{
    impl = new Impl(this);
}


LeggedBodyBar::Impl::Impl(LeggedBodyBar* self)
{
    bodySelectionManager = BodySelectionManager::instance();

    ToolButton* button;
    
    button = self->addButton(":/Body/icon/center-cm.svg");
    button->setToolTip(_("Move the center of mass to the position where its projection corresponds to the support feet cener"));
    button->sigClicked().connect([&](){ onCmButtonClicked(BodyItem::HOME_COP); });
    
    button = self->addButton(":/Body/icon/zmp-to-cm.svg");
    button->setToolTip(_("Move the center of mass to fit its projection to ZMP"));
    button->sigClicked().connect([&](){ onCmButtonClicked(BodyItem::ZERO_MOMENT_POINT); });
    
    button = self->addButton(":/Body/icon/cm-to-zmp.svg");
    button->setToolTip(_("Set ZMP to the projection of the center of mass"));
    button->sigClicked().connect([&](){ onZmpButtonClicked(BodyItem::CM_PROJECTION); });

    button = self->addButton(":/Body/icon/right-zmp.svg");
    button->setToolTip(_("Set ZMP under the right foot"));
    button->sigClicked().connect([&](){ onZmpButtonClicked(BodyItem::RIGHT_HOME_COP); });

    button = self->addButton(":/Body/icon/center-zmp.svg");
    button->setToolTip(_("Set ZMP at the center of the feet"));
    button->sigClicked().connect([&](){ onZmpButtonClicked(BodyItem::HOME_COP); });

    button = self->addButton(":/Body/icon/left-zmp.svg");
    button->setToolTip(_("Set ZMP under the left foot"));
    button->sigClicked().connect([&](){ onZmpButtonClicked(BodyItem::LEFT_HOME_COP); });

    self->addSeparator();

    button = self->addButton(":/Body/icon/stancelength.svg");
    button->setToolTip(_("Adjust the width between the feet"));
    button->sigClicked().connect([&](){ onStanceButtonClicked(); });

    stanceWidthSpin = new DoubleSpinBox;
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


void LeggedBodyBar::Impl::applyBodyItemOperation(std::function<void(BodyItem* bodyItem)> func)
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


void LeggedBodyBar::Impl::onCmButtonClicked(BodyItem::PositionType position)
{
    applyBodyItemOperation(
        [this, position](BodyItem* bodyItem){ moveCm(bodyItem, position); });
}


void LeggedBodyBar::Impl::moveCm(BodyItem* bodyItem, BodyItem::PositionType position)
{
    Vector3 c = bodyItem->centerOfMass();
    if(auto p = bodyItem->getParticularPosition(position)){
        c[0] = (*p)[0];
        c[1] = (*p)[1];
    }
    if(!bodyItem->doLegIkToMoveCm(c, true)){
        MessageView::instance()->notify(
            formatR(_("The center of mass of {} cannt be moved to the target position\n"),
                    bodyItem->displayName()));
    }
}


void LeggedBodyBar::Impl::onZmpButtonClicked(BodyItem::PositionType position)
{
    applyBodyItemOperation(
        [this, position](BodyItem* bodyItem){
            if(auto p = bodyItem->getParticularPosition(position)){
                auto legged = getLeggedBodyHelper(bodyItem->body());
                if(legged->isValid()){
                    legged->setZmp(*p, true);
                }
            }
        });
}


void LeggedBodyBar::Impl::onStanceButtonClicked()
{
    applyBodyItemOperation(
        [this](BodyItem* bodyItem){
            bodyItem->setStance(stanceWidthSpin->value());
        });
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
