/**
   @author Shin'ichiro Nakaoka
*/

#include "KinematicsBar.h"
#include <cnoid/MainWindow>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/SpinBox>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/Dialog>
#include <cnoid/LazyCaller>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class KinematicsBarSetupDialog : public Dialog
{
public:
    DoubleSpinBox snapDistanceSpin;
    SpinBox snapAngleSpin;
    DoubleSpinBox penetrationBlockDepthSpin;
    CheckBox lazyCollisionDetectionModeCheck;
    PushButton okButton;
        
    KinematicsBarSetupDialog();
    void storeState(Archive& archive);
    void restoreState(const Archive& archive);
};

}

namespace cnoid {

class KinematicsBarImpl
{
public:
    ToolButton* fkToggle;
    ToolButton* presetToggle;
    ToolButton* ikToggle;
    Signal<void()> sigKinematicsModeChanged;

    ToolButton* draggerToggle;
    ToolButton* footSnapToggle;
    ToolButton* jointPositionLimitToggle;
    ToolButton* penetrationBlockToggle;

    ToolButton* collisionLinkHighlightToggle;
    int collisionDetectionPriority;
    Signal<void()> sigCollisionVisualizationChanged;            

    KinematicsBarSetupDialog* setup;

    KinematicsBarImpl(KinematicsBar* self);
    void onCollisionVisualizationChanged();
    void onLazyCollisionDetectionModeToggled();
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}


KinematicsBar* KinematicsBar::instance()
{
    static KinematicsBar* instance = new KinematicsBar();
    return instance;
}


KinematicsBar::KinematicsBar()
    : ToolBar(N_("KinematicsBar"))
{
    impl = new KinematicsBarImpl(this);
}


KinematicsBarImpl::KinematicsBarImpl(KinematicsBar* self)
{
    setup = new KinematicsBarSetupDialog();
    
    fkToggle = self->addToggleButton(QIcon(":/Body/icon/fk.svg"), _("Enable forward kinematics"));
    fkToggle->setChecked(true);
    fkToggle->sigToggled().connect([&](bool on){ sigKinematicsModeChanged(); });
    presetToggle = self->addToggleButton(QIcon(":/Body/icon/fkik.svg"), _("Use preset Kinematics"));
    presetToggle->setChecked(true);
    presetToggle->sigToggled().connect([&](bool on){ sigKinematicsModeChanged(); });
    ikToggle = self->addToggleButton(QIcon(":/Body/icon/ik.svg"), _("Enable inverse kinematics"));
    ikToggle->setChecked(true);
    ikToggle->sigToggled().connect([&](bool on){ sigKinematicsModeChanged(); });
    self->addSpacing();

    draggerToggle = self->addToggleButton(QIcon(":/Body/icon/rotation.svg"), _("Enable link orientation editing"));
    draggerToggle->setChecked(true);

    /*
      footSnapToggle = self->addToggleButton(xpmfootsnapon, _("Snap foot to the floor"));
      footSnapToggle->setChecked(true);
    */
    /*
      jointPositionLimitToggle = self->addToggleButton(
      xpmlimitangle, _("Limit joint positions within the movable range specifications"));
      jointPositionLimitToggle->setChecked(true);
    */
    
    penetrationBlockToggle = self->addToggleButton(QIcon(":/Body/icon/block.svg"), _("Penetration block mode"));
    penetrationBlockToggle->setChecked(false);

    collisionLinkHighlightToggle = self->addToggleButton(QIcon(":/Body/icon/collisionoutline.svg"), _("Highlight colliding links"));
    collisionLinkHighlightToggle->setChecked(false);
    collisionLinkHighlightToggle->sigToggled().connect(
        std::bind(&KinematicsBarImpl::onCollisionVisualizationChanged, this));

    self->addButton(QIcon(":/Base/icon/setup.svg"))
        ->sigClicked().connect(
            std::bind(&KinematicsBarSetupDialog::show, setup));
    
    setup->lazyCollisionDetectionModeCheck.sigToggled().connect(
        std::bind(&KinematicsBarImpl::onLazyCollisionDetectionModeToggled, this));
    
    onLazyCollisionDetectionModeToggled();
}


KinematicsBar::~KinematicsBar()
{
    delete impl;
}


int KinematicsBar::mode() const
{
    if(impl->presetToggle->isChecked()){
        return PresetKinematics;
    } else if(impl->fkToggle->isChecked()){
        if(impl->ikToggle->isChecked()){
            return PresetKinematics;
        } else {
            return ForwardKinematics;
        }
    } else if(impl->ikToggle->isChecked()){
        return InverseKinematics;
    }
    return NoKinematics;
}


bool KinematicsBar::isForwardKinematicsEnabled() const
{
    return impl->fkToggle->isChecked();
}


bool KinematicsBar::isInverseKinematicsEnabled() const
{
    return impl->ikToggle->isChecked();
}


SignalProxy<void()> KinematicsBar::sigKinematicsModeChanged()
{
    return impl->sigKinematicsModeChanged;
}
    

bool KinematicsBar::isPositionDraggerEnabled() const
{
    return impl->draggerToggle->isChecked();
}


bool KinematicsBar::isFootSnapMode() const
{
    return impl->footSnapToggle->isChecked();
}


void KinematicsBar::getSnapThresholds(double& distance, double& angle) const
{
    distance = impl->setup->snapDistanceSpin.value();
    angle = radian(impl->setup->snapAngleSpin.value());
}


bool KinematicsBar::isJointPositionLimitMode() const
{
    return impl->jointPositionLimitToggle->isChecked();
}


bool KinematicsBar::isPenetrationBlockMode() const
{
    return impl->penetrationBlockToggle->isChecked();
}


double KinematicsBar::penetrationBlockDepth() const
{
    return impl->setup->penetrationBlockDepthSpin.value();
}


bool KinematicsBar::isCollisionLinkHighlihtMode() const
{
    return impl->collisionLinkHighlightToggle->isChecked();
}


int KinematicsBar::collisionDetectionPriority() const
{
    return impl->collisionDetectionPriority;
}


SignalProxy<void()> KinematicsBar::sigCollisionVisualizationChanged()
{
    return impl->sigCollisionVisualizationChanged;
}


void KinematicsBarImpl::onCollisionVisualizationChanged()
{
    sigCollisionVisualizationChanged();
}


void KinematicsBarImpl::onLazyCollisionDetectionModeToggled()
{
    if(setup->lazyCollisionDetectionModeCheck.isChecked()){
        collisionDetectionPriority = LazyCaller::PRIORITY_NORMAL;
    } else {
        collisionDetectionPriority = LazyCaller::PRIORITY_HIGH;
    }
}


bool KinematicsBar::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool KinematicsBarImpl::storeState(Archive& archive)
{
    archive.write("forward_kinematics", fkToggle->isChecked());
    archive.write("inverse_kinematics", ikToggle->isChecked());
    archive.write("preset_kinematics", presetToggle->isChecked());
    archive.write("enablePositionDragger", draggerToggle->isChecked());
    archive.write("penetrationBlock", penetrationBlockToggle->isChecked());
    //archive.write("footSnap", footSnapToggle->isChecked());
    archive.write("collisionLinkHighlight", collisionLinkHighlightToggle->isChecked());
    setup->storeState(archive);
    return true;
}


bool KinematicsBar::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool KinematicsBarImpl::restoreState(const Archive& archive)
{
    bool modeChanged = false;
    bool on;
    if(archive.read("preset_kinematics", on)){
        if(on != presetToggle->isChecked()){
            presetToggle->blockSignals(true);
            presetToggle->setChecked(on);
            presetToggle->blockSignals(false);
            modeChanged = true;
        }
    }
    if(archive.read("forward_kinematics", on)){
        if(on != fkToggle->isChecked()){
            fkToggle->blockSignals(true);
            fkToggle->setChecked(on);
            fkToggle->blockSignals(false);
            modeChanged = true;
        }
    }
    if(archive.read("inverse_kinematics", on)){
        if(on != ikToggle->isChecked()){
            ikToggle->blockSignals(true);
            ikToggle->setChecked(on);
            ikToggle->blockSignals(false);
            modeChanged = true;
        }
    }
    if(modeChanged){
        sigKinematicsModeChanged();
    }
    
    draggerToggle->setChecked(archive.get("enablePositionDragger", draggerToggle->isChecked()));
    penetrationBlockToggle->setChecked(archive.get("penetrationBlock", penetrationBlockToggle->isChecked()));
    //footSnapToggle->setChecked(archive.get("footSnap", footSnapToggle->isChecked()));
    collisionLinkHighlightToggle->setChecked(archive.get("collisionLinkHighlight", collisionLinkHighlightToggle->isChecked()));
    setup->restoreState(archive);
    
    return true;
}


namespace {

KinematicsBarSetupDialog::KinematicsBarSetupDialog()
{
    setWindowTitle(_("Kinematics Operation Setup"));
        
    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);
   
    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Snap thresholds:")));
    hbox->addSpacing(10);
    
    hbox->addWidget(new QLabel(_("distance")));
    snapDistanceSpin.setAlignment(Qt::AlignCenter);
    snapDistanceSpin.setDecimals(3);
    snapDistanceSpin.setRange(0.0, 0.999);
    snapDistanceSpin.setSingleStep(0.001);
    snapDistanceSpin.setValue(0.025);
    hbox->addWidget(&snapDistanceSpin);
    hbox->addWidget(new QLabel(_("[m]")));
    
    hbox->addSpacing(5);
    hbox->addWidget(new QLabel(_("angle")));
    snapAngleSpin.setAlignment(Qt::AlignCenter);
    snapAngleSpin.setRange(0, 90);
    snapAngleSpin.setValue(30);
    hbox->addWidget(&snapAngleSpin);
    hbox->addWidget(new QLabel(_("[deg]")));
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Penetration block depth")));
    penetrationBlockDepthSpin.setAlignment(Qt::AlignCenter);
    penetrationBlockDepthSpin.setDecimals(4);
    penetrationBlockDepthSpin.setRange(0.0, 0.0099);
    penetrationBlockDepthSpin.setSingleStep(0.0001);
    penetrationBlockDepthSpin.setValue(0.0005);
    hbox->addWidget(&penetrationBlockDepthSpin);
    hbox->addWidget(new QLabel(_("[m]")));
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout();
    lazyCollisionDetectionModeCheck.setText(_("Lazy collision detection mode"));
    lazyCollisionDetectionModeCheck.setChecked(true);
    hbox->addWidget(&lazyCollisionDetectionModeCheck);
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout();
    okButton.setText(_("OK"));
    okButton.setDefault(true);
    hbox->addWidget(&okButton);
    vbox->addLayout(hbox);
}


void KinematicsBarSetupDialog::storeState(Archive& archive)
{
    archive.write("snapDistance", snapDistanceSpin.value());
    archive.write("penetrationBlockDepth", penetrationBlockDepthSpin.value());
    archive.write("lazyCollisionDetectionMode", lazyCollisionDetectionModeCheck.isChecked());
}


void KinematicsBarSetupDialog::restoreState(const Archive& archive)
{
    snapDistanceSpin.setValue(archive.get("snapDistance", snapDistanceSpin.value()));
    penetrationBlockDepthSpin.setValue(archive.get("penetrationBlockDepth", penetrationBlockDepthSpin.value()));
    lazyCollisionDetectionModeCheck.setChecked(archive.get("lazyCollisionDetectionMode", lazyCollisionDetectionModeCheck.isChecked()));
}

}
