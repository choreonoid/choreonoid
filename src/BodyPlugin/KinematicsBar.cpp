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

const char* modeSymbol[] = { "AUTO", "FK", "IK" };

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
    ToolButton* autoModeRadio;
    ToolButton* fkModeRadio;
    ToolButton* ikModeRadio;

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
    self->setVisibleByDefault(true);
    
    setup = new KinematicsBarSetupDialog();
    
    fkModeRadio = self->addRadioButton(QIcon(":/Body/icons/fk.png"), _("Forward kinematics mode"));
    autoModeRadio = self->addRadioButton(QIcon(":/Body/icons/fkik.png"), _("Preset kinematics mode"));
    ikModeRadio = self->addRadioButton(QIcon(":/Body/icons/ik.png"), _("Inverse kinematics mode"));
    autoModeRadio->setChecked(true);
    self->addSeparator(2);

    draggerToggle = self->addToggleButton(QIcon(":/Body/icons/rotation.png"), _("Enable link orientation editing"));
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
    
    penetrationBlockToggle = self->addToggleButton(QIcon(":/Body/icons/block.png"), _("Penetration block mode"));
    penetrationBlockToggle->setChecked(false);

    collisionLinkHighlightToggle = self->addToggleButton(QIcon(":/Body/icons/collisionoutline.png"), _("Highlight colliding links"));
    collisionLinkHighlightToggle->setChecked(false);
    collisionLinkHighlightToggle->sigToggled().connect(
        std::bind(&KinematicsBarImpl::onCollisionVisualizationChanged, this));

    self->addButton(QIcon(":/Base/icons/setup.png"))
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
    if(impl->fkModeRadio->isChecked()){
        return FK_MODE;
    } else if(impl->ikModeRadio->isChecked()){
        return IK_MODE;
    }
    return AUTO_MODE;
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
    archive.write("mode", modeSymbol[mode()]);
    return impl->storeState(archive);
}


bool KinematicsBarImpl::storeState(Archive& archive)
{
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
    string storedModeSymbol = archive.get("mode", "AUTO");
    if(storedModeSymbol == "FK"){
        fkModeRadio->setChecked(true);
    } else if(storedModeSymbol == "IK"){
        ikModeRadio->setChecked(true);
    } else {
        autoModeRadio->setChecked(true);
        fkModeRadio->setChecked(false);
        ikModeRadio->setChecked(false);
    }

    draggerToggle->setChecked(archive.get("enablePositionDragger", draggerToggle->isChecked()));
    penetrationBlockToggle->setChecked(archive.get("penetrationBlock", penetrationBlockToggle->isChecked()));
    //footSnapToggle->setChecked(archive.get("footSnap", footSnapToggle->isChecked()));
    collisionLinkHighlightToggle->setChecked(archive.get("collisionLinkHighlight", collisionLinkHighlightToggle->isChecked()));
    setup->restoreState(archive);
    
    return true;
}


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
