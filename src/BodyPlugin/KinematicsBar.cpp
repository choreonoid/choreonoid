/**
   @author Shin'ichiro Nakaoka
*/

#include "KinematicsBar.h"
#include <cnoid/MainWindow>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include <cnoid/Dialog>
#include <cnoid/LazyCaller>
#include <QBoxLayout>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
const char* modeSymbol[] = { "AUTO", "FK", "IK" };
}

namespace cnoid {

class KinematicsBarSetupDialog : public Dialog
{
public:
    DoubleSpinBox snapDistanceSpin;
    SpinBox snapAngleSpin;
    DoubleSpinBox penetrationBlockDepthSpin;
    CheckBox lazyCollisionDetectionModeCheck;
    PushButton okButton;
        
    KinematicsBarSetupDialog() {

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

    void storeState(Archive& archive){
        archive.write("snapDistance", snapDistanceSpin.value());
        archive.write("penetrationBlockDepth", penetrationBlockDepthSpin.value());
        archive.write("lazyCollisionDetectionMode", lazyCollisionDetectionModeCheck.isChecked());
    }

    void restoreState(const Archive& archive){
        snapDistanceSpin.setValue(archive.get("snapDistance", snapDistanceSpin.value()));
        penetrationBlockDepthSpin.setValue(archive.get("penetrationBlockDepth", penetrationBlockDepthSpin.value()));
        lazyCollisionDetectionModeCheck.setChecked(archive.get("lazyCollisionDetectionMode", lazyCollisionDetectionModeCheck.isChecked()));
    }

    /*
      virtual void on_response(int id) {
      hide();
      }
    */
};

}


KinematicsBar* KinematicsBar::instance()
{
    static KinematicsBar* instance = new KinematicsBar();
    return instance;
}


KinematicsBar::KinematicsBar() : ToolBar(N_("KinematicsBar"))
{
    setup = new KinematicsBarSetupDialog();

    //addSeparator(2);
    
    fkModeRadio = addRadioButton(QIcon(":/Body/icons/fk.png"), _("Forward kinematics mode"));
    autoModeRadio = addRadioButton(QIcon(":/Body/icons/fkik.png"), _("Preset kinematics mode"));
    ikModeRadio = addRadioButton(QIcon(":/Body/icons/ik.png"), _("Inverse kinematics mode"));
    autoModeRadio->setChecked(true);

    addSeparator(2);

    attitudeToggle = addToggleButton(QIcon(":/Body/icons/rotation.png"), _("Enable link orientation editing"));
    attitudeToggle->setChecked(false);

    /*
      footSnapToggle = addToggleButton(xpmfootsnapon, _("Snap foot to the floor"));
      footSnapToggle->setChecked(true);
    */

    /*
      jointPositionLimitToggle = addToggleButton(
      xpmlimitangle, _("Limit joint positions within the movable range specifications"));
      jointPositionLimitToggle->setChecked(true);
    */
    
    penetrationBlockToggle = addToggleButton(QIcon(":/Body/icons/block.png"), _("Penetration block mode"));
    penetrationBlockToggle->setChecked(true);

    collisionLinkHighlightToggle = addToggleButton(QIcon(":/Body/icons/collisionoutline.png"), _("Highlight colliding links"));
    collisionLinkHighlightToggle->setChecked(false);
    collisionLinkHighlightToggle->sigToggled().connect(boost::bind(&KinematicsBar::onCollisionVisualizationChanged, this));

    addButton(QIcon(":/Base/icons/setup.png"))->sigClicked().connect(boost::bind(&KinematicsBarSetupDialog::show, setup));
    
    setup->lazyCollisionDetectionModeCheck.sigToggled().connect(
        boost::bind(&KinematicsBar::onLazyCollisionDetectionModeToggled, this));
    
    onLazyCollisionDetectionModeToggled();
}


KinematicsBar::~KinematicsBar()
{

}


int KinematicsBar::mode() const
{
    if(fkModeRadio->isChecked()){
        return FK_MODE;
    } else if(ikModeRadio->isChecked()){
        return IK_MODE;
    }
    return AUTO_MODE;
}


void KinematicsBar::getSnapThresholds(double& distance, double& angle) const
{
    distance = setup->snapDistanceSpin.value();
    angle = radian(setup->snapAngleSpin.value());
}


double KinematicsBar::penetrationBlockDepth() const
{
    return setup->penetrationBlockDepthSpin.value();
}


void KinematicsBar::onCollisionVisualizationChanged()
{
    sigCollisionVisualizationChanged_();
}


void KinematicsBar::onLazyCollisionDetectionModeToggled()
{
    if(setup->lazyCollisionDetectionModeCheck.isChecked()){
        collisionDetectionPriority_ = LazyCaller::PRIORITY_NORMAL;
    } else {
        collisionDetectionPriority_ = LazyCaller::PRIORITY_HIGH;
    }
}


bool KinematicsBar::storeState(Archive& archive)
{
    archive.write("mode", modeSymbol[mode()]);
    archive.write("attitude", attitudeToggle->isChecked());
    archive.write("penetrationBlock", penetrationBlockToggle->isChecked());
    //archive.write("footSnap", footSnapToggle->isChecked());
    archive.write("collisionLinkHighlight", collisionLinkHighlightToggle->isChecked());
    setup->storeState(archive);
    return true;
}


bool KinematicsBar::restoreState(const Archive& archive)
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

    attitudeToggle->setChecked(archive.get("attitude", attitudeToggle->isChecked()));
    penetrationBlockToggle->setChecked(archive.get("penetrationBlock", penetrationBlockToggle->isChecked()));
    //footSnapToggle->setChecked(archive.get("footSnap", footSnapToggle->isChecked()));
    collisionLinkHighlightToggle->setChecked(archive.get("collisionLinkHighlight", collisionLinkHighlightToggle->isChecked()));
    setup->restoreState(archive);
    
    return true;
}
