/**
   @author Shin'ichiro Nakaoka
*/

#include "BodyMotionGenerationBar.h"
#include "PoseSeqItem.h"
#include <cnoid/BodyItem>
#include <cnoid/PoseProvider>
#include <cnoid/BodyMotionPoseProvider>
#include <cnoid/PoseProviderToBodyMotionConverter>
#include <cnoid/BodyMotionUtil>
#include <cnoid/RootItem>
#include <cnoid/ItemList>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/ExtensionManager>
#include <cnoid/MenuManager>
#include <cnoid/MainMenu>
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
#include <cnoid/Separator>
#include <cnoid/Buttons>
#include <cnoid/ButtonGroup>
#include <cnoid/CheckBox>
#include <cnoid/Dialog>
#include <QDialogButtonBox>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
const bool TRACE_FUNCTIONS = false;
}

namespace cnoid {

class BodyMotionGenerationSetupDialog : public Dialog
{
public:
    QVBoxLayout* vbox;

    DoubleSpinBox timeScaleRatioSpin;
    DoubleSpinBox preInitialDurationSpin;
    DoubleSpinBox postFinalDurationSpin;
    CheckBox onlyTimeBarRangeCheck;
    CheckBox newBodyMotionItemCheck;

    RadioButton noStepAdjustmentRadio;
    RadioButton stealthyStepRadio;
    RadioButton toeStepRadio;
    ButtonGroup stepAdjustmentRadioGroup;

    vector<QWidget*> stealthyStepWidgets;
    DoubleSpinBox stealthyHeightRatioThreshSpin;
    DoubleSpinBox flatLiftingHeightSpin;
    DoubleSpinBox flatLandingHeightSpin;
    DoubleSpinBox impactReductionHeightSpin;
    DoubleSpinBox impactReductionTimeSpin;

    vector<QWidget*> toeStepWidgets;
    DoubleSpinBox toeContactAngleSpin;
    DoubleSpinBox toeContactTimeSpin;

    CheckBox autoZmpCheck;
    DoubleSpinBox minZmpTransitionTimeSpin;
    DoubleSpinBox zmpCenteringTimeThreshSpin;
    DoubleSpinBox zmpTimeMarginBeforeLiftingSpin;
    DoubleSpinBox zmpMaxDistanceFromCenterSpin;

    CheckBox se3Check;
    CheckBox lipSyncMixCheck;

    void addSeparator(QVBoxLayout* vbox){
        vbox->addSpacing(4);
        vbox->addWidget(new HSeparator);
        vbox->addSpacing(2);
    }

    void addSeparator(QVBoxLayout* vbox, QWidget* widget) {
        vbox->addSpacing(4);
        vbox->addLayout(new HSeparatorBox(widget));
        vbox->addSpacing(2);
    }

    QHBoxLayout* newRow(QVBoxLayout* vbox) {
        QHBoxLayout* hbox = new QHBoxLayout;
        hbox->setSpacing(2);
        hbox->setContentsMargins(2, 2, 2, 2);
        vbox->addLayout(hbox);
        return hbox;
    }
        
    BodyMotionGenerationSetupDialog()
        {
            setWindowTitle(_("Body Motion Generation Setup"));

            vbox = new QVBoxLayout;

            QHBoxLayout* hbox = newRow(vbox);
            hbox->addWidget(new QLabel(_("Time scale")));
            timeScaleRatioSpin.setDecimals(2);
            timeScaleRatioSpin.setRange(0.01, 9.99);
            timeScaleRatioSpin.setSingleStep(0.01);
            timeScaleRatioSpin.setValue(1.0);
            hbox->addWidget(&timeScaleRatioSpin);
            
            hbox->addSpacing(8);
            hbox->addWidget(new QLabel(_("Pre-initial")));
            preInitialDurationSpin.setDecimals(1);
            preInitialDurationSpin.setRange(0.0, 9.9);
            preInitialDurationSpin.setSingleStep(0.1);
            preInitialDurationSpin.setValue(1.0);
            hbox->addWidget(&preInitialDurationSpin);
            hbox->addWidget(new QLabel(_("[s]")));
            
            hbox->addSpacing(8);
            hbox->addWidget(new QLabel(_("Post-final")));
            postFinalDurationSpin.setDecimals(1);
            postFinalDurationSpin.setRange(0.0, 9.9);
            postFinalDurationSpin.setSingleStep(0.1);
            postFinalDurationSpin.setValue(1.0);
            hbox->addWidget(&postFinalDurationSpin);
            hbox->addWidget(new QLabel(_("[s]")));
            hbox->addStretch();

            hbox = newRow(vbox);
            onlyTimeBarRangeCheck.setText(_("Time bar's range only"));
            onlyTimeBarRangeCheck.setChecked(false);
            hbox->addWidget(&onlyTimeBarRangeCheck);
            
            se3Check.setText(_("Put all link positions"));
            se3Check.setChecked(false);
            hbox->addWidget(&se3Check);
            hbox->addStretch();
            
            hbox = newRow(vbox);
            newBodyMotionItemCheck.setText(_("Output a new body motion item for a standalone body motion target"));
            newBodyMotionItemCheck.setChecked(true);
            hbox->addWidget(&newBodyMotionItemCheck);
            hbox->addStretch();

            addSeparator(vbox, new QLabel(_("Step trajectory adjustment")));

            hbox = newRow(vbox);

            hbox->addWidget(new QLabel(_("Mode:")));

            noStepAdjustmentRadio.setText(_("No adjustment"));
            noStepAdjustmentRadio.setToolTip(
                _("The mode to generate foot step trajectories as a simple interpolation of foot key frames"));
            hbox->addWidget(&noStepAdjustmentRadio);
            
            stealthyStepRadio.setText(_("Stealthy steps"));
            stealthyStepRadio.setToolTip(_("The mode to make foot lifting and landing motions smoother to increase the stability"));
            hbox->addWidget(&stealthyStepRadio);

            toeStepRadio.setText(_("Toe steps"));
            toeStepRadio.setToolTip(_("The mode to use the toes in foot lifting and landing to increase the stability"));
            hbox->addWidget(&toeStepRadio);
            hbox->addStretch();

            hbox = newRow(vbox);
            auto label = new QLabel(_("Height ratio thresh"));
            stealthyStepWidgets.push_back(label);
            hbox->addWidget(label);
            stealthyHeightRatioThreshSpin.setAlignment(Qt::AlignCenter);
            stealthyHeightRatioThreshSpin.setDecimals(2);
            stealthyHeightRatioThreshSpin.setRange(1.00, 9.99);
            stealthyHeightRatioThreshSpin.setSingleStep(0.01);
            stealthyHeightRatioThreshSpin.setValue(2.0);
            stealthyStepWidgets.push_back(&stealthyHeightRatioThreshSpin);
            hbox->addWidget(&stealthyHeightRatioThreshSpin);
            hbox->addStretch();

            hbox = newRow(vbox);
            label = new QLabel(_("Flat Lifting Height"));
            stealthyStepWidgets.push_back(label);
            hbox->addWidget(label);
            flatLiftingHeightSpin.setAlignment(Qt::AlignCenter);
            flatLiftingHeightSpin.setDecimals(3);
            flatLiftingHeightSpin.setRange(0.0, 0.0999);
            flatLiftingHeightSpin.setSingleStep(0.001);
            flatLiftingHeightSpin.setValue(0.005);
            stealthyStepWidgets.push_back(&flatLiftingHeightSpin);
            hbox->addWidget(&flatLiftingHeightSpin);
            label = new QLabel(_("[m]"));
            stealthyStepWidgets.push_back(label);
            hbox->addWidget(label);

            hbox->addSpacing(8);
            label = new QLabel(_("Flat Landing Height"));
            stealthyStepWidgets.push_back(label);
            hbox->addWidget(label);
            flatLandingHeightSpin.setAlignment(Qt::AlignCenter);
            flatLandingHeightSpin.setDecimals(3);
            flatLandingHeightSpin.setRange(0.0, 0.0999);
            flatLandingHeightSpin.setSingleStep(0.001);
            flatLandingHeightSpin.setValue(0.005);
            stealthyStepWidgets.push_back(&flatLandingHeightSpin);
            hbox->addWidget(&flatLandingHeightSpin);
            label = new QLabel(_("[m]"));
            stealthyStepWidgets.push_back(label);
            hbox->addWidget(label);
            hbox->addStretch();

            hbox = newRow(vbox);
            label = new QLabel(_("Impact reduction height"));
            stealthyStepWidgets.push_back(label);
            hbox->addWidget(label);
            impactReductionHeightSpin.setAlignment(Qt::AlignCenter);
            impactReductionHeightSpin.setDecimals(3);
            impactReductionHeightSpin.setRange(0.0, 0.099);
            impactReductionHeightSpin.setSingleStep(0.001);
            impactReductionHeightSpin.setValue(0.005);
            stealthyStepWidgets.push_back(&impactReductionHeightSpin);
            hbox->addWidget(&impactReductionHeightSpin);
            label = new QLabel(_("[m]"));
            stealthyStepWidgets.push_back(label);
            hbox->addWidget(label);
            
            hbox->addSpacing(8);
            label = new QLabel(_("Impact reduction time"));
            stealthyStepWidgets.push_back(label);
            hbox->addWidget(label);
            impactReductionTimeSpin.setAlignment(Qt::AlignCenter);
            impactReductionTimeSpin.setDecimals(3);
            impactReductionTimeSpin.setRange(0.001, 0.999);
            impactReductionTimeSpin.setSingleStep(0.001);
            impactReductionTimeSpin.setValue(0.04);
            stealthyStepWidgets.push_back(&impactReductionTimeSpin);
            hbox->addWidget(&impactReductionTimeSpin);
            label = new QLabel(_("[s]"));
            stealthyStepWidgets.push_back(label);
            hbox->addWidget(label);
            hbox->addStretch();

            hbox = newRow(vbox);
            label = new QLabel(_("Toe contact angle"));
            toeStepWidgets.push_back(label);
            hbox->addWidget(label);
            toeContactAngleSpin.setAlignment(Qt::AlignCenter);
            toeContactAngleSpin.setDecimals(1);
            toeContactAngleSpin.setRange(0.1, 45.0);
            toeContactAngleSpin.setSingleStep(0.1);
            toeContactAngleSpin.setValue(10.0);
            toeStepWidgets.push_back(&toeContactAngleSpin);
            hbox->addWidget(&toeContactAngleSpin);
            label = new QLabel(_("[s]"));
            toeStepWidgets.push_back(label);
            hbox->addWidget(label);

            label = new QLabel(_("Toe contact time"));
            toeStepWidgets.push_back(label);
            hbox->addWidget(label);
            toeContactTimeSpin.setAlignment(Qt::AlignCenter);
            toeContactTimeSpin.setDecimals(3);
            toeContactTimeSpin.setRange(0.001, 0.999);
            toeContactTimeSpin.setSingleStep(0.001);
            toeContactTimeSpin.setValue(0.1);
            toeStepWidgets.push_back(&toeContactTimeSpin);
            hbox->addWidget(&toeContactTimeSpin);
            label = new QLabel(_("[s]"));
            toeStepWidgets.push_back(label);
            hbox->addWidget(label);
            hbox->addStretch();

            stepAdjustmentRadioGroup.addButton(&noStepAdjustmentRadio, PoseSeqInterpolator::NoStepAdjustmentMode);
            stepAdjustmentRadioGroup.addButton(&stealthyStepRadio, PoseSeqInterpolator::StealthyStepMode);
            stepAdjustmentRadioGroup.addButton(&toeStepRadio, PoseSeqInterpolator::ToeStepMode);
            stealthyStepRadio.setChecked(true);
            stepAdjustmentRadioGroup.sigButtonToggled().connect(
                [this](int id, bool checked){
                    if(checked){
                        onStepAdjustmentModeChanged(id);
                    }
                });
            onStepAdjustmentModeChanged(PoseSeqInterpolator::StealthyStepMode);
                             
            addSeparator(vbox, &autoZmpCheck);
            autoZmpCheck.setText(_("Auto ZMP Mode"));
            autoZmpCheck.setToolTip(_("Automatically insert ZMP and foot key poses for stable motion"));
            autoZmpCheck.setChecked(true);

            hbox = newRow(vbox);
            hbox->addWidget(new QLabel(_("Min. transtion time")));
            minZmpTransitionTimeSpin.setDecimals(2);
            minZmpTransitionTimeSpin.setRange(0.01, 0.99);
            minZmpTransitionTimeSpin.setSingleStep(0.01);
            minZmpTransitionTimeSpin.setValue(0.1);
            hbox->addWidget(&minZmpTransitionTimeSpin);
            hbox->addWidget(new QLabel(_("[s]")));

            hbox->addSpacing(8);
            hbox->addWidget(new QLabel(_("Centering time thresh")));
            zmpCenteringTimeThreshSpin.setDecimals(3);
            zmpCenteringTimeThreshSpin.setRange(0.001, 0.999);
            zmpCenteringTimeThreshSpin.setSingleStep(0.001);
            zmpCenteringTimeThreshSpin.setValue(0.03);
            hbox->addWidget(&zmpCenteringTimeThreshSpin);
            hbox->addWidget(new QLabel(_("[s]")));
            hbox->addStretch();

            hbox = newRow(vbox);
            hbox->addWidget(new QLabel(_("Time margin before lifting")));
            zmpTimeMarginBeforeLiftingSpin.setDecimals(3);
            zmpTimeMarginBeforeLiftingSpin.setRange(0.0, 0.999);
            zmpTimeMarginBeforeLiftingSpin.setSingleStep(0.001);
            zmpTimeMarginBeforeLiftingSpin.setValue(0.0);
            hbox->addWidget(&zmpTimeMarginBeforeLiftingSpin);
            hbox->addWidget(new QLabel(_("[s]")));

            hbox->addSpacing(8);
            hbox->addWidget(new QLabel(_("Max distance from center")));
            zmpMaxDistanceFromCenterSpin.setDecimals(3);
            zmpMaxDistanceFromCenterSpin.setRange(0.001, 0.999);
            zmpMaxDistanceFromCenterSpin.setSingleStep(0.001);
            zmpMaxDistanceFromCenterSpin.setValue(0.02);
            hbox->addWidget(&zmpMaxDistanceFromCenterSpin);
            hbox->addWidget(new QLabel(_("[m]")));
            hbox->addStretch();

            addSeparator(vbox);
            hbox = newRow(vbox);

            lipSyncMixCheck.setText(_("Mix lip-sync motion"));
            lipSyncMixCheck.setChecked(false);
            hbox->addWidget(&lipSyncMixCheck);
            hbox->addStretch();

            QVBoxLayout* topVBox = new QVBoxLayout;
            topVBox->addLayout(vbox);

            addSeparator(topVBox);

            QPushButton* okButton = new QPushButton(_("&Ok"));
            okButton->setDefault(true);
            QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
            buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
            connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
            topVBox->addWidget(buttonBox);

            setLayout(topVBox);
        }

    void onStepAdjustmentModeChanged(int mode){
        for(auto& widget : stealthyStepWidgets){
            widget->setEnabled(mode == PoseSeqInterpolator::StealthyStepMode);
        }
        for(auto& widget : toeStepWidgets){
            widget->setEnabled(mode == PoseSeqInterpolator::ToeStepMode);
        }
        
    }

    void storeState(Archive& archive){
        archive.write("timeScaleRatio", timeScaleRatioSpin.value());
        archive.write("preInitialDuration", preInitialDurationSpin.value());
        archive.write("postFinalDuration", postFinalDurationSpin.value());
        archive.write("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked());
        archive.write("output_new_body_motion_item", newBodyMotionItemCheck.isChecked());

        const char* mode = nullptr;
        switch(stepAdjustmentRadioGroup.checkedId()){
            case PoseSeqInterpolator::NoStepAdjustmentMode: mode = "disabled"; break;
            case PoseSeqInterpolator::StealthyStepMode:     mode = "stealthy"; break;
            case PoseSeqInterpolator::ToeStepMode:          mode = "toe";      break;
            default: break;
        }
        if(mode){
            archive.write("step_trajectory_adjustment_mode", mode);
        }

        archive.write("stealthyHeightRatioThresh", stealthyHeightRatioThreshSpin.value());
        archive.write("flatLiftingHeight", flatLiftingHeightSpin.value());
        archive.write("flatLandingHeight", flatLandingHeightSpin.value());
        archive.write("impactReductionHeight", impactReductionHeightSpin.value());
        archive.write("impactReductionTime", impactReductionTimeSpin.value());

        archive.write("toe_contact_time", toeContactTimeSpin.value());
        archive.write("toe_contact_angle", toeContactAngleSpin.value());

        archive.write("autoZmp", autoZmpCheck.isChecked());
        archive.write("minZmpTransitionTime", minZmpTransitionTimeSpin.value());
        archive.write("zmpCenteringTimeThresh", zmpCenteringTimeThreshSpin.value());
        archive.write("zmpTimeMarginBeforeLiftingSpin", zmpTimeMarginBeforeLiftingSpin.value());
        archive.write("zmpMaxDistanceFromCenter", zmpMaxDistanceFromCenterSpin.value());
        archive.write("allLinkPositions", se3Check.isChecked());
        archive.write("lipSyncMix", lipSyncMixCheck.isChecked());
    }

    void restoreState(const Archive& archive){
        timeScaleRatioSpin.setValue(archive.get("timeScaleRatio", timeScaleRatioSpin.value()));
        preInitialDurationSpin.setValue(archive.get("preInitialDuration", preInitialDurationSpin.value()));
        postFinalDurationSpin.setValue(archive.get("postFinalDuration", postFinalDurationSpin.value()));
        onlyTimeBarRangeCheck.setChecked(archive.get("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked()));
        newBodyMotionItemCheck.setChecked(
            archive.get({ "output_new_body_motion_item", "makeNewBodyMotionItem" }, newBodyMotionItemCheck.isChecked()));

        string symbol;
        if(archive.read("step_trajectory_adjustment_mode", symbol)){
            int mode = PoseSeqInterpolator::NoStepAdjustmentMode;
            if(symbol == "stealthy"){
                mode = PoseSeqInterpolator::StealthyStepMode;
            } else if(symbol == "toe"){
                mode = PoseSeqInterpolator::ToeStepMode;
            }
            stepAdjustmentRadioGroup.button(mode)->setChecked(true);
        } else {
            if(archive.get("stealthyStepMode", false)){
                stepAdjustmentRadioGroup.button(PoseSeqInterpolator::StealthyStepMode)->setChecked(true);
            }
        }
        
        stealthyHeightRatioThreshSpin.setValue(
            archive.get("stealthyHeightRatioThresh", stealthyHeightRatioThreshSpin.value()));
        flatLiftingHeightSpin.setValue(archive.get("flatLiftingHeight", flatLiftingHeightSpin.value()));
        flatLandingHeightSpin.setValue(archive.get("flatLandingHeight", flatLandingHeightSpin.value()));
        impactReductionHeightSpin.setValue(archive.get("impactReductionHeight", impactReductionHeightSpin.value()));
        impactReductionTimeSpin.setValue(archive.get("impactReductionTime", impactReductionTimeSpin.value()));

        toeContactTimeSpin.setValue(archive.get("toe_contact_time", toeContactTimeSpin.value()));
        toeContactAngleSpin.setValue(archive.get("toe_contact_angle", toeContactAngleSpin.value()));

        autoZmpCheck.setChecked(archive.get("autoZmp", autoZmpCheck.isChecked()));
        minZmpTransitionTimeSpin.setValue(archive.get("minZmpTransitionTime", minZmpTransitionTimeSpin.value()));
        zmpCenteringTimeThreshSpin.setValue(archive.get("zmpCenteringTimeThresh", zmpCenteringTimeThreshSpin.value()));
        zmpTimeMarginBeforeLiftingSpin.setValue(
            archive.get("zmpTimeMarginBeforeLiftingSpin", zmpTimeMarginBeforeLiftingSpin.value()));
        zmpMaxDistanceFromCenterSpin.setValue(archive.get(
                                                  "zmpMaxDistanceFromCenter", zmpMaxDistanceFromCenterSpin.value()));            
            
        se3Check.setChecked(archive.get("allLinkPositions", se3Check.isChecked()));
        lipSyncMixCheck.setChecked(archive.get("lipSyncMix", lipSyncMixCheck.isChecked()));
    }
};

}
    

void BodyMotionGenerationBar::initializeInstance(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        BodyMotionGenerationBar* bar = instance();
        ext->addToolBar(bar);

        if(auto optionsMenu = MainMenu::instance()->get_Options_Menu()){
            MenuManager& mm = ext->menuManager();
            mm.setCurrent(optionsMenu).setPath(N_("Pose Seq Processing"));
            bar->autoGenerationForNewBodyCheck = mm.addCheckItem(_("Automatic Generation for a New Body"));
        } else {
            bar->autoGenerationForNewBodyCheck = new Action;
        }
        bar->autoGenerationForNewBodyCheck->setChecked(true);

        initialized = true;
    }
}


BodyMotionGenerationBar* BodyMotionGenerationBar::instance()
{
    static BodyMotionGenerationBar* bar = new BodyMotionGenerationBar;
    return bar;
}


BodyMotionGenerationBar::BodyMotionGenerationBar()
    : ToolBar(N_("BodyMotionGenerationBar"))
{
    bodyMotionPoseProvider = new BodyMotionPoseProvider;
    poseProviderToBodyMotionConverter = new PoseProviderToBodyMotionConverter;
    timeBar = TimeBar::instance();
    setup = new BodyMotionGenerationSetupDialog;
    balancer = nullptr;

    auto generationButton = addButton(QIcon(":/PoseSeq/icon/trajectory-generation.svg"));
    generationButton->setToolTip(_("Generate body motions"));
    generationButton->sigClicked().connect([&](){ onGenerationButtonClicked(); });

    interpolationParameterWidgetsConnection.add(
        setup->timeScaleRatioSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));
    
    interpolationParameterWidgetsConnection.add(
        setup->preInitialDurationSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->postFinalDurationSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->onlyTimeBarRangeCheck.sigToggled().connect(
            [&](bool){ notifyInterpolationParametersChanged(); }));
    
    autoGenerationToggle = addToggleButton(QIcon(":/PoseSeq/icon/auto-update.svg"));
    autoGenerationToggle->setToolTip(_("Automatic Balance Adjustment Mode"));
    autoGenerationToggle->setChecked(false);
    
    balancerToggle = addToggleButton(QIcon(":/PoseSeq/icon/balancer.svg"));
    balancerToggle->setToolTip(_("Enable the balancer"));
    balancerToggle->setEnabled(false);
    balancerToggle->setChecked(false);

    addButton(QIcon(":/Base/icon/setup.svg"))
        ->sigClicked().connect([this](){ setup->show(); });
    
    interpolationParameterWidgetsConnection.add(
        setup->stepAdjustmentRadioGroup.sigButtonToggled().connect(
            [&](int, bool){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->stealthyHeightRatioThreshSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));
    
    interpolationParameterWidgetsConnection.add(
        setup->flatLiftingHeightSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->flatLandingHeightSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));
    
    interpolationParameterWidgetsConnection.add(
        setup->impactReductionHeightSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->impactReductionTimeSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->toeContactAngleSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->toeContactTimeSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));
    
    interpolationParameterWidgetsConnection.add(
        setup->autoZmpCheck.sigToggled().connect(
            [&](bool){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->minZmpTransitionTimeSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->zmpCenteringTimeThreshSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->zmpTimeMarginBeforeLiftingSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));

    interpolationParameterWidgetsConnection.add(
        setup->zmpMaxDistanceFromCenterSpin.sigValueChanged().connect(
            [&](double){ notifyInterpolationParametersChanged(); }));
    
    interpolationParameterWidgetsConnection.add(
        setup->lipSyncMixCheck.sigToggled().connect(
            [&](bool){ notifyInterpolationParametersChanged(); }));
}


BodyMotionGenerationBar::~BodyMotionGenerationBar()
{
    delete bodyMotionPoseProvider;
    delete poseProviderToBodyMotionConverter;
}


void BodyMotionGenerationBar::notifyInterpolationParametersChanged()
{
    sigInterpolationParametersChanged_.request();
}

        
void BodyMotionGenerationBar::onGenerationButtonClicked()
{
    set<BodyMotionItem*> motionItems; // for avoiding overlap

    auto selectedItems = RootItem::instance()->selectedItems();
    for(size_t i=0; i < selectedItems.size(); ++i){
        PoseSeqItem* poseSeqItem = selectedItems.get<PoseSeqItem>(i);
        if(poseSeqItem){
            motionItems.insert(poseSeqItem->bodyMotionItem());
        } else {
            BodyMotionItem* motionItem = selectedItems.get<BodyMotionItem>(i);
            if(motionItem){
                motionItems.insert(motionItem);
            }
        }
    }

    for(set<BodyMotionItem*>::iterator p = motionItems.begin(); p != motionItems.end(); ++p){
        BodyMotionItem* motionItem = *p;
        BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>(true);
        if(bodyItem){
            PoseProvider* provider = nullptr;
            PoseSeqItem* poseSeqItem = dynamic_cast<PoseSeqItem*>(motionItem->parentItem());
            if(poseSeqItem){
                provider = poseSeqItem->interpolator().get();
            } else {
                bodyMotionPoseProvider->initialize(bodyItem->body(), motionItem->motion());
                provider = bodyMotionPoseProvider;

                if(setup->newBodyMotionItemCheck.isChecked()){
                    BodyMotionItem* newMotionItem = new BodyMotionItem;
                    newMotionItem->setName(motionItem->name() + "'");
                    motionItem->parentItem()->insertChild(motionItem->nextItem(), newMotionItem);
                    motionItem = newMotionItem;
                }
            }
            shapeBodyMotion(bodyItem->body(), provider, motionItem, true);
        }
    }
}


void BodyMotionGenerationBar::setBalancer(Balancer* balancer)
{
    this->balancer = balancer;
    balancerToggle->setEnabled(balancer != nullptr);
    if(balancer){
        setup->vbox->addWidget(balancer->panel());
    }
}


void BodyMotionGenerationBar::unsetBalancer()
{
    balancerToggle->setEnabled(false);
    if(balancer){
        setup->layout()->removeWidget(balancer->panel());
        balancer = nullptr;
    }
}


bool BodyMotionGenerationBar::shapeBodyMotion
(BodyPtr body, PoseProvider* provider, BodyMotionItemPtr motionItem, bool putMessages)
{
    bool result = false;
    
    if(balancerToggle->isChecked() && balancer){
        result = balancer->apply(body, provider, motionItem, putMessages);
    } else {
        result = shapeBodyMotionWithSimpleInterpolation(body, provider, motionItem);
    }

    return result;
}
        

bool BodyMotionGenerationBar::shapeBodyMotionWithSimpleInterpolation
(BodyPtr& body, PoseProvider* provider, BodyMotionItemPtr motionItem)
{
    if(setup->onlyTimeBarRangeCheck.isChecked()){
        poseProviderToBodyMotionConverter->setTimeRange(timeBar->minTime(), timeBar->maxTime());
    } else {
        poseProviderToBodyMotionConverter->setFullTimeRange();
    }
    poseProviderToBodyMotionConverter->setAllLinkPositionOutput(setup->se3Check.isChecked());
        
    auto motion = motionItem->motion();
    motion->setFrameRate(timeBar->frameRate());

    bool result = poseProviderToBodyMotionConverter->convert(body, provider, *motion);
    
    if(result){
        motionItem->notifyUpdate();
    }
    return result;
}


bool BodyMotionGenerationBar::storeState(Archive& archive)
{
    archive.write("autoGenerationForNewBody", autoGenerationForNewBodyCheck->isChecked());
    archive.write("balancer", balancerToggle->isChecked());
    archive.write("autoGeneration", autoGenerationToggle->isChecked());
    setup->storeState(archive);
    if(balancer){
        balancer->storeState(archive);
    }
    return true;
}


bool BodyMotionGenerationBar::restoreState(const Archive& archive)
{
    autoGenerationForNewBodyCheck->setChecked(archive.get("autoGenerationForNewBody", autoGenerationForNewBodyCheck->isChecked()));
    balancerToggle->setChecked(archive.get("balancer", balancerToggle->isChecked()));
    autoGenerationToggle->setChecked(archive.get("autoGeneration", autoGenerationToggle->isChecked()));
    setup->restoreState(archive);
    if(balancer){
        balancer->restoreState(archive);
    }
    return true;
}


bool BodyMotionGenerationBar::isBalancerEnabled() const
{
    return balancerToggle->isChecked();
}
            
bool BodyMotionGenerationBar::isAutoGenerationMode() const
{
    return autoGenerationToggle->isChecked();
}

bool BodyMotionGenerationBar::isAutoGenerationForNewBodyEnabled() const
{
    return autoGenerationForNewBodyCheck->isChecked();
}

double BodyMotionGenerationBar::timeScaleRatio() const
{
    return setup->timeScaleRatioSpin.value();
}

double BodyMotionGenerationBar::preInitialDuration() const
{
    return setup->preInitialDurationSpin.value();
}

double BodyMotionGenerationBar::postFinalDuration() const
{
    return setup->postFinalDurationSpin.value();
}

bool BodyMotionGenerationBar::isTimeBarRangeOnly() const
{
    return setup->onlyTimeBarRangeCheck.isChecked();
}

int BodyMotionGenerationBar::stepTrajectoryAdjustmentMode() const
{
    return setup->stepAdjustmentRadioGroup.checkedId();
}

double BodyMotionGenerationBar::stealthyHeightRatioThresh() const
{
    return setup->stealthyHeightRatioThreshSpin.value();
}

double BodyMotionGenerationBar::flatLiftingHeight() const
{
    return setup->flatLiftingHeightSpin.value();
}

double BodyMotionGenerationBar::flatLandingHeight() const
{
    return setup->flatLandingHeightSpin.value();
}

double BodyMotionGenerationBar::impactReductionHeight() const
{
    return setup->impactReductionHeightSpin.value();
}

double BodyMotionGenerationBar::impactReductionTime() const
{
    return setup->impactReductionTimeSpin.value();
}

double BodyMotionGenerationBar::toeContactTime() const
{
    return setup->toeContactTimeSpin.value();
}

double BodyMotionGenerationBar::toeContactAngle() const
{
    return radian(setup->toeContactAngleSpin.value());
}

bool BodyMotionGenerationBar::isAutoZmpAdjustmentMode() const
{
    return setup->autoZmpCheck.isChecked();
}

double BodyMotionGenerationBar::minZmpTransitionTime() const
{
    return setup->minZmpTransitionTimeSpin.value();
}

double BodyMotionGenerationBar::zmpCenteringTimeThresh() const
{
    return setup->zmpCenteringTimeThreshSpin.value();
}

double BodyMotionGenerationBar::zmpTimeMarginBeforeLifting() const
{
    return setup->zmpTimeMarginBeforeLiftingSpin.value();
}

double BodyMotionGenerationBar::zmpMaxDistanceFromCenter() const
{
    return setup->zmpMaxDistanceFromCenterSpin.value();
}

bool BodyMotionGenerationBar::isSe3Enabled() const
{
    return setup->se3Check.isChecked();
}

bool BodyMotionGenerationBar::isLipSyncMixMode() const
{
    return setup->lipSyncMixCheck.isChecked();
}
