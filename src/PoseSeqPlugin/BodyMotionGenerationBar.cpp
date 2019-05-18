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
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/MenuManager>
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
#include <cnoid/Separator>
#include <cnoid/Buttons>
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
    CheckBox newBodyItemCheck;
        
    CheckBox stealthyStepCheck;
    DoubleSpinBox stealthyHeightRatioThreshSpin;
    DoubleSpinBox flatLiftingHeightSpin;
    DoubleSpinBox flatLandingHeightSpin;
    DoubleSpinBox impactReductionHeightSpin;
    DoubleSpinBox impactReductionTimeSpin;

    CheckBox autoZmpCheck;
    DoubleSpinBox minZmpTransitionTimeSpin;
    DoubleSpinBox zmpCenteringTimeThreshSpin;
    DoubleSpinBox zmpTimeMarginBeforeLiftingSpin;
    DoubleSpinBox zmpMaxDistanceFromCenterSpin;
        
    CheckBox se3Check;
    CheckBox lipSyncMixCheck;

    void addSeparator(QVBoxLayout* vbox){
        vbox->addSpacing(4);
        vbox->addWidget(new HSeparator());
        vbox->addSpacing(2);
    }

    void addSeparator(QVBoxLayout* vbox, QWidget* widget) {
        vbox->addSpacing(4);
        vbox->addLayout(new HSeparatorBox(widget));
        vbox->addSpacing(2);
    }

    QHBoxLayout* newRow(QVBoxLayout* vbox) {
        QHBoxLayout* hbox = new QHBoxLayout();
        hbox->setSpacing(2);
        hbox->setContentsMargins(2, 2, 2, 2);
        vbox->addLayout(hbox);
        return hbox;
    }
        
    BodyMotionGenerationSetupDialog()
        {
            setWindowTitle(_("Body Motion Generation Setup"));

            vbox = new QVBoxLayout();

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
            newBodyItemCheck.setText(_("Make a new body item"));
            newBodyItemCheck.setChecked(true);
            hbox->addWidget(&newBodyItemCheck);
            hbox->addStretch();

            addSeparator(vbox, &stealthyStepCheck);
            stealthyStepCheck.setText(_("Stealthy Step Mode"));
            stealthyStepCheck.setToolTip(_("This mode makes foot lifting / landing smoother to increase the stability"));
            stealthyStepCheck.setChecked(true);

            hbox = newRow(vbox);
            hbox->addWidget(new QLabel(_("Height ratio thresh")));
            stealthyHeightRatioThreshSpin.setAlignment(Qt::AlignCenter);
            stealthyHeightRatioThreshSpin.setDecimals(2);
            stealthyHeightRatioThreshSpin.setRange(1.00, 9.99);
            stealthyHeightRatioThreshSpin.setSingleStep(0.01);
            stealthyHeightRatioThreshSpin.setValue(2.0);
            hbox->addWidget(&stealthyHeightRatioThreshSpin);
            hbox->addStretch();

            hbox = newRow(vbox);
            hbox->addWidget(new QLabel(_("Flat Lifting Height")));
            flatLiftingHeightSpin.setAlignment(Qt::AlignCenter);
            flatLiftingHeightSpin.setDecimals(3);
            flatLiftingHeightSpin.setRange(0.0, 0.0999);
            flatLiftingHeightSpin.setSingleStep(0.001);
            flatLiftingHeightSpin.setValue(0.005);
            hbox->addWidget(&flatLiftingHeightSpin);
            hbox->addWidget(new QLabel(_("[m]")));

            hbox->addSpacing(8);
            hbox->addWidget(new QLabel(_("Flat Landing Height")));
            flatLandingHeightSpin.setAlignment(Qt::AlignCenter);
            flatLandingHeightSpin.setDecimals(3);
            flatLandingHeightSpin.setRange(0.0, 0.0999);
            flatLandingHeightSpin.setSingleStep(0.001);
            flatLandingHeightSpin.setValue(0.005);
            hbox->addWidget(&flatLandingHeightSpin);
            hbox->addWidget(new QLabel(_("[m]")));
            hbox->addStretch();

            hbox = newRow(vbox);
            hbox->addWidget(new QLabel(_("Impact reduction height")));
            impactReductionHeightSpin.setAlignment(Qt::AlignCenter);
            impactReductionHeightSpin.setDecimals(3);
            impactReductionHeightSpin.setRange(0.0, 0.099);
            impactReductionHeightSpin.setSingleStep(0.001);
            impactReductionHeightSpin.setValue(0.005);
            hbox->addWidget(&impactReductionHeightSpin);
            hbox->addWidget(new QLabel(_("[m]")));
            
            hbox->addSpacing(8);
            hbox->addWidget(new QLabel(_("Impact reduction time")));
            impactReductionTimeSpin.setAlignment(Qt::AlignCenter);
            impactReductionTimeSpin.setDecimals(3);
            impactReductionTimeSpin.setRange(0.001, 0.999);
            impactReductionTimeSpin.setSingleStep(0.001);
            impactReductionTimeSpin.setValue(0.04);
            hbox->addWidget(&impactReductionTimeSpin);
            hbox->addWidget(new QLabel(_("[s]")));
            hbox->addStretch();
                             
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

            QVBoxLayout* topVBox = new QVBoxLayout();
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
        

    void storeState(Archive& archive){
        archive.write("timeScaleRatio", timeScaleRatioSpin.value());
        archive.write("preInitialDuration", preInitialDurationSpin.value());
        archive.write("postFinalDuration", postFinalDurationSpin.value());
        archive.write("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked());
        archive.write("makeNewBodyItem", newBodyItemCheck.isChecked());
        archive.write("stealthyStepMode", stealthyStepCheck.isChecked());
        archive.write("stealthyHeightRatioThresh", stealthyHeightRatioThreshSpin.value());
        archive.write("flatLiftingHeight", flatLiftingHeightSpin.value());
        archive.write("flatLandingHeight", flatLandingHeightSpin.value());
        archive.write("impactReductionHeight", impactReductionHeightSpin.value());
        archive.write("impactReductionTime", impactReductionTimeSpin.value());
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
        newBodyItemCheck.setChecked(archive.get("makeNewBodyItem", newBodyItemCheck.isChecked()));
        stealthyStepCheck.setChecked(archive.get("stealthyStepMode", stealthyStepCheck.isChecked()));
        stealthyHeightRatioThreshSpin.setValue(archive.get("stealthyHeightRatioThresh", stealthyHeightRatioThreshSpin.value()));
        flatLiftingHeightSpin.setValue(archive.get("flatLiftingHeight", flatLiftingHeightSpin.value()));
        flatLandingHeightSpin.setValue(archive.get("flatLandingHeight", flatLandingHeightSpin.value()));
        impactReductionHeightSpin.setValue(archive.get("impactReductionHeight", impactReductionHeightSpin.value()));
        impactReductionTimeSpin.setValue(archive.get("impactReductionTime", impactReductionTimeSpin.value()));

        autoZmpCheck.setChecked(archive.get("autoZmp", autoZmpCheck.isChecked()));
        minZmpTransitionTimeSpin.setValue(archive.get("minZmpTransitionTime", minZmpTransitionTimeSpin.value()));
        zmpCenteringTimeThreshSpin.setValue(archive.get("zmpCenteringTimeThresh", zmpCenteringTimeThreshSpin.value()));
        zmpTimeMarginBeforeLiftingSpin.setValue(archive.get("zmpTimeMarginBeforeLiftingSpin", zmpTimeMarginBeforeLiftingSpin.value()));
        zmpMaxDistanceFromCenterSpin.setValue(archive.get("zmpMaxDistanceFromCenter", zmpMaxDistanceFromCenterSpin.value()));            
            
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

        MenuManager& mm = ext->menuManager();
    
        mm.setPath("/Options").setPath(N_("Pose Seq Processing"));

        bar->autoInterpolationUpdateCheck = mm.addCheckItem(_("Automatic Interpolation Update"));
        bar->autoInterpolationUpdateCheck->setChecked(true);

        bar->autoGenerationForNewBodyCheck = mm.addCheckItem(_("Automatic Generation for a New Body"));
        bar->autoGenerationForNewBodyCheck->setChecked(true);

        mm.addSeparator();
    
        initialized = true;
    }
}


BodyMotionGenerationBar* BodyMotionGenerationBar::instance()
{
    static BodyMotionGenerationBar* bar = new BodyMotionGenerationBar();
    return bar;
}


BodyMotionGenerationBar::BodyMotionGenerationBar()
    : ToolBar("BodyMotionGenerationBar")
{
    setVisibleByDefault(true);

    bodyMotionPoseProvider = new BodyMotionPoseProvider();
    poseProviderToBodyMotionConverter = new PoseProviderToBodyMotionConverter();
    timeBar = TimeBar::instance();
    setup = new BodyMotionGenerationSetupDialog();
    balancer = 0;

    addButton(QIcon(":/PoseSeq/icons/trajectory-generation.png"), _("Generate body motions"))
        ->sigClicked().connect(std::bind(&BodyMotionGenerationBar::onGenerationButtonClicked, this));

    interpolationParameterWidgetsConnection.add(
        setup->timeScaleRatioSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));
    
    interpolationParameterWidgetsConnection.add(
        setup->preInitialDurationSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));

    interpolationParameterWidgetsConnection.add(
        setup->postFinalDurationSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));

    
    interpolationParameterWidgetsConnection.add(
        setup->onlyTimeBarRangeCheck.sigToggled().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));
    
    autoGenerationToggle = addToggleButton(QIcon(":/PoseSeq/icons/auto-update.png"), _("Automatic Balance Adjustment Mode"));
    autoGenerationToggle->setChecked(false);
    
    balancerToggle = addToggleButton(QIcon(":/PoseSeq/icons/balancer.png"), _("Enable the balancer"));
    balancerToggle->setEnabled(false);
    balancerToggle->setChecked(false);

    addButton(QIcon(":/Base/icons/setup.png"))->sigClicked().connect(std::bind(&BodyMotionGenerationSetupDialog::show, setup));
    
    interpolationParameterWidgetsConnection.add(
        setup->stealthyStepCheck.sigToggled().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));

    interpolationParameterWidgetsConnection.add(
        setup->stealthyHeightRatioThreshSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));
    
    interpolationParameterWidgetsConnection.add(
        setup->flatLiftingHeightSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));

    interpolationParameterWidgetsConnection.add(
        setup->flatLandingHeightSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));
    
    interpolationParameterWidgetsConnection.add(
        setup->impactReductionHeightSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));

    interpolationParameterWidgetsConnection.add(
        setup->impactReductionTimeSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));
    
    interpolationParameterWidgetsConnection.add(
        setup->autoZmpCheck.sigToggled().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));

    interpolationParameterWidgetsConnection.add(
        setup->minZmpTransitionTimeSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));

    interpolationParameterWidgetsConnection.add(
        setup->zmpCenteringTimeThreshSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));

    interpolationParameterWidgetsConnection.add(
        setup->zmpTimeMarginBeforeLiftingSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));

    interpolationParameterWidgetsConnection.add(
        setup->zmpMaxDistanceFromCenterSpin.sigValueChanged().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));
    
    interpolationParameterWidgetsConnection.add(
        setup->lipSyncMixCheck.sigToggled().connect(
            std::bind(&BodyMotionGenerationBar::notifyInterpolationParametersChanged, this)));
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
    ItemList<Item> selectedItems = ItemTreeView::mainInstance()->selectedItems<Item>();

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
            PoseProvider* provider = 0;
            PoseSeqItem* poseSeqItem = dynamic_cast<PoseSeqItem*>(motionItem->parentItem());
            if(poseSeqItem){
                provider = poseSeqItem->interpolator().get();
            } else {
                bodyMotionPoseProvider->initialize(bodyItem->body(), motionItem->motion());
                provider = bodyMotionPoseProvider;

                if(setup->newBodyItemCheck.isChecked()){
                    BodyMotionItem* newMotionItem = new BodyMotionItem();
                    newMotionItem->setName(motionItem->name() + "'");
                    motionItem->parentItem()->insertChildItem(newMotionItem, motionItem->nextItem());
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
    balancerToggle->setEnabled(balancer != 0);
    if(balancer){
        setup->vbox->addWidget(balancer->panel());
    }
}


void BodyMotionGenerationBar::unsetBalancer()
{
    balancerToggle->setEnabled(false);
    if(balancer){
        setup->layout()->removeWidget(balancer->panel());
        balancer = 0;
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


bool BodyMotionGenerationBar::isAutoInterpolationUpdateMode() const
{
    return autoInterpolationUpdateCheck->isChecked();
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

bool BodyMotionGenerationBar::isStealthyStepMode() const
{
    return setup->stealthyStepCheck.isChecked();
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
