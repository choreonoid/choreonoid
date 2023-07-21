#include "BodyMotionGenerationBar.h"
#include "PoseSeqItem.h"
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
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
#include <cnoid/LazySignal>
#include <cnoid/SpinBox>
#include <cnoid/Separator>
#include <cnoid/Buttons>
#include <cnoid/ButtonGroup>
#include <cnoid/CheckBox>
#include <cnoid/Dialog>
#include <QDialogButtonBox>
#include <fmt/format.h>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const bool TRACE_FUNCTIONS = false;

class BodyMotionGenerationSetupDialog : public Dialog
{
public:
    BodyMotionGenerationBar::Impl* impl;
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

    BodyMotionGenerationSetupDialog(BodyMotionGenerationBar::Impl* impl);
    void addSeparator(QVBoxLayout* vbox);
    void addSeparator(QVBoxLayout* vbox, QWidget* widget);
    QHBoxLayout* newRow(QVBoxLayout* vbox);
    void setAsParameterWidget(QAbstractButton& button);
    void setAsParameterWidget(DoubleSpinBox& spin);
    void onStepAdjustmentModeChanged(int mode);
    void storeState(Archive& archive);
    void restoreState(const Archive& archive);
};

}

namespace cnoid {

class BodyMotionGenerationBar::Impl
{
public:
    BodyMotionGenerationBar* self;
    unique_ptr<BodyMotionPoseProvider> bodyMotionPoseProvider;
    unique_ptr<PoseProviderToBodyMotionConverter> poseProviderToBodyMotionConverter;
    unique_ptr<BodyMotionPoseProvider> secondBodyMotionPoseProvider;
    Balancer* balancer;
    TimeBar* timeBar;
    BodyMotionGenerationSetupDialog* setup;
    LazySignal< Signal<void()> >sigInterpolationParametersChanged;
    Action* autoGenerationForNewBodyCheck;
    QIcon balancerIcon;
    ToolButton* balancerToggle;
    ToolButton* secondBalancerToggle;
    ToolButton* autoGenerationToggle;

    struct ExtraMotionFilter
    {
        string key;
        ToolButton* toggleButton;
        std::function<bool(BodyItem* bodyItem, BodyMotionItem* motionItem)> function;
        
        ExtraMotionFilter(
            const char* key,
            ToolButton* button,
            const std::function<bool(BodyItem* bodyItem, BodyMotionItem* motionItem)>& func)
            : key(key), toggleButton(button), function(func) { }
    };
    vector<ExtraMotionFilter> extraMotionFilters;

    Impl(BodyMotionGenerationBar* self);
    void updateBalancerToggles();
    void onGenerationButtonClicked();
    bool shapeBodyMotion(
        BodyItem* bodyItem, PoseProvider* provider, BodyMotionItem* motionItem, bool putMessages);
    bool shapeBodyMotionWithSimpleInterpolation(
        BodyItem* bodyItem, PoseProvider* provider, BodyMotionItem* outputMotionItem);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
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
            bar->impl->autoGenerationForNewBodyCheck =
                mm.addCheckItem(_("Automatic Generation for a New Body"));
        } else {
            bar->impl->autoGenerationForNewBodyCheck = new Action;
        }
        bar->impl->autoGenerationForNewBodyCheck->setChecked(true);

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
    impl = new Impl(this);
}


BodyMotionGenerationBar::Impl::Impl(BodyMotionGenerationBar* self)
    : self(self)
{
    timeBar = TimeBar::instance();
    setup = new BodyMotionGenerationSetupDialog(this);
    balancer = nullptr;

    auto generationButton = self->addButton(QIcon(":/PoseSeq/icon/trajectory-generation.svg"));
    generationButton->setToolTip(_("Generate body motions"));
    generationButton->sigClicked().connect([this](){ onGenerationButtonClicked(); });

    autoGenerationToggle = self->addToggleButton(QIcon(":/PoseSeq/icon/auto-update.svg"));
    autoGenerationToggle->setToolTip(_("Automatic Balance Adjustment Mode"));
    autoGenerationToggle->setChecked(false);

    balancerIcon = QIcon(":/PoseSeq/icon/balancer.svg");
    
    balancerToggle = self->addToggleButton(balancerIcon);
    balancerToggle->setToolTip(_("Enable the balancer"));
    balancerToggle->setEnabled(false);
    balancerToggle->setChecked(false);

    secondBalancerToggle = nullptr;

    self->addButton(QIcon(":/Base/icon/setup.svg"))
        ->sigClicked().connect([this](){ setup->show(); });
}


BodyMotionGenerationBar::~BodyMotionGenerationBar()
{
    delete impl;
}


void BodyMotionGenerationBar::setBalancer(Balancer* balancer)
{
    impl->balancer = balancer;
    impl->updateBalancerToggles();
    if(balancer){
        impl->setup->vbox->addWidget(balancer->panel());
    }
}


void BodyMotionGenerationBar::unsetBalancer()
{
    impl->updateBalancerToggles();
    if(impl->balancer){
        impl->setup->layout()->removeWidget(impl->balancer->panel());
        impl->balancer = nullptr;
    }
}


void BodyMotionGenerationBar::Impl::updateBalancerToggles()
{
    bool on = balancer != nullptr;
    
    balancerToggle->setEnabled(on);

    if(!extraMotionFilters.empty()){
        if(!secondBalancerToggle){
            self->setInsertionPosition(3 + extraMotionFilters.size());
            secondBalancerToggle = self->addToggleButton(balancerIcon);
            secondBalancerToggle->setToolTip(_("Enable the second balancer"));
            secondBalancerToggle->setChecked(balancerToggle->isChecked());
        }
        if(on){
            secondBalancerToggle->setEnabled(on);
        }
    }
}    


void BodyMotionGenerationBar::addMotionFilter
(const char* key, const QIcon& buttonIcon, const char* toolTip,
 std::function<bool(BodyItem* bodyItem, BodyMotionItem* motionItem)> filter)
{
    setInsertionPosition(3);
    auto button = addToggleButton(buttonIcon);
    button->setToolTip(toolTip);
    impl->extraMotionFilters.emplace_back(key, button, filter);
    impl->updateBalancerToggles();
}


SignalProxy<void()> BodyMotionGenerationBar::sigInterpolationParametersChanged()
{
    return impl->sigInterpolationParametersChanged.signal();
}


void BodyMotionGenerationBar::Impl::onGenerationButtonClicked()
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

    for(auto it = motionItems.begin(); it != motionItems.end(); ++it){
        BodyMotionItem* motionItem = *it;
        if(auto bodyItem = motionItem->findOwnerItem<BodyItem>(true)){
            PoseProvider* provider = nullptr;
            if(auto poseSeqItem = motionItem->parentItem<PoseSeqItem>()){
                provider = poseSeqItem->interpolator().get();
            } else {
                if(balancerToggle->isChecked() && balancer){
                    if(!bodyMotionPoseProvider){
                        bodyMotionPoseProvider = make_unique<BodyMotionPoseProvider>();
                    }
                    if(bodyMotionPoseProvider->setMotion(bodyItem->body(), motionItem->motion())){
                        provider = bodyMotionPoseProvider.get();
                    }
                }
                if(setup->newBodyMotionItemCheck.isChecked()){
                    BodyMotionItem* newMotionItem;
                    if(provider){
                        newMotionItem = new BodyMotionItem;
                    } else {
                        newMotionItem = static_cast<BodyMotionItem*>(motionItem->clone());
                    }
                    newMotionItem->setTemporary(true);
                    newMotionItem->setName(motionItem->name() + "'");
                    motionItem->parentItem()->insertChild(motionItem->nextItem(), newMotionItem);
                    motionItem = newMotionItem;
                }
            }
            shapeBodyMotion(bodyItem, provider, motionItem, true);
        }
    }
}


bool BodyMotionGenerationBar::shapeBodyMotion
(BodyItem* bodyItem, PoseProvider* provider, BodyMotionItem* outputMotionItem, bool putMessages)
{
    return impl->shapeBodyMotion(bodyItem, provider, outputMotionItem, putMessages);
}


bool BodyMotionGenerationBar::Impl::shapeBodyMotion
(BodyItem* bodyItem, PoseProvider* provider, BodyMotionItem* motionItem, bool putMessages)
{
    bool isProcessed = false;
    bool failed = false;
    
    if(provider){
        if(balancerToggle->isChecked() && balancer){
            if(balancer->apply(bodyItem, provider, motionItem, putMessages)){
                isProcessed = true;
            } else {
                failed = true;
            }
        } else {
            if(shapeBodyMotionWithSimpleInterpolation(bodyItem, provider, motionItem)){
                isProcessed = true;
            } else {
                failed = true;
            }
        }
    }

    bool isExtraFilterApplied = false;
    if(!failed){
        for(auto& filter : extraMotionFilters){
            if(filter.toggleButton->isChecked()){
                if(filter.function(bodyItem, motionItem)){
                    isProcessed = true;
                    isExtraFilterApplied = true;
                } else {
                    failed = true;
                    break;
                }
            }
        }
    }

    if(!failed && isExtraFilterApplied && secondBalancerToggle->isChecked()){
        if(!secondBodyMotionPoseProvider){
            secondBodyMotionPoseProvider = make_unique<BodyMotionPoseProvider>();
        }
        secondBodyMotionPoseProvider->setMotion(bodyItem->body(), motionItem->motion());
        if(!balancer->apply(bodyItem, secondBodyMotionPoseProvider.get(), motionItem, putMessages)){
            failed = true;
        }
    }

    if(isProcessed){
        motionItem->notifyUpdate();
    } else {
        MessageView::instance()->putln(
            format(_("{0} cannot be a target of the body motion generation."), motionItem->name()),
            MessageView::Error);
    }

    return isProcessed && !failed;
}
        

bool BodyMotionGenerationBar::Impl::shapeBodyMotionWithSimpleInterpolation
(BodyItem* bodyItem, PoseProvider* provider, BodyMotionItem* outputMotionItem)
{
    if(!poseProviderToBodyMotionConverter){
        poseProviderToBodyMotionConverter = make_unique<PoseProviderToBodyMotionConverter>();
    }
    if(setup->onlyTimeBarRangeCheck.isChecked()){
        poseProviderToBodyMotionConverter->setTimeRange(timeBar->minTime(), timeBar->maxTime());
    } else {
        poseProviderToBodyMotionConverter->setFullTimeRange();
    }
    poseProviderToBodyMotionConverter->setAllLinkPositionOutput(setup->se3Check.isChecked());
        
    auto motion = outputMotionItem->motion();
    motion->setFrameRate(timeBar->frameRate());

    return poseProviderToBodyMotionConverter->convert(bodyItem->body(), provider, *motion);
}


bool BodyMotionGenerationBar::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool BodyMotionGenerationBar::Impl::storeState(Archive& archive)
{
    archive.write("auto_generation", autoGenerationToggle->isChecked());
    archive.write("auto_generation_for_new_body", autoGenerationForNewBodyCheck->isChecked());
    archive.write("balancer", balancerToggle->isChecked());
    if(secondBalancerToggle){
        archive.write("second_balancer", secondBalancerToggle->isChecked());
    }

    for(auto& filter : extraMotionFilters){
        archive.write(filter.key, filter.toggleButton->isChecked());
    }
    
    setup->storeState(archive);
    
    if(balancer){
        balancer->storeState(&archive);
    }
    return true;
}


bool BodyMotionGenerationBar::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool BodyMotionGenerationBar::Impl::restoreState(const Archive& archive)
{
    autoGenerationToggle->setChecked(
        archive.get({ "auto_generation", "autoGeneration" }, autoGenerationToggle->isChecked()));

    autoGenerationForNewBodyCheck->setChecked(
        archive.get({ "auto_generation_for_new_body", "autoGenerationForNewBody" },
                    autoGenerationForNewBodyCheck->isChecked()));
        
    balancerToggle->setChecked(archive.get("balancer", balancerToggle->isChecked()));
    if(secondBalancerToggle){
        secondBalancerToggle->setChecked(archive.get("second_balancer", secondBalancerToggle->isChecked()));
    }

    bool on;
    for(auto& filter : extraMotionFilters){
        if(archive.read(filter.key, on)){
            filter.toggleButton->setChecked(on);
        }
    }
    
    setup->restoreState(archive);
    if(balancer){
        balancer->restoreState(&archive);
    }
    return true;
}


bool BodyMotionGenerationBar::isBalancerEnabled() const
{
    return impl->balancerToggle->isChecked();
}
            
bool BodyMotionGenerationBar::isAutoGenerationMode() const
{
    return impl->autoGenerationToggle->isChecked();
}

bool BodyMotionGenerationBar::isAutoGenerationForNewBodyEnabled() const
{
    return impl->autoGenerationForNewBodyCheck->isChecked();
}

double BodyMotionGenerationBar::timeScaleRatio() const
{
    return impl->setup->timeScaleRatioSpin.value();
}

double BodyMotionGenerationBar::preInitialDuration() const
{
    return impl->setup->preInitialDurationSpin.value();
}

double BodyMotionGenerationBar::postFinalDuration() const
{
    return impl->setup->postFinalDurationSpin.value();
}

bool BodyMotionGenerationBar::isTimeBarRangeOnly() const
{
    return impl->setup->onlyTimeBarRangeCheck.isChecked();
}

int BodyMotionGenerationBar::stepTrajectoryAdjustmentMode() const
{
    return impl->setup->stepAdjustmentRadioGroup.checkedId();
}

double BodyMotionGenerationBar::stealthyHeightRatioThresh() const
{
    return impl->setup->stealthyHeightRatioThreshSpin.value();
}

double BodyMotionGenerationBar::flatLiftingHeight() const
{
    return impl->setup->flatLiftingHeightSpin.value();
}

double BodyMotionGenerationBar::flatLandingHeight() const
{
    return impl->setup->flatLandingHeightSpin.value();
}

double BodyMotionGenerationBar::impactReductionHeight() const
{
    return impl->setup->impactReductionHeightSpin.value();
}

double BodyMotionGenerationBar::impactReductionTime() const
{
    return impl->setup->impactReductionTimeSpin.value();
}

double BodyMotionGenerationBar::toeContactTime() const
{
    return impl->setup->toeContactTimeSpin.value();
}

double BodyMotionGenerationBar::toeContactAngle() const
{
    return radian(impl->setup->toeContactAngleSpin.value());
}

bool BodyMotionGenerationBar::isAutoZmpAdjustmentMode() const
{
    return impl->setup->autoZmpCheck.isChecked();
}

double BodyMotionGenerationBar::minZmpTransitionTime() const
{
    return impl->setup->minZmpTransitionTimeSpin.value();
}

double BodyMotionGenerationBar::zmpCenteringTimeThresh() const
{
    return impl->setup->zmpCenteringTimeThreshSpin.value();
}

double BodyMotionGenerationBar::zmpTimeMarginBeforeLifting() const
{
    return impl->setup->zmpTimeMarginBeforeLiftingSpin.value();
}

double BodyMotionGenerationBar::zmpMaxDistanceFromCenter() const
{
    return impl->setup->zmpMaxDistanceFromCenterSpin.value();
}

bool BodyMotionGenerationBar::isSe3Enabled() const
{
    return impl->setup->se3Check.isChecked();
}

bool BodyMotionGenerationBar::isLipSyncMixMode() const
{
    return impl->setup->lipSyncMixCheck.isChecked();
}


BodyMotionGenerationSetupDialog::BodyMotionGenerationSetupDialog(BodyMotionGenerationBar::Impl* impl)
    : impl(impl)
{
    setWindowTitle(_("Body Motion Generation Setup"));

    vbox = new QVBoxLayout;

    QHBoxLayout* hbox = newRow(vbox);
    hbox->addWidget(new QLabel(_("Time scale")));
    timeScaleRatioSpin.setDecimals(2);
    timeScaleRatioSpin.setRange(0.01, 9.99);
    timeScaleRatioSpin.setSingleStep(0.01);
    timeScaleRatioSpin.setValue(1.0);
    setAsParameterWidget(timeScaleRatioSpin);
    hbox->addWidget(&timeScaleRatioSpin);

    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Pre-initial")));
    preInitialDurationSpin.setDecimals(1);
    preInitialDurationSpin.setRange(0.0, 9.9);
    preInitialDurationSpin.setSingleStep(0.1);
    preInitialDurationSpin.setValue(1.0);
    setAsParameterWidget(preInitialDurationSpin);
    hbox->addWidget(&preInitialDurationSpin);
    hbox->addWidget(new QLabel(_("[s]")));

    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Post-final")));
    postFinalDurationSpin.setDecimals(1);
    postFinalDurationSpin.setRange(0.0, 9.9);
    postFinalDurationSpin.setSingleStep(0.1);
    postFinalDurationSpin.setValue(1.0);
    setAsParameterWidget(postFinalDurationSpin);
    hbox->addWidget(&postFinalDurationSpin);
    hbox->addWidget(new QLabel(_("[s]")));
    hbox->addStretch();

    hbox = newRow(vbox);
    onlyTimeBarRangeCheck.setText(_("Time bar's range only"));
    onlyTimeBarRangeCheck.setChecked(false);
    setAsParameterWidget(onlyTimeBarRangeCheck);
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
    setAsParameterWidget(noStepAdjustmentRadio);
    hbox->addWidget(&noStepAdjustmentRadio);

    stealthyStepRadio.setText(_("Stealthy steps"));
    stealthyStepRadio.setToolTip(_("The mode to make foot lifting and landing motions smoother to increase the stability"));
    setAsParameterWidget(stealthyStepRadio);
    hbox->addWidget(&stealthyStepRadio);

    toeStepRadio.setText(_("Toe steps"));
    toeStepRadio.setToolTip(_("The mode to use the toes in foot lifting and landing to increase the stability"));
    setAsParameterWidget(toeStepRadio);
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
    setAsParameterWidget(stealthyHeightRatioThreshSpin);
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
    setAsParameterWidget(flatLiftingHeightSpin);
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
    setAsParameterWidget(flatLandingHeightSpin);
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
    setAsParameterWidget(impactReductionHeightSpin);
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
    setAsParameterWidget(impactReductionTimeSpin);
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
    setAsParameterWidget(toeContactAngleSpin);
    toeStepWidgets.push_back(&toeContactAngleSpin);
    hbox->addWidget(&toeContactAngleSpin);
    label = new QLabel(_("[deg]"));
    toeStepWidgets.push_back(label);
    hbox->addWidget(label);
    hbox->addSpacing(8);

    label = new QLabel(_("Toe contact time"));
    toeStepWidgets.push_back(label);
    hbox->addWidget(label);
    toeContactTimeSpin.setAlignment(Qt::AlignCenter);
    toeContactTimeSpin.setDecimals(3);
    toeContactTimeSpin.setRange(0.001, 0.999);
    toeContactTimeSpin.setSingleStep(0.001);
    toeContactTimeSpin.setValue(0.1);
    setAsParameterWidget(toeContactTimeSpin);
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
    setAsParameterWidget(autoZmpCheck);

    hbox = newRow(vbox);
    hbox->addWidget(new QLabel(_("Min. transtion time")));
    minZmpTransitionTimeSpin.setDecimals(2);
    minZmpTransitionTimeSpin.setRange(0.01, 0.99);
    minZmpTransitionTimeSpin.setSingleStep(0.01);
    minZmpTransitionTimeSpin.setValue(0.1);
    setAsParameterWidget(minZmpTransitionTimeSpin);
    hbox->addWidget(&minZmpTransitionTimeSpin);
    hbox->addWidget(new QLabel(_("[s]")));

    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Centering time thresh")));
    zmpCenteringTimeThreshSpin.setDecimals(3);
    zmpCenteringTimeThreshSpin.setRange(0.001, 0.999);
    zmpCenteringTimeThreshSpin.setSingleStep(0.001);
    zmpCenteringTimeThreshSpin.setValue(0.03);
    setAsParameterWidget(zmpCenteringTimeThreshSpin);
    hbox->addWidget(&zmpCenteringTimeThreshSpin);
    hbox->addWidget(new QLabel(_("[s]")));
    hbox->addStretch();

    hbox = newRow(vbox);
    hbox->addWidget(new QLabel(_("Time margin before lifting")));
    zmpTimeMarginBeforeLiftingSpin.setDecimals(3);
    zmpTimeMarginBeforeLiftingSpin.setRange(0.0, 0.999);
    zmpTimeMarginBeforeLiftingSpin.setSingleStep(0.001);
    zmpTimeMarginBeforeLiftingSpin.setValue(0.0);
    setAsParameterWidget(zmpTimeMarginBeforeLiftingSpin);
    hbox->addWidget(&zmpTimeMarginBeforeLiftingSpin);
    hbox->addWidget(new QLabel(_("[s]")));

    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Max distance from center")));
    zmpMaxDistanceFromCenterSpin.setDecimals(3);
    zmpMaxDistanceFromCenterSpin.setRange(0.001, 0.999);
    zmpMaxDistanceFromCenterSpin.setSingleStep(0.001);
    zmpMaxDistanceFromCenterSpin.setValue(0.02);
    setAsParameterWidget(zmpMaxDistanceFromCenterSpin);
    hbox->addWidget(&zmpMaxDistanceFromCenterSpin);
    hbox->addWidget(new QLabel(_("[m]")));
    hbox->addStretch();

    addSeparator(vbox);
    hbox = newRow(vbox);

    lipSyncMixCheck.setText(_("Mix lip-sync motion"));
    lipSyncMixCheck.setChecked(false);
    setAsParameterWidget(lipSyncMixCheck);
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


void BodyMotionGenerationSetupDialog::addSeparator(QVBoxLayout* vbox)
{
    vbox->addSpacing(4);
    vbox->addWidget(new HSeparator);
    vbox->addSpacing(2);
}


void BodyMotionGenerationSetupDialog::addSeparator(QVBoxLayout* vbox, QWidget* widget)
{
    vbox->addSpacing(4);
    vbox->addLayout(new HSeparatorBox(widget));
    vbox->addSpacing(2);
}


QHBoxLayout* BodyMotionGenerationSetupDialog::newRow(QVBoxLayout* vbox)
{
    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->setSpacing(2);
    hbox->setContentsMargins(2, 2, 2, 2);
    vbox->addLayout(hbox);
    return hbox;
}


void BodyMotionGenerationSetupDialog::setAsParameterWidget(QAbstractButton& button)
{
    connect(&button, &QAbstractButton::toggled,
            [this](bool){ impl->sigInterpolationParametersChanged.request(); });
}
            

void BodyMotionGenerationSetupDialog::setAsParameterWidget(DoubleSpinBox& spin)
{
    connect(&spin, (void(QDoubleSpinBox::*)(double)) &QDoubleSpinBox::valueChanged,
            [this](double){ impl->sigInterpolationParametersChanged.request(); });
}


void BodyMotionGenerationSetupDialog::onStepAdjustmentModeChanged(int mode)
{
    for(auto& widget : stealthyStepWidgets){
        widget->setEnabled(mode == PoseSeqInterpolator::StealthyStepMode);
    }
    for(auto& widget : toeStepWidgets){
        widget->setEnabled(mode == PoseSeqInterpolator::ToeStepMode);
    }
}

void BodyMotionGenerationSetupDialog::storeState(Archive& archive)
{
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


void BodyMotionGenerationSetupDialog::restoreState(const Archive& archive)
{
    timeScaleRatioSpin.setValue(archive.get("timeScaleRatio", timeScaleRatioSpin.value()));
    preInitialDurationSpin.setValue(archive.get("preInitialDuration", preInitialDurationSpin.value()));
    postFinalDurationSpin.setValue(archive.get("postFinalDuration", postFinalDurationSpin.value()));
    onlyTimeBarRangeCheck.setChecked(archive.get("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked()));
    newBodyMotionItemCheck.setChecked(
        archive.get({ "output_new_body_motion_item", "makeNewBodyMotionItem" },
                    newBodyMotionItemCheck.isChecked()));

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
    zmpMaxDistanceFromCenterSpin.setValue(
        archive.get("zmpMaxDistanceFromCenter", zmpMaxDistanceFromCenterSpin.value()));            
            
        se3Check.setChecked(archive.get("allLinkPositions", se3Check.isChecked()));
        lipSyncMixCheck.setChecked(archive.get("lipSyncMix", lipSyncMixCheck.isChecked()));
}
