/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "WaistBalancer.h"
#include <cnoid/Config>
#include <cnoid/App>
#include <cnoid/Plugin>
#include <cnoid/Archive>
#include <cnoid/TimeBar>
#include <cnoid/MessageView>
#include <cnoid/BodyMotionGenerationBar>
#include <cnoid/CheckBox>
#include <cnoid/Separator>
#include <fmt/format.h>
#include <QSpinBox>
#include <QComboBox>
#include <QTime>
#include "gettext.h"


using namespace cnoid;
using fmt::format;


namespace {

    class BalancerPanel;
    
    BodyMotionGenerationBar* bar;
    WaistBalancer* balancer;
    BalancerPanel* panel;
        
    class BalancerPanel : public BodyMotionGenerationBar::Balancer, public QWidget
    {
    public:
        QDoubleSpinBox timeToStartBalancerSpin;
        QSpinBox balancerIterationSpin;
        QCheckBox plainBalancerModeCheck;
        QComboBox boundaryConditionCombo;
        QComboBox boundarySmootherCombo;
        QDoubleSpinBox boundarySmootherTimeSpin;
        QCheckBox boundaryCmAdjustmentCheck;
        QDoubleSpinBox boundaryCmAdjustmentTimeSpin;
        QCheckBox waistHeightRelaxationCheck;
        QDoubleSpinBox gravitySpin;
        QDoubleSpinBox dynamicsTimeRatioSpin;

        QWidget* panel() { return this; }

        QHBoxLayout* newRow(QVBoxLayout* vbox) {
            QHBoxLayout* hbox = new QHBoxLayout();
            hbox->setSpacing(2);
            hbox->setContentsMargins(2, 2, 2, 2);
            vbox->addLayout(hbox);
            return hbox;
        }

        BalancerPanel() {

            QVBoxLayout* vbox = new QVBoxLayout();
            
            vbox->addSpacing(4);
            vbox->addLayout(new HSeparatorBox(new QLabel(_("Balance Adjustment"))));
            vbox->addSpacing(2);

            QHBoxLayout* hbox = newRow(vbox);
            hbox->addWidget(new QLabel(_("Starting margin")));
            timeToStartBalancerSpin.setToolTip(_("Initial time margin before applying the balancer"));
            timeToStartBalancerSpin.setDecimals(1);
            timeToStartBalancerSpin.setRange(0.0, 1.0);
            timeToStartBalancerSpin.setSingleStep(0.1);
            timeToStartBalancerSpin.setValue(0.0);
            hbox->addWidget(&timeToStartBalancerSpin);
            hbox->addWidget(new QLabel(_("[s]")));
            
            hbox->addSpacing(8);
            hbox->addWidget(new QLabel(_("Iteration")));
            balancerIterationSpin.setAlignment(Qt::AlignCenter);
            balancerIterationSpin.setRange(0, 99);
            balancerIterationSpin.setValue(2);
            hbox->addWidget(&balancerIterationSpin);

            hbox->addSpacing(8);
            plainBalancerModeCheck.setText(_("Plain-initial"));
            plainBalancerModeCheck.setToolTip(_("Initial balanced trajectory only depends on the desired ZMP"));
            hbox->addWidget(&plainBalancerModeCheck);
            hbox->addStretch();
            
            hbox = newRow(vbox);
            hbox->addWidget(new QLabel(_("Boundary")));
            boundaryConditionCombo.setToolTip(_("Boundary condition type"));
            for(int i=0; i < WaistBalancer::NUM_BOUNDARY_CONDITION_TYPES; ++i){
                boundaryConditionCombo.addItem(
                    getText(CNOID_GETTEXT_DOMAIN_NAME, WaistBalancer::boundaryConditionTypeNameOf(i)));
            }
            boundaryConditionCombo.setCurrentIndex(WaistBalancer::DEFAULT_BOUNDARY_CONDITION);
            hbox->addWidget(&boundaryConditionCombo);

            boundarySmootherCombo.setToolTip(_("Boundary smoother trajectory type"));
            for(int i=0; i < WaistBalancer::NUM_BOUNDARY_SMOOTHER_TYPES; ++i){
                boundarySmootherCombo.addItem(
                    getText(CNOID_GETTEXT_DOMAIN_NAME, WaistBalancer::boundarySmootherTypeNameOf(i)));
            }
            boundarySmootherCombo.setCurrentIndex(WaistBalancer::DEFAULT_BOUNDARY_SMOOTHER);
            hbox->addWidget(&boundarySmootherCombo);

            boundarySmootherTimeSpin.setToolTip(_("Time length of smooth connection"));
            boundarySmootherTimeSpin.setDecimals(1);
            boundarySmootherTimeSpin.setRange(0.0, 9.9);
            boundarySmootherTimeSpin.setSingleStep(0.1);
            boundarySmootherTimeSpin.setValue(0.5);
            hbox->addWidget(&boundarySmootherTimeSpin);
            hbox->addWidget(new QLabel(_("[s]")));
            hbox->addStretch();

            hbox = newRow(vbox);
            boundaryCmAdjustmentCheck.setText(_("Boundary CM adjustment"));
            boundaryCmAdjustmentCheck.setToolTip(_("Adjust the original CM trajectories around the boundaries"));
            hbox->addWidget(&boundaryCmAdjustmentCheck);

            boundaryCmAdjustmentTimeSpin.setToolTip(_("Transition time of the adjustment"));
            boundaryCmAdjustmentTimeSpin.setDecimals(1);
            boundaryCmAdjustmentTimeSpin.setRange(0.1, 9.9);
            boundaryCmAdjustmentTimeSpin.setSingleStep(0.1);
            boundaryCmAdjustmentTimeSpin.setValue(1.0);
            hbox->addWidget(&boundaryCmAdjustmentTimeSpin);
            hbox->addWidget(new QLabel(_("[s]")));
            hbox->addStretch();

            hbox = newRow(vbox);
            waistHeightRelaxationCheck.setText(_("Waist height relaxation"));
            waistHeightRelaxationCheck.setToolTip(
                _("Vertical waist position is lowered when the leg length is not enough."));
            hbox->addWidget(&waistHeightRelaxationCheck);
            hbox->addStretch();
            
            hbox = newRow(vbox);
            hbox->addWidget(new QLabel(_("Gravity")));
            gravitySpin.setDecimals(2);
            gravitySpin.setRange(0.01, 99.99);
            gravitySpin.setSingleStep(0.01);
            gravitySpin.setValue(9.80);
            hbox->addWidget(&gravitySpin);
            hbox->addWidget(new QLabel(_("[m/s^2]")));
            
            hbox->addSpacing(8);
            hbox->addWidget(new QLabel(_("Dynamics time ratio")));
            dynamicsTimeRatioSpin.setDecimals(2);
            dynamicsTimeRatioSpin.setRange(0.01, 9.99);
            dynamicsTimeRatioSpin.setSingleStep(0.01);
            dynamicsTimeRatioSpin.setValue(1.0);
            hbox->addWidget(&dynamicsTimeRatioSpin);
            hbox->addStretch();

            setLayout(vbox);
        }

        void storeState(Archive& archive){
            archive.write("timeToStartBalancer", timeToStartBalancerSpin.value());
            archive.write("balancerIterations", balancerIterationSpin.value());
            archive.write("plainBalancerMode", plainBalancerModeCheck.isChecked());
            archive.write("boundaryConditionType",
                          WaistBalancer::boundaryConditionTypeNameOf(boundaryConditionCombo.currentIndex()));
            archive.write("boundarySmootherType",
                          WaistBalancer::boundarySmootherTypeNameOf(boundarySmootherCombo.currentIndex()));
            archive.write("boundarySmootherTime", boundarySmootherTimeSpin.value());
            archive.write("boundaryCmAdjustment", boundaryCmAdjustmentCheck.isChecked());
            archive.write("boundaryCmAdjustmentTime", boundaryCmAdjustmentTimeSpin.value());
            archive.write("waistHeightRelaxation", waistHeightRelaxationCheck.isChecked());
            archive.write("gravity", gravitySpin.value());
            archive.write("dynamicsTimeRatio", dynamicsTimeRatioSpin.value());
        }

        void restoreState(const Archive& archive){
            timeToStartBalancerSpin.setValue(archive.get("timeToStartBalancer", timeToStartBalancerSpin.value()));
            balancerIterationSpin.setValue(archive.get("balancerIterations", balancerIterationSpin.value()));
            plainBalancerModeCheck.setChecked(archive.get("plainBalancerMode", plainBalancerModeCheck.isChecked()));

            boundaryConditionCombo.setCurrentIndex(
                WaistBalancer::boundaryConditionTypeOf(
                    archive.get("boundaryConditionType", boundaryConditionCombo.currentText().toStdString())));
            
            boundarySmootherCombo.setCurrentIndex(
                WaistBalancer::boundarySmootherTypeOf(
                    archive.get("boundarySmootherType", boundarySmootherCombo.currentText().toStdString())));
                
            boundarySmootherTimeSpin.setValue(archive.get("boundarySmootherTime", boundarySmootherTimeSpin.value()));
            boundaryCmAdjustmentCheck.setChecked(archive.get("boundaryCmAdjustment", boundaryCmAdjustmentCheck.isChecked()));
            boundaryCmAdjustmentTimeSpin.setValue(archive.get("boundaryCmAdjustmentTime", boundaryCmAdjustmentTimeSpin.value()));
            waistHeightRelaxationCheck.setChecked(archive.get("waistHeightRelaxation", waistHeightRelaxationCheck.isChecked()));
            gravitySpin.setValue(archive.get("gravity", gravitySpin.value()));
            dynamicsTimeRatioSpin.setValue(archive.get("dynamicsTimeRatio", dynamicsTimeRatioSpin.value()));
        }

        bool apply(BodyPtr& body, PoseProvider* provider, BodyMotionItemPtr motionItem, bool putMessages) {

            balancer->setBody(body);

            TimeBar* timeBar = TimeBar::instance();
            
            if(bar->isTimeBarRangeOnly()){
                balancer->setTimeRange(timeBar->minTime(), timeBar->maxTime());
            } else {
                balancer->setFullTimeRange();
            }
            balancer->setTimeMargins(timeToStartBalancerSpin.value(),
                                     bar->preInitialDuration(),
                                     bar->postFinalDuration());
            
            balancer->setNumIterations(balancerIterationSpin.value());
            balancer->setBoundaryConditionType(boundaryConditionCombo.currentIndex());
            balancer->setBoundarySmoother(boundarySmootherCombo.currentIndex(),
                                          boundarySmootherTimeSpin.value());
            balancer->enableBoundaryCmAdjustment(boundaryCmAdjustmentCheck.isChecked(),
                                                 boundaryCmAdjustmentTimeSpin.value());
            balancer->setInitialWaistTrajectoryMode(plainBalancerModeCheck.isChecked() ? 0 : 1);
            balancer->enableWaistHeightRelaxation(waistHeightRelaxationCheck.isChecked());
            balancer->setGravity(gravitySpin.value());
            balancer->setDynamicsTimeRatio(dynamicsTimeRatioSpin.value());
            
            MessageView* mv = MessageView::mainInstance();
            if(putMessages){
                mv->notify(format(_("Applying the waist balance filter with {} iterations ... "), balancer->numIterations()));
                mv->flush();
            }
        
            QTime time;
            time.start();
            
            motionItem->motion()->setFrameRate(timeBar->frameRate());
            bool result = balancer->apply(provider, *motionItem->motion(), bar->isSe3Enabled());
            
            if(result){
                if(putMessages){
                    mv->notify(format(_("OK ! ({} [s] consumed.)"), (time.elapsed() / 1000.0)));
                }
                motionItem->notifyUpdate();
            } else {
                if(putMessages){
                    mv->notify(_("failed."));
                }
            }
            
            return result;
        }
    };
    

    class BalancerPlugin : public Plugin
    {
    public:
        BalancerPlugin() : Plugin("Balancer") {
            require("PoseSeq");
        }

        virtual bool initialize(){

            bar = BodyMotionGenerationBar::instance();
            balancer = new WaistBalancer();
            balancer->setMessageOutputStream(mvout());
            panel = new BalancerPanel();

            bar->setBalancer(panel);

            return true;
        }

        virtual bool finalize(){

            bar->unsetBalancer();
            
            delete balancer;
            delete panel;

            return true;
        }
    };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(BalancerPlugin);
