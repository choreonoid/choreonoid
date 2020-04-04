/**
   \author Shin'ichiro Nakaoka
*/

#include "JointDisplacementView.h"
#include "BodySelectionManager.h"
#include "BodyItem.h"
#include <cnoid/DisplayedValueFormatManager>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/Buttons>
#include <cnoid/SpinBox>
#include <cnoid/Slider>
#include <cnoid/Dial>
#include <cnoid/LazyCaller>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/RootItem>
#include <cnoid/MathUtil>
#include <QLabel>
#include <QGridLayout>
#include <QScrollArea>
#include <QKeyEvent>
#include <QStyle>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const QString normalStyle("font-weight: normal");
const QString warningStyle("font-weight: bold; color: red");
const QString handleNormalStyle("");
const QString handleWarningStyle("QSlider::handle:horizontal {background-color: red;}");

class JointIndicator;

/**
   Slider resolution.
   This value should not be increased more because QDial won't work correctly in that case.
*/
const double resolution = 100000.0;

}

namespace cnoid {

class JointDisplacementView::Impl : public QObject
{
public:
    JointDisplacementView* self;

    BodySelectionManager* bodySelectionManager;
    BodyItemPtr currentBodyItem;
    vector<int> activeJointIds;
    vector<JointIndicator*> jointIndicators;
    LazyCaller updateJointDisplacementsLater;
            
    ScopedConnection bodySelectionManagerConnection;
    ScopedConnection linkSelectionChangeConnection;
    ScopedConnection kinematicStateChangeConnection;

    DisplayedValueFormatManager* valueFormatManager;
    ScopedConnection valueFormatManagerConnection;
    int lengthUnit;
    int lengthDecimals;
    double defaultMaxLength;
    double lengthStep;
    int angleUnit;
    int angleDecimals;
    double defaultMaxAngle;
    double angleStep;

    QLabel targetLabel;
    Action* selectedJointsOnlyCheck;
    Action* jointIdCheck;
    Action* jointNameCheck;
    Action* overlapJointNameCheck;
    Action* sliderCheck;
    Action* dialCheck;
    Action* phaseCheck;
            
    QScrollArea scrollArea;
    QWidget gridBase;
    QGridLayout grid;

    ToolButton menuButton;
    MenuManager menuManager;

    Impl(JointDisplacementView* self);
    ~Impl();
    void createPanel();
    void createOptionMenu();
    void onActivated();
    void onMenuButtonClicked();
    void onCurrentBodyItemChanged(BodyItem* bodyItem);
    void setBodyItem(BodyItem* bodyItem);
    void updateIndicatorGrid();
    void initializeIndicators(int num);
    void onNumColumnsChanged(int n);
    void onUnitChanged();
    bool eventFilter(QObject* object, QEvent* event);
    bool onSliderKeyPressEvent(Slider* slider, QKeyEvent* event);
    void focusSlider(int index);
    bool onDialKeyPressEvent(Dial* dial, QKeyEvent* event);
    void focusDial(int index);
    void notifyJointDisplacementInput();
    void updateJointDisplacements();
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
    void restoreCurrentBodyItem(const Archive& archive);
};

}

namespace {
    
class JointIndicator
{
public:
    JointDisplacementView::Impl* viewImpl;
    int index;
    Link* joint;
    double unitConversionRatio;
    DoubleSpinBox spin;
    Slider slider;
    Dial dial;
    bool isWarningState;
    SpinBox phaseSpin;
    int minPhase;
    int maxPhase;
    QLabel idLabel;
    QLabel nameLabel;
    QLabel lowerLimitLabel;
    QLabel upperLimitLabel;

    JointIndicator(JointDisplacementView::Impl* viewImpl, int index)
        : viewImpl(viewImpl),
          index(index),
          joint(nullptr),
          spin(&viewImpl->gridBase),
          slider(Qt::Horizontal, &viewImpl->gridBase),
          isWarningState(false),
          dial(&viewImpl->gridBase),
          phaseSpin(&viewImpl->gridBase),
          idLabel(&viewImpl->gridBase),
          nameLabel(&viewImpl->gridBase),
          lowerLimitLabel(&viewImpl->gridBase),
          upperLimitLabel(&viewImpl->gridBase)
    {
        idLabel.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        nameLabel.setAlignment(Qt::AlignCenter);
        nameLabel.setTextInteractionFlags(Qt::TextSelectableByMouse);
        lowerLimitLabel.setAlignment(Qt::AlignCenter);
        upperLimitLabel.setAlignment(Qt::AlignCenter);
            
        spin.setAlignment(Qt::AlignCenter);
        spin.sigValueChanged().connect(
            [=](double v){ onDisplacementInput(v); });
            
        slider.setSingleStep(0.1 * resolution);
        slider.setProperty("JointSliderIndex", index);
        slider.installEventFilter(viewImpl);
        slider.sigValueChanged().connect(
            [=](int v){ onDisplacementInput(v / resolution); });

        dial.setSingleStep(0.1 * resolution);
        dial.setProperty("JointDialIndex", index);
        dial.installEventFilter(viewImpl);
        dial.sigValueChanged().connect(
            [=](int v){ onDisplacementInput(v / resolution); });

        phaseSpin.sigValueChanged().connect(
            [=](int v){ onPhaseInput(v); });

        phaseSpin.setPrefix("T");
        phaseSpin.setRange(0, 0);
    }

    int attachTo(QGridLayout& grid, int row, int col, bool overlapJointName)
    {
        if(overlapJointName){
            grid.addWidget(&nameLabel, row++, col, 1, 6);
            grid.addWidget(&idLabel, row, col++);
        } else {
            grid.addWidget(&idLabel, row, col++);
            grid.addWidget(&nameLabel, row, col++);
        }
        grid.addWidget(&spin, row, col++);
        grid.addWidget(&lowerLimitLabel, row, col++);
        grid.addWidget(&slider, row, col++);
        grid.addWidget(&upperLimitLabel, row, col++);
        grid.addWidget(&dial, row, col++);
        grid.addWidget(&phaseSpin, row, col++);

        return row + 1;
    }

    void setNextTabOrderIndicator(JointIndicator* next)
    {
        QWidget::setTabOrder(&spin, &next->spin);
        QWidget::setTabOrder(&slider, &next->slider);
        QWidget::setTabOrder(&dial, &next->dial);
        QWidget::setTabOrder(&phaseSpin, &next->phaseSpin);
    }

    bool setRangeLabelValues(double lower, double upper, int precision)
    {
        bool isValid = true;
        int lowerDigits = static_cast<int>(log10(fabs(lower))) + 1;
        if(lowerDigits > 5){
            lowerLimitLabel.setText("*");
            isValid = false;
        } else {
            lowerLimitLabel.setText(QString::number(lower, 'f', precision));
        }
        int upperDigits = static_cast<int>(log10(fabs(upper))) + 1;
        if(upperDigits > 5){
            upperLimitLabel.setText("*");
            isValid = false;
        } else {
            upperLimitLabel.setText(QString::number(upper, 'f', precision));
        }
        return isValid;
    }

    void initialize(Link* joint)
    {
        this->joint = joint;
        
        idLabel.setText(QString("%1:").arg(joint->jointId()));
        idLabel.setVisible(viewImpl->jointIdCheck->isChecked());

        nameLabel.setText(joint->name().c_str());
        nameLabel.setVisible(viewImpl->jointNameCheck->isChecked());

        spin.show();

        bool on = viewImpl->sliderCheck->isChecked();
        lowerLimitLabel.setVisible(on);
        slider.setVisible(on);
        upperLimitLabel.setVisible(on);
        dial.setVisible(viewImpl->dialCheck->isChecked());

        unitConversionRatio = 1.0;
        double lower = joint->q_lower();
        double upper = joint->q_upper();

        slider.blockSignals(true);
        spin.blockSignals(true);
        dial.blockSignals(true);
        phaseSpin.blockSignals(true);

        if(joint->isRevoluteJoint()){
            bool isValidRange = true;
            if(viewImpl->angleUnit == DisplayedValueFormatManager::Degree){
                unitConversionRatio = 180.0 / PI;
                lower *= unitConversionRatio;
                upper *= unitConversionRatio;
                if(!setRangeLabelValues(lower, upper, 0)){
                    lower = -180.0;
                    upper = 180.0;
                    isValidRange = false;
                }
            } else {
                if(!setRangeLabelValues(lower, upper, viewImpl->angleDecimals)){
                    lower = -M_PI;
                    upper = M_PI;
                    isValidRange = false;
                }
            }
            slider.setRange(lower * resolution, upper * resolution);
            slider.setEnabled(true);

            spin.setDecimals(viewImpl->angleDecimals);
            spin.setRange(-viewImpl->defaultMaxAngle, viewImpl->defaultMaxAngle);
            spin.setSingleStep(viewImpl->angleStep);
            spin.setEnabled(true);

            if(!isValidRange){
                dial.setWrapping(true);
                dial.setNotchesVisible(false);
            } else {
                dial.setWrapping(false);
                dial.setNotchesVisible(true);
            }
            dial.setRange(lower * resolution, upper * resolution);
            dial.setEnabled(true);

        } else if(joint->isPrismaticJoint()){
            if(viewImpl->lengthUnit == DisplayedValueFormatManager::Millimeter){
                unitConversionRatio = 1000.0;
                lower *= unitConversionRatio;
                upper *= unitConversionRatio;
                if(!setRangeLabelValues(lower, upper, viewImpl->lengthDecimals)){
                    lower = -1000.0;
                    upper = 1000.0;
                }
            } else {
                if(!setRangeLabelValues(lower, upper, viewImpl->lengthDecimals)){
                    lower = -1.0;
                    upper = 1.0;
                }
            }
            slider.setRange(lower * resolution, upper * resolution);
            slider.setEnabled(true);
            
            spin.setDecimals(viewImpl->lengthDecimals);
            spin.setRange(-viewImpl->defaultMaxLength, viewImpl->defaultMaxLength);
            spin.setSingleStep(viewImpl->lengthStep);
            spin.setEnabled(true);
            
        } else {
            slider.setRange(0, 0);
            setRangeLabelValues(0.0, 0.0, 0);
            slider.setEnabled(false);
            spin.setDecimals(0);
            spin.setRange(0.0, 0.0);
            spin.setEnabled(false);
        }
            
        bool hasPhases = false;
        if(viewImpl->phaseCheck->isChecked()){
            minPhase = 0;
            maxPhase = 0;
            double q0 = joint->q_initial();
            if((joint->q_upper() - q0) > M_PI){
                maxPhase = 1 + trunc((joint->q_upper() - q0 - M_PI) / (2.0 * M_PI));
                hasPhases = true;
            }
            if((joint->q_lower() - q0) < -M_PI){
                minPhase = -1 + trunc((joint->q_lower() - q0 + M_PI) / (2.0 * M_PI));
                hasPhases = true;
            }
            phaseSpin.setRange(minPhase, maxPhase);
        }
        phaseSpin.setVisible(hasPhases);

        spin.blockSignals(false);
        slider.blockSignals(false);
        dial.blockSignals(false);
        phaseSpin.blockSignals(false);

        updateDisplacement(true);
    }

    void updateDisplacement(bool forceUpdate)
    {
        double q = joint->q();
        double v = unitConversionRatio * q;

        if(forceUpdate || v != spin.value()){

            if(q > joint->q_upper() || q < joint->q_lower()){
                spin.setStyleSheet(warningStyle);
                slider.setStyleSheet(handleWarningStyle);
                if(q > joint->q_upper()){
                    upperLimitLabel.setStyleSheet(warningStyle);
                } else {
                    upperLimitLabel.setStyleSheet(normalStyle);
                }
                if(q < joint->q_lower()){
                    lowerLimitLabel.setStyleSheet(warningStyle);
                } else {
                    lowerLimitLabel.setStyleSheet(normalStyle);
                }
                isWarningState = true;
                
            } else if(isWarningState){
                spin.setStyleSheet(normalStyle);
                slider.setStyleSheet(handleNormalStyle);
                lowerLimitLabel.setStyleSheet(normalStyle);
                upperLimitLabel.setStyleSheet(normalStyle);
                isWarningState = false;
            }
            
            spin.blockSignals(true);
            if(v > spin.maximum()){
                spin.setRange(spin.minimum(), v);
            } else if(v < spin.minimum()){
                spin.setRange(v, spin.maximum());
            }
            spin.setValue(v);
            spin.blockSignals(false);

            if(slider.isVisible()){
                slider.blockSignals(true);
                slider.setValue(v * resolution);
                slider.blockSignals(false);
            }
            if(dial.isVisible()){
                dial.blockSignals(true);
                dial.setValue(v * resolution);
                dial.blockSignals(false);
            }
            if(phaseSpin.isVisible()){
                phaseSpin.blockSignals(true);
                int phase = getCurrentPhase();
                int currentMinPhase = minPhase;
                int currentMaxPhase = maxPhase;
                if(joint->q() - 2.0 * M_PI < joint->q_lower() && phase > minPhase){
                    ++currentMinPhase;
                }
                if(joint->q() + 2.0 * M_PI > joint->q_upper() && phase < maxPhase){
                    --currentMaxPhase;
                }
                phaseSpin.setRange(currentMinPhase, currentMaxPhase);
                phaseSpin.setValue(phase);
                phaseSpin.blockSignals(false);
            }
        }
    }

    int getCurrentPhase()
    {        
        int phase = 0;
        double q0 = joint->q_initial();
        if((joint->q() - q0) > M_PI){
            phase = 1 + trunc((joint->q() - q0 - M_PI) / (2.0 * M_PI));
        } else if((joint->q() - q0) < -M_PI){
            phase = -1 + trunc((joint->q() - q0 + M_PI) / (2.0 * M_PI));
        }
        return phase;
    }

    void onDisplacementInput(double value)
    {
        joint->q() = value / unitConversionRatio;
        updateDisplacement(true);
        viewImpl->notifyJointDisplacementInput();
    }

    void onPhaseInput(int phase)
    {
        int currentPhase = getCurrentPhase();
        joint->q() += (phase - currentPhase) * 2.0 * M_PI;
        updateDisplacement(true);
        viewImpl->notifyJointDisplacementInput();
    }

    void removeWidgesFrom(QGridLayout& grid)
    {
        grid.removeWidget(&idLabel);
        grid.removeWidget(&nameLabel);
        grid.removeWidget(&spin);
        grid.removeWidget(&lowerLimitLabel);
        grid.removeWidget(&slider);
        grid.removeWidget(&upperLimitLabel);
        grid.removeWidget(&dial);
        grid.removeWidget(&phaseSpin);
    }
};

}


void JointDisplacementView::initializeClass(ExtensionManager* ext)
{
    auto& vm = ext->viewManager();
    vm.registerClass<JointDisplacementView>(
        "JointDisplacementView", N_("Joint Displacement"), ViewManager::SINGLE_DEFAULT);
    vm.registerClassAlias("JointSliderView", "JointDisplacementView");
}


JointDisplacementView::JointDisplacementView()
{
    impl = new Impl(this);
}


JointDisplacementView::Impl::Impl(JointDisplacementView* self) :
    self(self)
{
    self->setDefaultLayoutArea(View::CENTER);
    createPanel();
    bodySelectionManager = BodySelectionManager::instance();
}


JointDisplacementView::~JointDisplacementView()
{
    delete impl;
}


JointDisplacementView::Impl::~Impl()
{
    for(size_t i=0; i < jointIndicators.size(); ++i){
        delete jointIndicators[i];
    }
}


void JointDisplacementView::Impl::createPanel()
{
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    auto style = self->style();
    int hspacing = style->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
    int vspacing = style->pixelMetric(QStyle::PM_LayoutVerticalSpacing);
    int lmargin = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int rmargin = style->pixelMetric(QStyle::PM_LayoutRightMargin);
    int tmargin = style->pixelMetric(QStyle::PM_LayoutTopMargin);
    int bmargin = style->pixelMetric(QStyle::PM_LayoutBottomMargin);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    self->setLayout(vbox);

    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(lmargin / 2, tmargin / 2, rmargin / 2, bmargin / 2);
    hbox->addStretch(1);
    targetLabel.setStyleSheet("font-weight: bold");
    targetLabel.setAlignment(Qt::AlignLeft);
    hbox->addWidget(&targetLabel);
    hbox->addStretch(10);
    hbox->addStretch();
    
    menuButton.setText("*");
    menuButton.setToolTip(_("Option"));
    menuButton.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    menuButton.sigClicked().connect([&](){ onMenuButtonClicked(); });
    hbox->addWidget(&menuButton);
    vbox->addLayout(hbox);

    createOptionMenu();
    
    auto gridvbox = new QVBoxLayout;
    gridvbox->setContentsMargins(lmargin / 2, 0, rmargin / 2, 0);
    gridBase.setLayout(gridvbox);
    grid.setHorizontalSpacing(hspacing / 2);
    grid.setVerticalSpacing(vspacing / 2);
    gridvbox->addLayout(&grid);
    gridvbox->addStretch();
    
    scrollArea.setFrameShape(QFrame::NoFrame);
    scrollArea.setWidgetResizable(true);
    scrollArea.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea.setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea.setWidget(&gridBase);

    vbox->addWidget(&scrollArea, 1);

    valueFormatManager = DisplayedValueFormatManager::instance();
    valueFormatManagerConnection =
        valueFormatManager->sigFormatChanged().connect(
            [&](){ updateIndicatorGrid(); });
    
    updateIndicatorGrid();

    updateJointDisplacementsLater.setFunction([&](){ updateJointDisplacements(); });
    updateJointDisplacementsLater.setPriority(LazyCaller::PRIORITY_LOW);
}


void JointDisplacementView::Impl::createOptionMenu()
{
    menuManager.setNewPopupMenu(self);

    selectedJointsOnlyCheck = menuManager.addCheckItem(_("Selected joints only"));
    selectedJointsOnlyCheck->sigToggled().connect(
        [&](bool){ updateIndicatorGrid(); });

    jointIdCheck = menuManager.addCheckItem(_("Joint ID"));
    jointIdCheck->sigToggled().connect(
        [&](bool){ updateIndicatorGrid(); });

    jointNameCheck = menuManager.addCheckItem(_("Joint name"));
    jointNameCheck->setChecked(true);
    jointNameCheck->sigToggled().connect(
        [&](bool){ updateIndicatorGrid(); });

    overlapJointNameCheck = menuManager.addCheckItem(_("Overlap joint name"));
    overlapJointNameCheck->sigToggled().connect(
        [&](bool){ updateIndicatorGrid(); });

    sliderCheck = menuManager.addCheckItem(_("Slider"));
    sliderCheck->setChecked(true);
    sliderCheck->sigToggled().connect(
        [&](bool){ updateIndicatorGrid(); });

    dialCheck = menuManager.addCheckItem(_("Dial"));
    dialCheck->sigToggled().connect(
        [&](bool){ updateIndicatorGrid(); });

    phaseCheck = menuManager.addCheckItem(_("Phase"));
    phaseCheck->sigToggled().connect(
        [&](bool){ updateIndicatorGrid(); });
}


void JointDisplacementView::onActivated()
{
    impl->onActivated();
}


void JointDisplacementView::Impl::onActivated()
{
    bodySelectionManagerConnection =
        bodySelectionManager->sigCurrentBodyItemChanged().connect(
            [&](BodyItem* bodyItem){ onCurrentBodyItemChanged(bodyItem); });

    onCurrentBodyItemChanged(bodySelectionManager->currentBodyItem());
}


void JointDisplacementView::onDeactivated()
{
    impl->bodySelectionManagerConnection.disconnect();
    impl->setBodyItem(nullptr);
}


void JointDisplacementView::Impl::onMenuButtonClicked()
{
    menuManager.popupMenu()->popup(menuButton.mapToGlobal(QPoint(0,0)));
}


void JointDisplacementView::Impl::onCurrentBodyItemChanged(BodyItem* bodyItem)
{
    if(!bodyItem){
        setBodyItem(nullptr);
    } else {
        if(bodyItem->body()->numJoints() > 0){
            setBodyItem(bodyItem);
        } else {
            while(bodyItem = bodyItem->parentBodyItem()){
                if(bodyItem->body()->numJoints() > 0){
                    setBodyItem(bodyItem);
                    break;
                }
            }
        }
    }
}


void JointDisplacementView::Impl::setBodyItem(BodyItem* bodyItem)
{
    if(bodyItem != currentBodyItem){

        linkSelectionChangeConnection.disconnect();
        kinematicStateChangeConnection.disconnect();

        currentBodyItem = bodyItem;
        updateIndicatorGrid();

        if(bodyItem){
            linkSelectionChangeConnection =
                bodySelectionManager->sigLinkSelectionChanged(bodyItem).connect(
                    [&](const std::vector<bool>&){ updateIndicatorGrid(); });
            
            kinematicStateChangeConnection =
                bodyItem->sigKinematicStateChanged().connect(updateJointDisplacementsLater);
            updateJointDisplacements();
        }
    }
}


void JointDisplacementView::Impl::updateIndicatorGrid()
{
    if(!currentBodyItem){
        self->setEnabled(false);
        targetLabel.setText("------");
        initializeIndicators(0);

    } else {
        self->setEnabled(true);
        targetLabel.setText(currentBodyItem->name().c_str());

        BodyPtr body = currentBodyItem->body();
        int numJoints = body->numJoints();
        
        if(selectedJointsOnlyCheck->isChecked()){
            auto& linkSelection = bodySelectionManager->linkSelection(currentBodyItem);
            activeJointIds.clear();
            for(int i=0; i < numJoints; ++i){
                Link* joint = body->joint(i);
                if(joint->isValid() && linkSelection[joint->index()]){
                    activeJointIds.push_back(i);
                }
            }
        } else {
            activeJointIds.resize(numJoints);
            for(int i=0; i < numJoints; ++i){
                activeJointIds[i] = i;
            }
        }

        int n = activeJointIds.size();
        initializeIndicators(n);

        lengthUnit = valueFormatManager->lengthUnit();
        if(lengthUnit == DisplayedValueFormatManager::Millimeter){
            defaultMaxLength = 10000.0;
        } else {
            defaultMaxLength = 10.0;
        }
        lengthDecimals = valueFormatManager->lengthDecimals();
        defaultMaxLength -= pow(10.0, -lengthDecimals);
        lengthStep = valueFormatManager->lengthStep();
        
        angleUnit = valueFormatManager->angleUnit();
        if(angleUnit == DisplayedValueFormatManager::Degree){
            defaultMaxAngle = 1000.0;
        } else {
            defaultMaxAngle = 10.0;
        }
        angleDecimals = valueFormatManager->angleDecimals();
        defaultMaxAngle -= pow(10.0, -angleDecimals);
        angleStep = valueFormatManager->angleStep();

        int row = 0;
        for(int i=0; i < n; ++i){
            auto indicator = jointIndicators[i];
            indicator->initialize(body->joint(activeJointIds[i]));
            row = indicator->attachTo(grid, row, 0, overlapJointNameCheck->isChecked());
        }
        for(int i=0; i < n - 1; ++i){
            jointIndicators[i]->setNextTabOrderIndicator(jointIndicators[i+1]);
        }
    }
}


void JointDisplacementView::Impl::initializeIndicators(int num)
{
    int prevNum = jointIndicators.size();

    for(int i=0; i < prevNum; ++i){
        jointIndicators[i]->removeWidgesFrom(grid);
    }

    if(num > prevNum){
        for(int i=prevNum; i < num; ++i){
            int index = jointIndicators.size();
            jointIndicators.push_back(new JointIndicator(this, index));
        }
    } else if(num < prevNum){
        for(int i=num; i < prevNum; ++i){
            delete jointIndicators[i];
        }
        jointIndicators.resize(num);
    }
}


void JointDisplacementView::Impl::onNumColumnsChanged(int n)
{
    callLater([&](){ updateIndicatorGrid(); });
}


void JointDisplacementView::Impl::onUnitChanged()
{
    BodyPtr body = currentBodyItem->body();
    for(size_t i=0; i < activeJointIds.size(); ++i){
        int jointId = activeJointIds[i];
        jointIndicators[jointId]->initialize(body->joint(jointId));
    }
}


bool JointDisplacementView::Impl::eventFilter(QObject* object, QEvent* event)
{
    Slider* slider = dynamic_cast<Slider*>(object);
    if(slider && (event->type() == QEvent::KeyPress)){
        return onSliderKeyPressEvent(slider, static_cast<QKeyEvent*>(event));
    }

    Dial* dial = dynamic_cast<Dial*>(object);
    if(dial && (event->type() == QEvent::KeyPress)){
        return onDialKeyPressEvent(dial, static_cast<QKeyEvent*>(event));
    }

    return QObject::eventFilter(object, event);
}

bool JointDisplacementView::Impl::onSliderKeyPressEvent(Slider* slider, QKeyEvent* event)
{
    int index = slider->property("JointSliderIndex").toInt();
    
    bool doContinue = false;

    switch(event->key()){

    case Qt::Key_Up:
        focusSlider(index - 1);
        break;
        
    case Qt::Key_Down:
        focusSlider(index + 1);
        break;

    default:
        doContinue = true;
        break;
    }

    return !doContinue;
}


bool JointDisplacementView::Impl::onDialKeyPressEvent(Dial* dial, QKeyEvent* event)
{
    int index = dial->property("JointDialIndex").toInt();

    bool doContinue = false;

    switch(event->key()){

    case Qt::Key_Up:
        focusDial(index - 1);
        break;

    case Qt::Key_Down:
        focusDial(index + 1);
        break;

    default:
        doContinue = true;
        break;
    }

    return !doContinue;
}


void JointDisplacementView::Impl::focusSlider(int index)
{
    if(index >= 0 && index < static_cast<int>(jointIndicators.size())){
        Slider& slider = jointIndicators[index]->slider;
        slider.setFocus(Qt::OtherFocusReason);
        scrollArea.ensureWidgetVisible(&slider);
    }
}


void JointDisplacementView::Impl::focusDial(int index)
{
    if(index >= 0 && index < static_cast<int>(jointIndicators.size())){
        Dial& dial = jointIndicators[index]->dial;
        dial.setFocus(Qt::OtherFocusReason);
        scrollArea.ensureWidgetVisible(&dial);
    }
}

        
void JointDisplacementView::Impl::notifyJointDisplacementInput()
{
    kinematicStateChangeConnection.block();
    currentBodyItem->notifyKinematicStateChange(true);
    kinematicStateChangeConnection.unblock();
}


void JointDisplacementView::Impl::updateJointDisplacements()
{
    for(size_t i=0; i < activeJointIds.size(); ++i){
        jointIndicators[i]->updateDisplacement(false);
    }
}


bool JointDisplacementView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool JointDisplacementView::Impl::storeState(Archive& archive)
{
    archive.write("showSelectedJoints", selectedJointsOnlyCheck->isChecked());
    archive.write("showJointIDs", jointIdCheck->isChecked());
    archive.write("showJointNames", jointNameCheck->isChecked());
    archive.write("overlapJointNames", overlapJointNameCheck->isChecked());
    archive.write("showSliders", sliderCheck->isChecked());
    archive.write("showDials", dialCheck->isChecked());
    archive.write("showPhases", phaseCheck->isChecked());
    archive.writeItemId("currentBodyItem", currentBodyItem);
    
    return true;
}


bool JointDisplacementView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool JointDisplacementView::Impl::restoreState(const Archive& archive)
{
    bool on;
    if(archive.read("showSelectedJoints", on)){
        selectedJointsOnlyCheck->setChecked(on);
    } else if(archive.read("showAllJoints", on)){
        selectedJointsOnlyCheck->setChecked(!on);
    }
    if(archive.read("showJointIds", on) ||
       archive.read("jointId", on)){
        jointIdCheck->setChecked(on);
    }
    if(archive.read("showJointNames", on) ||
       archive.read("name", on)){
        jointNameCheck->setChecked(on);
    }
    if(archive.read("overlapJointNames", on)){
        overlapJointNameCheck->setChecked(on);
    } else if(archive.read("labelOnLeft", on)){
        overlapJointNameCheck->setChecked(!on);
    }
    if(archive.read("showSliders", on) ||
       archive.read("slider", on)){
        sliderCheck->setChecked(on);
    }
    if(archive.read("showDials", on) ||
       archive.read("dials", on)){
        dialCheck->setChecked(on);
    }
    if(archive.read("showPhases", on)){
        phaseCheck->setChecked(on);
    }

    archive.addPostProcess([&](){ restoreCurrentBodyItem(archive); });

    return true;
}


void JointDisplacementView::Impl::restoreCurrentBodyItem(const Archive& archive)
{
    setBodyItem(archive.findItem<BodyItem>("currentBodyItem"));
}
