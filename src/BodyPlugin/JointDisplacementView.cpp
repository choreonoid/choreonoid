/**
   \author Shin'ichiro Nakaoka
*/

#include "JointDisplacementView.h"
#include "BodyItem.h"
#include "BodyBar.h"
#include "LinkSelectionView.h"
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
#include <QGridLayout>
#include <QScrollArea>
#include <QKeyEvent>
#include <QStyle>
#include "gettext.h"

#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

class JointIndicator;

/**
   Slider resolution.
   This value should not be increased more because QDial won't work correctly in that case.
*/
const double resolution = 100000.0;

}

namespace cnoid {

class JointDisplacementViewImpl : public QObject
{
public:
    JointDisplacementView* self;
            
    vector<int> activeJointIds;
    vector<JointIndicator*> jointIndicators;
    BodyItemPtr currentBodyItem;
            
    ScopedConnection kinematicStateChangeConnection;
    ScopedConnection bodyBarConnection;
    ScopedConnection linkSelectionChangeConnection;
    LazyCaller updateJointDisplacementsLater;

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

    JointDisplacementViewImpl(JointDisplacementView* self);
    ~JointDisplacementViewImpl();
    void createPanel();
    void createOptionMenu();
    void onMenuButtonClicked();
    void onSelectedBodyItemsChanged(const ItemList<BodyItem>& selected);
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
    JointDisplacementViewImpl* viewImpl;
    int index;
    Link* joint;
    double unitConversionRatio;
    DoubleSpinBox spin;
    Slider slider;
    Dial dial;
    SpinBox phaseSpin;
    int minPhase;
    int maxPhase;
    QLabel idLabel;
    QLabel nameLabel;
    QLabel lowerLimitLabel;
    QLabel upperLimitLabel;

    JointIndicator(JointDisplacementViewImpl* viewImpl, int index)
        : viewImpl(viewImpl),
          index(index),
          joint(nullptr),
          spin(&viewImpl->gridBase),
          slider(Qt::Horizontal, &viewImpl->gridBase),
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
            grid.addWidget(&idLabel, row, col);
        } else {
            grid.addWidget(&idLabel, row, col);
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

    bool isDegreeMode() const { return true; }

    void setRangeLabelValues(double lower, double upper, int precision)
    {
        if(fabs(lower) > 10000.0){
            lowerLimitLabel.setText(QString::number(lower, 'g', precision));
        } else {
            lowerLimitLabel.setText(QString::number(lower, 'f', precision));
        }
        if(fabs(upper) > 10000.0){
            upperLimitLabel.setText(QString::number(upper, 'g', precision));
        } else {
            upperLimitLabel.setText(QString::number(upper, 'f', precision));
        }
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

        unitConversionRatio = 1.0;
        double max;
        if(joint->isRotationalJoint()){
            if(isDegreeMode()){
                unitConversionRatio = 180.0 / PI;
            }
            max = 2.0 * PI;
        } else { // SLIDE_JOINT
            max = std::numeric_limits<double>::max();
        }
        double lower = unitConversionRatio * (joint->q_lower() < -max ? -max : joint->q_lower());
        double upper = unitConversionRatio * (joint->q_upper() > max ? max : joint->q_upper());

        dial.setVisible(viewImpl->dialCheck->isChecked());

        slider.blockSignals(true);
        spin.blockSignals(true);
        dial.blockSignals(true);
        phaseSpin.blockSignals(true);

        slider.setRange(lower * resolution, upper * resolution);

        double deci;
        if(unitConversionRatio != 1.0){ // degree mode
            spin.setDecimals(1);
            spin.setRange(-999.9, 999.9);
            spin.setSingleStep(0.1);
            setRangeLabelValues(lower, upper, 1);
            deci = 0.1;
        } else { // radian or meter
            spin.setDecimals(4);
            spin.setRange(-9.99, 9.99);
            spin.setSingleStep(0.0001);
            setRangeLabelValues(lower, upper, 3);
            deci = 0.0001;
        }
        double v = unitConversionRatio * joint->q();
        if(v < spin.minimum() || v > spin.maximum()){
            int v0 = v < 0.0? (int)-v : (int)v;
            double max = pow(10, (int)log10(v0) + 1)-deci;
            spin.setRange(-max, max);
        }

        if(joint->q_lower() == -std::numeric_limits<double>::max() &&
           joint->q_upper() == std::numeric_limits<double>::max()){
            dial.setWrapping(true);
            dial.setNotchesVisible(false);
            dial.setRange(-PI * unitConversionRatio * resolution, PI * unitConversionRatio * resolution);
        }else {
            dial.setWrapping(false);
            dial.setNotchesVisible(true);
            dial.setRange(lower * resolution, upper * resolution);
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
        double v = unitConversionRatio * joint->q();

        if(forceUpdate || v != spin.value()){

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
    impl = new JointDisplacementViewImpl(this);
}


JointDisplacementViewImpl::JointDisplacementViewImpl(JointDisplacementView* self) :
    self(self)
{
    self->setDefaultLayoutArea(View::CENTER);
    createPanel();
}


JointDisplacementView::~JointDisplacementView()
{
    delete impl;
}


JointDisplacementViewImpl::~JointDisplacementViewImpl()
{
    for(size_t i=0; i < jointIndicators.size(); ++i){
        delete jointIndicators[i];
    }
}


void JointDisplacementViewImpl::createPanel()
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
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->setSpacing(0);
    self->setLayout(vbox);

    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(lmargin / 2, rmargin / 2, tmargin / 2, bmargin / 2);
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

    updateIndicatorGrid();

    updateJointDisplacementsLater.setFunction([&](){ updateJointDisplacements(); });
    updateJointDisplacementsLater.setPriority(LazyCaller::PRIORITY_LOW);
}


void JointDisplacementViewImpl::createOptionMenu()
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
    auto bb = BodyBar::instance();
    impl->bodyBarConnection =
        bb->sigBodyItemSelectionChanged().connect(
            [&](const ItemList<BodyItem>& selected){
                impl->onSelectedBodyItemsChanged(selected);
            });

    impl->onSelectedBodyItemsChanged(bb->selectedBodyItems());
    if(!impl->currentBodyItem){
        ItemList<BodyItem> items;
        items.extractSubTreeItems(RootItem::instance());
        impl->onSelectedBodyItemsChanged(items);
    }
}


void JointDisplacementView::onDeactivated()
{
    impl->bodyBarConnection.disconnect();
    impl->setBodyItem(nullptr);
}


void JointDisplacementViewImpl::onMenuButtonClicked()
{
    menuManager.popupMenu()->popup(menuButton.mapToGlobal(QPoint(0,0)));
}


void JointDisplacementViewImpl::onSelectedBodyItemsChanged(const ItemList<BodyItem>& selected)
{
    for(auto& item : selected){
        if(item->body()->numJoints() > 0){
            setBodyItem(item);
            return;
        }
    }
}


void JointDisplacementViewImpl::setBodyItem(BodyItem* bodyItem)
{
    if(bodyItem != currentBodyItem){

        linkSelectionChangeConnection.disconnect();
        kinematicStateChangeConnection.disconnect();

        currentBodyItem = bodyItem;
        updateIndicatorGrid();

        if(bodyItem){
            linkSelectionChangeConnection =
                LinkSelectionView::mainInstance()->sigSelectionChanged(bodyItem).connect(
                    [&](){ updateIndicatorGrid(); });
            
            kinematicStateChangeConnection =
                bodyItem->sigKinematicStateChanged().connect(updateJointDisplacementsLater);
            updateJointDisplacements();
        }
    }
}


void JointDisplacementViewImpl::updateIndicatorGrid()
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
            const auto& linkSelection =
                LinkSelectionView::instance()->linkSelection(currentBodyItem);
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


void JointDisplacementViewImpl::initializeIndicators(int num)
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


void JointDisplacementViewImpl::onNumColumnsChanged(int n)
{
    callLater([&](){ updateIndicatorGrid(); });
}


void JointDisplacementViewImpl::onUnitChanged()
{
    BodyPtr body = currentBodyItem->body();
    for(size_t i=0; i < activeJointIds.size(); ++i){
        int jointId = activeJointIds[i];
        jointIndicators[jointId]->initialize(body->joint(jointId));
    }
}


bool JointDisplacementViewImpl::eventFilter(QObject* object, QEvent* event)
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

bool JointDisplacementViewImpl::onSliderKeyPressEvent(Slider* slider, QKeyEvent* event)
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


bool JointDisplacementViewImpl::onDialKeyPressEvent(Dial* dial, QKeyEvent* event)
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


void JointDisplacementViewImpl::focusSlider(int index)
{
    if(index >= 0 && index < static_cast<int>(jointIndicators.size())){
        Slider& slider = jointIndicators[index]->slider;
        slider.setFocus(Qt::OtherFocusReason);
        scrollArea.ensureWidgetVisible(&slider);
    }
}


void JointDisplacementViewImpl::focusDial(int index)
{
    if(index >= 0 && index < static_cast<int>(jointIndicators.size())){
        Dial& dial = jointIndicators[index]->dial;
        dial.setFocus(Qt::OtherFocusReason);
        scrollArea.ensureWidgetVisible(&dial);
    }
}

        
void JointDisplacementViewImpl::notifyJointDisplacementInput()
{
    kinematicStateChangeConnection.block();
    currentBodyItem->notifyKinematicStateChange(true);
    kinematicStateChangeConnection.unblock();
}


void JointDisplacementViewImpl::updateJointDisplacements()
{
    for(size_t i=0; i < activeJointIds.size(); ++i){
        jointIndicators[i]->updateDisplacement(false);
    }
}


bool JointDisplacementView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool JointDisplacementViewImpl::storeState(Archive& archive)
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


bool JointDisplacementViewImpl::restoreState(const Archive& archive)
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


void JointDisplacementViewImpl::restoreCurrentBodyItem(const Archive& archive)
{
    setBodyItem(archive.findItem<BodyItem>("currentBodyItem"));
}
