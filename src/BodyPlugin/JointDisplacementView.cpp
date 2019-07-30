/** \file
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
#include <cnoid/Separator>
#include <cnoid/LazyCaller>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/RootItem>
#include <QGridLayout>
#include <QScrollArea>
#include <QKeyEvent>
#include <QStyle>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class SliderUnit;

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
            
    ToolButton menuButton;
    MenuManager menuManager;
    QLabel targetLabel;
    Action* selectedJointsOnlyCheck;
    Action* jointIdCheck;
    Action* jointNameCheck;
    Action* overlapJointNameCheck;
    Action* sliderCheck;
    Action* dialCheck;
            
    vector<int> activeJointIds;
    vector<SliderUnit*> jointSliders;
    BodyItemPtr currentBodyItem;
            
    ScopedConnection kinematicStateChangeConnection;
    ScopedConnection bodyBarConnection;
    
    LazyCaller updateJointPositionsLater;
        
    Connection connectionOfBodyItemDetachedFromRoot;
    Connection connectionOfLinkSelectionChanged;

    QScrollArea scrollArea;
    QWidget sliderGridBase;
    QGridLayout sliderGrid;

    JointDisplacementViewImpl(JointDisplacementView* self);
    ~JointDisplacementViewImpl();
    void createPanel();
    void createOptionMenu();
    void onMenuButtonClicked();
    void onSelectedBodyItemsChanged(const ItemList<BodyItem>& selected);
    void setBodyItem(BodyItem* bodyItem);
    void updateSliderGrid();
    void attachSliderUnits(SliderUnit* unit, int row, int col);
    void initializeSliders(int num);
    void onNumColumnsChanged(int n);
    void onUnitChanged();
    bool eventFilter(QObject* object, QEvent* event);
    bool onSliderKeyPressEvent(Slider* slider, QKeyEvent* event);
    void focusSlider(int index);
    bool onDialKeyPressEvent(Dial* dial, QKeyEvent* event);
    void focusDial(int index);
    void onJointSliderChanged(int sliderIndex);
    void updateJointPositions();
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
    void restoreCurrentBodyItem(const Archive& archive);
};

}

namespace {
    
class SliderUnit
{
public:
    JointDisplacementViewImpl* viewImpl;
    int index;
    QLabel idLabel;
    QLabel nameLabel;
    DoubleSpinBox spin;
    QLabel lowerLimitLabel;
    Slider slider;
    QLabel upperLimitLabel;
    Dial dial;
    SpinBox phaseSpin;
    double unitConversionRatio;

    SliderUnit(JointDisplacementViewImpl* viewImpl, int index)
        : viewImpl(viewImpl),
          index(index),
          idLabel(&viewImpl->sliderGridBase),
          nameLabel(&viewImpl->sliderGridBase),
          spin(&viewImpl->sliderGridBase),
          lowerLimitLabel(&viewImpl->sliderGridBase),
          slider(Qt::Horizontal, &viewImpl->sliderGridBase),
          upperLimitLabel(&viewImpl->sliderGridBase),
          dial(&viewImpl->sliderGridBase)
    {
        idLabel.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        nameLabel.setAlignment(Qt::AlignCenter);
        nameLabel.setTextInteractionFlags(Qt::TextSelectableByMouse);
        lowerLimitLabel.setAlignment(Qt::AlignCenter);
        upperLimitLabel.setAlignment(Qt::AlignCenter);
            
        spin.setAlignment(Qt::AlignCenter);
        spin.sigValueChanged().connect([&](double v){ onSpinValueChanged(v); });
            
        slider.setSingleStep(0.1 * resolution);
        slider.setProperty("JointSliderIndex", index);
        slider.installEventFilter(viewImpl);
        slider.sigValueChanged().connect([&](double v){ onSliderValueChanged(v); });

        dial.setSingleStep(0.1 * resolution);
        dial.setProperty("JointDialIndex", index);
        dial.installEventFilter(viewImpl);
        dial.sigValueChanged().connect([&](double v){ onDialValueChanged(v); });
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
        spin.show();

        bool on = viewImpl->sliderCheck->isChecked();
        lowerLimitLabel.setVisible(on);
        slider.setVisible(on);
        upperLimitLabel.setVisible(on);

        dial.setVisible(viewImpl->dialCheck->isChecked());

        nameLabel.setText(joint->name().c_str());
        nameLabel.setVisible(viewImpl->jointNameCheck->isChecked());

        idLabel.setText(QString("%1:").arg(joint->jointId()));
        idLabel.setVisible(viewImpl->jointIdCheck->isChecked());

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

        slider.blockSignals(true);
        spin.blockSignals(true);
        dial.blockSignals(true);

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
        if( v<spin.minimum() || v>spin.maximum()){
            int v0 = v<0.0? (int)-v : (int)v;
            double max = pow(10, (int)log10(v0)+1)-deci;
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

        spin.blockSignals(false);
        slider.blockSignals(false);
        dial.blockSignals(false);

        updatePosition(joint);
    }
        
    double value() const
    {
        return spin.value() / unitConversionRatio;
    }

    void updatePosition(Link* joint)
    {
        double v = unitConversionRatio * joint->q();
        if(v != spin.value()){
            slider.blockSignals(true);
            spin.blockSignals(true);
            dial.blockSignals(true);
            spin.setValue(v);
            slider.setValue(v * resolution);
            dial.setValue(v * resolution);
            spin.blockSignals(false);
            slider.blockSignals(false);
            dial.blockSignals(false);
        }
    }

    void onSliderValueChanged(double value)
    {
        spin.blockSignals(true);
        spin.setValue(value / resolution);
        spin.blockSignals(false);
        dial.blockSignals(true);
        dial.setValue(value);
        dial.blockSignals(false);
        viewImpl->onJointSliderChanged(index);
    }

    void onDialValueChanged(double value)
    {
        spin.blockSignals(true);
        double v = value / resolution;
        if(v > spin.maximum() || v < spin.minimum() ) {
            spin.setRange(spin.minimum()*10, spin.maximum()*10);
        }
        spin.setValue(v);
        spin.blockSignals(false);
        slider.blockSignals(true);
        slider.setValue(value);
        slider.blockSignals(false);
        viewImpl->onJointSliderChanged(index);
    }

    void onSpinValueChanged(double value)
    {
        slider.blockSignals(true);
        slider.setValue(value * resolution);
        slider.blockSignals(false);
        dial.blockSignals(true);
        dial.setValue(value* resolution);
        dial.blockSignals(false);
        viewImpl->onJointSliderChanged(index);
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
    for(size_t i=0; i < jointSliders.size(); ++i){
        delete jointSliders[i];
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
    
    auto sliderVBox = new QVBoxLayout;
    sliderVBox->setContentsMargins(lmargin / 2, 0, rmargin / 2, 0);
    sliderGridBase.setLayout(sliderVBox);
    sliderGrid.setHorizontalSpacing(hspacing / 2);
    sliderGrid.setVerticalSpacing(vspacing / 2);
    sliderVBox->addLayout(&sliderGrid);
    sliderVBox->addStretch();
    
    scrollArea.setFrameShape(QFrame::NoFrame);
    scrollArea.setWidgetResizable(true);
    scrollArea.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea.setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea.setWidget(&sliderGridBase);

    vbox->addWidget(&scrollArea, 1);

    updateSliderGrid();

    updateJointPositionsLater.setFunction([&](){ updateJointPositions(); });
    updateJointPositionsLater.setPriority(LazyCaller::PRIORITY_LOW);
}


void JointDisplacementViewImpl::createOptionMenu()
{
    menuManager.setNewPopupMenu(self);

    selectedJointsOnlyCheck = menuManager.addCheckItem(_("Selected joints only"));
    selectedJointsOnlyCheck->sigToggled().connect(
        [&](bool){ updateSliderGrid(); });

    jointIdCheck = menuManager.addCheckItem(_("Joint IDs"));
    jointIdCheck->sigToggled().connect(
        [&](bool){ updateSliderGrid(); });

    jointNameCheck = menuManager.addCheckItem(_("Joint names"));
    jointNameCheck->setChecked(true);
    jointNameCheck->sigToggled().connect(
        [&](bool){ updateSliderGrid(); });

    overlapJointNameCheck = menuManager.addCheckItem(_("Overlap joint names"));
    overlapJointNameCheck->sigToggled().connect(
        [&](bool){ updateSliderGrid(); });

    sliderCheck = menuManager.addCheckItem(_("Sliders"));
    sliderCheck->setChecked(true);
    sliderCheck->sigToggled().connect(
        [&](bool){ updateSliderGrid(); });

    dialCheck = menuManager.addCheckItem(_("Dials"));
    dialCheck->sigToggled().connect(
        [&](bool){ updateSliderGrid(); });
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

        connectionOfLinkSelectionChanged.disconnect();
        kinematicStateChangeConnection.disconnect();

        currentBodyItem = bodyItem;
        updateSliderGrid();

        if(bodyItem){
            connectionOfLinkSelectionChanged =
                LinkSelectionView::mainInstance()->sigSelectionChanged(bodyItem).connect(
                    [&](){ updateSliderGrid(); });
            
            kinematicStateChangeConnection =
                bodyItem->sigKinematicStateChanged().connect(updateJointPositionsLater);
            updateJointPositions();
        }
    }
}


void JointDisplacementViewImpl::updateSliderGrid()
{
    if(!currentBodyItem){
        self->setEnabled(false);
        targetLabel.setText("------");
        initializeSliders(0);

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
        
        initializeSliders(n);

        int row = 0;
        for(int i=0; i < n; ++i){
            SliderUnit* unit = jointSliders[i];
            unit->initialize(body->joint(activeJointIds[i]));
            if(overlapJointNameCheck->isChecked()){
                sliderGrid.addWidget(&unit->nameLabel, row, 0, 1, 6);
                sliderGrid.addWidget(&unit->idLabel, row + 1, 0);
                attachSliderUnits(unit, row + 1, 1);
                row += 2;
            } else {
                sliderGrid.addWidget(&unit->idLabel,row, 0);
                sliderGrid.addWidget(&unit->nameLabel,row, 1);
                attachSliderUnits(unit, row, 2);
                row += 1;
            }
        }
    }
}


void JointDisplacementViewImpl::attachSliderUnits(SliderUnit* unit, int row, int col)
{
    sliderGrid.addWidget(&unit->spin, row, col);
    sliderGrid.addWidget(&unit->lowerLimitLabel,row, col + 1);
    sliderGrid.addWidget(&unit->slider, row, col + 2);
    sliderGrid.addWidget(&unit->upperLimitLabel,row, col + 3);
    sliderGrid.addWidget(&unit->dial, row, col + 4);
}


void JointDisplacementViewImpl::initializeSliders(int num)
{
    int prevNum = jointSliders.size();

    for(int i=0; i < prevNum; ++i){
        jointSliders[i]->removeWidgesFrom(sliderGrid);
    }

    if(num > prevNum){
        for(int i=prevNum; i < num; ++i){
            int index = jointSliders.size();
            jointSliders.push_back(new SliderUnit(this, index));
        }
    } else if(num < prevNum){
        for(int i=num; i < prevNum; ++i){
            delete jointSliders[i];
        }
        jointSliders.resize(num);
    }
}


void JointDisplacementViewImpl::onNumColumnsChanged(int n)
{
    callLater([&](){ updateSliderGrid(); });
}


void JointDisplacementViewImpl::onUnitChanged()
{
    BodyPtr body = currentBodyItem->body();
    for(size_t i=0; i < activeJointIds.size(); ++i){
        int jointId = activeJointIds[i];
        jointSliders[jointId]->initialize(body->joint(jointId));
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
    if(index >= 0 && index < static_cast<int>(jointSliders.size())){
        Slider& slider = jointSliders[index]->slider;
        slider.setFocus(Qt::OtherFocusReason);
        scrollArea.ensureWidgetVisible(&slider);
    }
}


void JointDisplacementViewImpl::focusDial(int index)
{
    if(index >= 0 && index < static_cast<int>(jointSliders.size())){
        Dial& dial = jointSliders[index]->dial;
        dial.setFocus(Qt::OtherFocusReason);
        scrollArea.ensureWidgetVisible(&dial);
    }
}

        
void JointDisplacementViewImpl::onJointSliderChanged(int sliderIndex)
{
    int jointId = activeJointIds[sliderIndex];
    Link* joint = currentBodyItem->body()->joint(jointId);
    joint->q() = jointSliders[sliderIndex]->value();

    kinematicStateChangeConnection.block();
    currentBodyItem->notifyKinematicStateChange(true);
    kinematicStateChangeConnection.unblock();
}


void JointDisplacementViewImpl::updateJointPositions()
{
    BodyPtr body = currentBodyItem->body();
    for(size_t i=0; i < activeJointIds.size(); ++i){
        int jointId = activeJointIds[i];
        jointSliders[i]->updatePosition(body->joint(jointId));
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

    archive.addPostProcess([&](){ restoreCurrentBodyItem(archive); });

    return true;
}


void JointDisplacementViewImpl::restoreCurrentBodyItem(const Archive& archive)
{
    setBodyItem(archive.findItem<BodyItem>("currentBodyItem"));
}
