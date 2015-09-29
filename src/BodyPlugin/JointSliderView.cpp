/** \file
    \author Shin'ichiro Nakaoka
*/

#include "JointSliderView.h"
#include "BodyItem.h"
#include "BodyBar.h"
#include "LinkSelectionView.h"
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/Button>
#include <cnoid/SpinBox>
#include <cnoid/Slider>
#include <cnoid/Separator>
#include <cnoid/LazyCaller>
#include <cnoid/ViewManager>
#include <QGridLayout>
#include <QScrollArea>
#include <QKeyEvent>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
class SliderUnit;

// slider resolution
const double r = 1000000.0;
}

namespace cnoid {

class JointSliderViewImpl : public QObject
{
public:
    JointSliderViewImpl(JointSliderView* self);
    ~JointSliderViewImpl();
            
    JointSliderView* self;
            
    vector<int> activeJointIds;
    vector<SliderUnit*> jointSliders;
    BodyItemPtr currentBodyItem;
            
    Connection connectionOfKinematicStateChanged;
    Connection connectionOfCurrentBodyItemChanged;
    
    LazyCaller updateJointPositionsLater;
        
    Connection connectionOfBodyItemDetachedFromRoot;
    Connection connectionOfLinkSelectionChanged;

    ToggleToolButton showAllToggle;
    ToggleToolButton jointIdToggle;
    ToggleToolButton nameToggle;            
    ToggleToolButton labelOnLeftToggle;
    ToggleToolButton putSpinEntryCheck;
    ToggleToolButton putSliderCheck;
    SpinBox numColumnsSpin;

    QButtonGroup unitRadioGroup;
    ToggleToolButton degreeRadio;
    ToggleToolButton radianRadio;
            
    QScrollArea scrollArea;
    QWidget sliderGridBase;
    QGridLayout sliderGrid;

    void updateSliderGrid();
    void attachSliderUnits(SliderUnit* unit, int row, int col);
    void initializeSliders(int num);
    void onNumColumnsChanged(int n);
    void onUnitChanged();
    bool eventFilter(QObject* object, QEvent* event);
    bool onSliderKeyPressEvent(Slider* slider, QKeyEvent* event);
    void focusSlider(int index);
    void onJointSliderChanged(int sliderIndex);
    void updateJointPositions();
    void onCurrentBodyItemChanged(BodyItem* bodyItem);
    void enableConnectionToSigKinematicStateChanged(bool on);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
    void restoreCurrentBodyItem(const Archive& archive);
};

}

namespace {
    
class SliderUnit
{
public:
    JointSliderViewImpl* viewImpl;
    int index;
    QLabel idLabel;
    QLabel nameLabel;
    DoubleSpinBox spin;
    QLabel lowerLimitLabel;
    Slider slider;
    QLabel upperLimitLabel;
    double unitConversionRatio;

    SliderUnit(JointSliderViewImpl* viewImpl, int index)
        : viewImpl(viewImpl),
          index(index),
          idLabel(&viewImpl->sliderGridBase),
          nameLabel(&viewImpl->sliderGridBase),
          spin(&viewImpl->sliderGridBase),
          lowerLimitLabel(&viewImpl->sliderGridBase),
          slider(Qt::Horizontal, &viewImpl->sliderGridBase),
          upperLimitLabel(&viewImpl->sliderGridBase) {

        idLabel.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        nameLabel.setAlignment(Qt::AlignCenter);
        nameLabel.setTextInteractionFlags(Qt::TextSelectableByMouse);
        lowerLimitLabel.setAlignment(Qt::AlignCenter);
        upperLimitLabel.setAlignment(Qt::AlignCenter);
            
        spin.setAlignment(Qt::AlignCenter);
        spin.sigValueChanged().connect(boost::bind(&SliderUnit::onSpinValueChanged, this, _1));
            
        slider.setSingleStep(0.1 * r);
        slider.setProperty("JointSliderIndex", index);
        slider.installEventFilter(viewImpl);
        slider.sigValueChanged().connect(boost::bind(&SliderUnit::onSliderValueChanged, this, _1));
    }

    void initialize(Link* joint){

        if(viewImpl->putSpinEntryCheck.isChecked()){
            spin.show();
        } else {
            spin.hide();
        }
        if(viewImpl->putSliderCheck.isChecked()){
            lowerLimitLabel.show();
            slider.show();
            upperLimitLabel.show();
        } else {
            lowerLimitLabel.hide();
            slider.hide();
            upperLimitLabel.hide();
        }
        nameLabel.setText(joint->name().c_str());
        if(viewImpl->nameToggle.isChecked()){
            nameLabel.show();
        } else {
            nameLabel.hide();
        }
        idLabel.setText(QString("%1:").arg(joint->jointId()));
        if(viewImpl->jointIdToggle.isChecked()){
            idLabel.show();
        } else {
            idLabel.hide();
        }

        unitConversionRatio = 1.0;
        double max;
        if(joint->isRotationalJoint()){
            if(viewImpl->degreeRadio.isChecked()){
                unitConversionRatio = 180.0 / PI;
            }
            max = unitConversionRatio * 2.0 * PI;
        } else { // SLIDE_JOINT
            max = std::numeric_limits<double>::max();
        }
        double lower = std::max(unitConversionRatio * joint->q_lower(), -max);
        double upper = std::min(unitConversionRatio * joint->q_upper(),  max);

        slider.blockSignals(true);
        spin.blockSignals(true);

        slider.setRange(lower * r, upper * r);

        if(unitConversionRatio != 1.0){ // degree mode
            spin.setDecimals(1);
            spin.setRange(-999.9, 999.9);
            spin.setSingleStep(0.1);
            lowerLimitLabel.setText(QString::number(lower, 'f', 1));
            upperLimitLabel.setText(QString::number(upper, 'f', 1));
        } else { // radian or meter
            spin.setDecimals(4);
            spin.setRange(-9.99, 9.99);
            spin.setSingleStep(0.0001);
            lowerLimitLabel.setText(QString::number(lower, 'f', 3));
            upperLimitLabel.setText(QString::number(upper, 'f', 3));
        }

        spin.blockSignals(false);
        slider.blockSignals(false);

        updatePosition(joint);
    }
        
    double value() const {
        return spin.value() / unitConversionRatio;
    }

    void updatePosition(Link* joint) {
        double v = unitConversionRatio * joint->q();
        if(v != spin.value()){
            slider.blockSignals(true);
            spin.blockSignals(true);
            spin.setValue(v);
            slider.setValue(v * r);
            spin.blockSignals(false);
            slider.blockSignals(false);
        }
    }

    void onSliderValueChanged(double value){
        spin.blockSignals(true);
        spin.setValue(value / r);
        spin.blockSignals(false);
        viewImpl->onJointSliderChanged(index);
    }

    void onSpinValueChanged(double value){
        slider.blockSignals(true);
        slider.setValue(value * r);
        slider.blockSignals(false);
        viewImpl->onJointSliderChanged(index);
    }

    void removeWidgesFrom(QGridLayout& grid){
        grid.removeWidget(&idLabel);
        grid.removeWidget(&nameLabel);
        grid.removeWidget(&spin);
        grid.removeWidget(&lowerLimitLabel);
        grid.removeWidget(&slider);
        grid.removeWidget(&upperLimitLabel);
    }
};
}


void JointSliderView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<JointSliderView>(
        "JointSliderView", N_("Joint Sliders"), ViewManager::SINGLE_DEFAULT);
}


JointSliderView::JointSliderView()
{
    impl = new JointSliderViewImpl(this);
}


JointSliderViewImpl::JointSliderViewImpl(JointSliderView* self) :
    self(self)
{
    using boost::bind;
    
    self->setDefaultLayoutArea(View::CENTER);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->setSpacing(0);

    showAllToggle.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    showAllToggle.setText(_("All"));
    showAllToggle.setToolTip(_("Show all the joints including unselected ones"));
    showAllToggle.setChecked(true);
    showAllToggle.sigToggled().connect(bind(&JointSliderViewImpl::updateSliderGrid, this));
    hbox->addWidget(&showAllToggle);

    jointIdToggle.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    jointIdToggle.setText(_("ID"));
    jointIdToggle.setToolTip(_("Show joint IDs"));
    jointIdToggle.setChecked(false);
    jointIdToggle.sigToggled().connect(bind(&JointSliderViewImpl::updateSliderGrid, this));
    hbox->addWidget(&jointIdToggle);

    nameToggle.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    nameToggle.setText(_("Name"));
    nameToggle.setToolTip(_("Show joint names"));
    nameToggle.setChecked(true);
    nameToggle.sigToggled().connect(bind(&JointSliderViewImpl::updateSliderGrid, this));
    hbox->addWidget(&nameToggle);
    
    putSpinEntryCheck.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    putSpinEntryCheck.setText(_("Entry"));
    putSpinEntryCheck.setToolTip(_("Show spin entries for numerical input"));
    putSpinEntryCheck.setChecked(true);
    putSpinEntryCheck.sigToggled().connect(bind(&JointSliderViewImpl::updateSliderGrid, this));
    hbox->addWidget(&putSpinEntryCheck);

    putSliderCheck.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    putSliderCheck.setText(_("Slider"));
    putSliderCheck.setToolTip(_("Show sliders for chaning joint positions"));
    putSliderCheck.setChecked(true);
    putSliderCheck.sigToggled().connect(bind(&JointSliderViewImpl::updateSliderGrid, this));
    hbox->addWidget(&putSliderCheck);
    
    labelOnLeftToggle.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    labelOnLeftToggle.setText(_("IL"));
    labelOnLeftToggle.setToolTip(_("Put all the components for each joint in-line"));
    labelOnLeftToggle.setChecked(true);
    labelOnLeftToggle.sigToggled().connect(bind(&JointSliderViewImpl::updateSliderGrid, this));
    hbox->addWidget(&labelOnLeftToggle);

    hbox->addSpacing(4);
    hbox->addWidget(new VSeparator());
    hbox->addSpacing(4);
    
    numColumnsSpin.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    numColumnsSpin.setToolTip(_("The number of columns"));
    numColumnsSpin.setRange(1, 9);
    numColumnsSpin.setValue(1);
    numColumnsSpin.sigValueChanged().connect(
        bind(&JointSliderViewImpl::onNumColumnsChanged, this, _1));
    hbox->addWidget(&numColumnsSpin);

    hbox->addSpacing(4);
    hbox->addWidget(new VSeparator());
    hbox->addSpacing(4);

    unitRadioGroup.addButton(&degreeRadio);
    degreeRadio.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    degreeRadio.setText(_("Deg."));
    degreeRadio.setChecked(true);
    degreeRadio.sigToggled().connect(bind(&JointSliderViewImpl::onUnitChanged, this));
    hbox->addWidget(&degreeRadio);

    unitRadioGroup.addButton(&radianRadio);
    radianRadio.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    radianRadio.setText(_("Rad."));
    radianRadio.sigToggled().connect(bind(&JointSliderViewImpl::onUnitChanged, this));
    hbox->addWidget(&radianRadio);

    hbox->addStretch();
    vbox->addLayout(hbox);

    sliderGrid.setSpacing(0);
    QVBoxLayout* gridVBox = new QVBoxLayout();
    gridVBox->addLayout(&sliderGrid);
    gridVBox->addStretch();
    sliderGridBase.setLayout(gridVBox);
    scrollArea.setFrameShape(QFrame::NoFrame);
    scrollArea.setWidgetResizable(true);
    scrollArea.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea.setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea.setWidget(&sliderGridBase);

    vbox->addWidget(&scrollArea, 1);
    self->setLayout(vbox);

    updateSliderGrid();

    updateJointPositionsLater.setFunction(bind(&JointSliderViewImpl::updateJointPositions, this));
    updateJointPositionsLater.setPriority(LazyCaller::PRIORITY_LOW);

    connectionOfCurrentBodyItemChanged = 
        BodyBar::instance()->sigCurrentBodyItemChanged().connect(
            bind(&JointSliderViewImpl::onCurrentBodyItemChanged, this, _1));

    self->sigActivated().connect(bind(&JointSliderViewImpl::enableConnectionToSigKinematicStateChanged, this, true));
    self->sigDeactivated().connect(bind(&JointSliderViewImpl::enableConnectionToSigKinematicStateChanged, this, false));
}


JointSliderView::~JointSliderView()
{
    delete impl;
}


JointSliderViewImpl::~JointSliderViewImpl()
{
    for(size_t i=0; i < jointSliders.size(); ++i){
        delete jointSliders[i];
    }
    connectionOfKinematicStateChanged.disconnect();
    connectionOfCurrentBodyItemChanged.disconnect();
}


void JointSliderViewImpl::updateSliderGrid()
{
    if(!currentBodyItem){
        initializeSliders(0);

    } else {

        BodyPtr body = currentBodyItem->body();
        int numJoints = body->numJoints();
        
        if(!showAllToggle.isChecked()){
            const boost::dynamic_bitset<>& linkSelection =
                LinkSelectionView::mainInstance()->getLinkSelection(currentBodyItem);
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

        int nColumns = numColumnsSpin.value();
        bool isLabelAtLeft = labelOnLeftToggle.isChecked();
        int nUnitColumns, nGridColumns;
        if(isLabelAtLeft){
            nUnitColumns = 6;
            nGridColumns = nColumns * nUnitColumns;
        } else {
            nUnitColumns = 5;
            nGridColumns = nColumns * nUnitColumns;
        }

        int row = 0;
        int col = 0;

        for(int i=0; i < n; ++i){

            SliderUnit* unit = jointSliders[i];

            unit->initialize(body->joint(activeJointIds[i]));
            
            if(!isLabelAtLeft){
                sliderGrid.addWidget(&unit->nameLabel, row, col, 1, nUnitColumns);
                sliderGrid.addWidget(&unit->idLabel, row + 1, col);
                attachSliderUnits(unit, row + 1, col + 1);
                col += nUnitColumns;
                if(col == nGridColumns){
                    col = 0;
                    row += 2;
                }
            } else {
                sliderGrid.addWidget(&unit->idLabel,row, col);
                sliderGrid.addWidget(&unit->nameLabel,row, col + 1);
                attachSliderUnits(unit, row, col + 2);
                col += nUnitColumns;
                if(col == nGridColumns){
                    col = 0;
                    row += 1;
                }
            }
        }
    }
}


void JointSliderViewImpl::attachSliderUnits(SliderUnit* unit, int row, int col)
{
    sliderGrid.addWidget(&unit->spin, row, col);
    sliderGrid.addWidget(&unit->lowerLimitLabel,row, col + 1);
    sliderGrid.addWidget(&unit->slider, row, col + 2);
    sliderGrid.addWidget(&unit->upperLimitLabel,row, col + 3);
}


void JointSliderViewImpl::initializeSliders(int num)
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


void JointSliderViewImpl::onNumColumnsChanged(int n)
{
    callLater(boost::bind(&JointSliderViewImpl::updateSliderGrid, this));
}


void JointSliderViewImpl::onUnitChanged()
{
    BodyPtr body = currentBodyItem->body();
    for(size_t i=0; i < activeJointIds.size(); ++i){
        int jointId = activeJointIds[i];
        jointSliders[jointId]->initialize(body->joint(jointId));
    }
}


bool JointSliderViewImpl::eventFilter(QObject* object, QEvent* event)
{
    Slider* slider = dynamic_cast<Slider*>(object);
    if(slider && (event->type() == QEvent::KeyPress)){
        return onSliderKeyPressEvent(slider, static_cast<QKeyEvent*>(event));
    }
    return QObject::eventFilter(object, event);
}

bool JointSliderViewImpl::onSliderKeyPressEvent(Slider* slider, QKeyEvent* event)
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


void JointSliderViewImpl::focusSlider(int index)
{
    if(index >= 0 && index < static_cast<int>(jointSliders.size())){
        Slider& slider = jointSliders[index]->slider;
        slider.setFocus(Qt::OtherFocusReason);
        scrollArea.ensureWidgetVisible(&slider);
    }
}

        
void JointSliderViewImpl::onJointSliderChanged(int sliderIndex)
{
    int jointId = activeJointIds[sliderIndex];
    Link* joint = currentBodyItem->body()->joint(jointId);
    joint->q() = jointSliders[sliderIndex]->value();
    
    connectionOfKinematicStateChanged.block();
    currentBodyItem->notifyKinematicStateChange(true);
    connectionOfKinematicStateChanged.unblock();
}


void JointSliderViewImpl::updateJointPositions()
{
    BodyPtr body = currentBodyItem->body();
    for(size_t i=0; i < activeJointIds.size(); ++i){
        int jointId = activeJointIds[i];
        jointSliders[i]->updatePosition(body->joint(jointId));
    }
}


void JointSliderViewImpl::onCurrentBodyItemChanged(BodyItem* bodyItem)
{
    currentBodyItem = bodyItem;

    connectionOfLinkSelectionChanged.disconnect();

    if(currentBodyItem){
        connectionOfLinkSelectionChanged =
            LinkSelectionView::mainInstance()->sigSelectionChanged(bodyItem).connect
            (boost::bind(&JointSliderViewImpl::updateSliderGrid, this));
    }
    
    updateSliderGrid();
    
    enableConnectionToSigKinematicStateChanged(true);
}


void JointSliderViewImpl::enableConnectionToSigKinematicStateChanged(bool on)
{
    connectionOfKinematicStateChanged.disconnect();

    if(on && self->isActive() && currentBodyItem){
        connectionOfKinematicStateChanged = currentBodyItem->sigKinematicStateChanged().connect(
            //bind(&JointSliderViewImpl::updateJointPositions, this));
            boost::bind(updateJointPositionsLater));
        updateJointPositions();
    }
}


bool JointSliderView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool JointSliderViewImpl::storeState(Archive& archive)
{
    archive.write("showAllJoints", (showAllToggle.isChecked()));
    archive.write("jointId", (jointIdToggle.isChecked()));
    archive.write("name", (nameToggle.isChecked()));
    archive.write("numColumns", (numColumnsSpin.value()));
    archive.write("spinBox", (putSpinEntryCheck.isChecked()));
    archive.write("slider", (putSliderCheck.isChecked()));
    archive.write("labelOnLeft", (labelOnLeftToggle.isChecked()));
    archive.writeItemId("currentBodyItem", currentBodyItem);
    return true;
}


bool JointSliderView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool JointSliderViewImpl::restoreState(const Archive& archive)
{
    showAllToggle.setChecked(archive.get("showAllJoints", true));
    jointIdToggle.setChecked(archive.get("jointId", false));
    nameToggle.setChecked(archive.get("name", true));
    numColumnsSpin.setValue(archive.get("numColumns", 1));
    putSpinEntryCheck.setChecked(archive.get("spinBox", true));
    putSliderCheck.setChecked(archive.get("slider", true));
    labelOnLeftToggle.setChecked(archive.get("labelOnLeft", true));

    archive.addPostProcess(
        boost::bind(&JointSliderViewImpl::restoreCurrentBodyItem, this, boost::ref(archive)));

    return true;
}


void JointSliderViewImpl::restoreCurrentBodyItem(const Archive& archive)
{
    onCurrentBodyItemChanged(archive.findItem<BodyItem>("currentBodyItem"));
}
