#include "DisplayValueFormatBar.h"
#include "ExtensionManager.h"
#include "DisplayValueFormat.h"
#include <cnoid/ComboBox>
#include <cnoid/SpinBox>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class DisplayValueFormatBar::Impl
{
public:
    DisplayValueFormat* displayValueFormat;
    ScopedConnection displayValueFormatConnection;
    ComboBox lengthUnitCombo;
    SpinBox lengthPrecisionSpin;
    ComboBox angleUnitCombo;
    SpinBox anglePrecisionSpin;
    ComboBox coordinateSystemCombo;

    Impl(DisplayValueFormatBar* self);
    void onDisplayValueFormatChanged(bool doBlockSignals);
    void updateLengthUnit(int unitType);
    void updateLengthDecimals(int decimals);
    void updateAngleUnit(int unitType);
    void updateAngleDecimals(int decimals);
    void updateCoordinateSystem(int system);
};

}


void DisplayValueFormatBar::initialize(ExtensionManager* ext)
{
    ext->addToolBar(new DisplayValueFormatBar);
}


DisplayValueFormatBar::DisplayValueFormatBar()
    : ToolBar(N_("DisplayValueFormatBar"))
{
    impl = new Impl(this);
}


DisplayValueFormatBar::Impl::Impl(DisplayValueFormatBar* self)
{
    displayValueFormat = DisplayValueFormat::master();

    displayValueFormatConnection =
        displayValueFormat->sigFormatChanged().connect(
            [this](){ onDisplayValueFormatChanged(true); });
    
    lengthUnitCombo.addItem(_("Meter"), DisplayValueFormat::Meter);
    lengthUnitCombo.addItem(_("Millimeter"), DisplayValueFormat::Millimeter);
    self->addWidget(&lengthUnitCombo);

    auto label = self->addLabel(" . ");
    lengthPrecisionSpin.setRange(0, 10);
    lengthPrecisionSpin.setToolTip(_("Length value precision (number of decimals)"));
    self->addWidget(&lengthPrecisionSpin);

    self->addSpacing();
    angleUnitCombo.addItem(_("Degree"), DisplayValueFormat::Degree);
    angleUnitCombo.addItem(_("Radian"), DisplayValueFormat::Radian);
    self->addWidget(&angleUnitCombo);
    
    self->addLabel(" . ");
    anglePrecisionSpin.setRange(0, 10);
    anglePrecisionSpin.setToolTip(_("Angle value precision (number of decimals)"));
    self->addWidget(&anglePrecisionSpin);

    self->addSpacing();
    coordinateSystemCombo.addItem(_("Right-handed"), DisplayValueFormat::RightHanded);
    coordinateSystemCombo.addItem(_("Left-handed"), DisplayValueFormat::LeftHanded);
    self->addWidget(&coordinateSystemCombo);

    onDisplayValueFormatChanged(false);

    lengthUnitCombo.sigCurrentIndexChanged().connect(
        [this](int index){ updateLengthUnit(lengthUnitCombo.itemData(index).toInt()); });
    lengthPrecisionSpin.sigValueChanged().connect(
        [this](int decimals){ updateLengthDecimals(decimals); });
    angleUnitCombo.sigCurrentIndexChanged().connect(
        [this](int index){ updateAngleUnit(angleUnitCombo.itemData(index).toInt()); });
    anglePrecisionSpin.sigValueChanged().connect(
        [this](int decimals){ updateAngleDecimals(decimals); });
    coordinateSystemCombo.sigCurrentIndexChanged().connect(
        [this](int index){ updateCoordinateSystem(coordinateSystemCombo.itemData(index).toInt()); });
}


DisplayValueFormatBar::~DisplayValueFormatBar()
{
    delete impl;
}


void DisplayValueFormatBar::Impl::onDisplayValueFormatChanged(bool doBlockSignals)
{
    if(doBlockSignals){
        lengthUnitCombo.blockSignals(true);
        lengthPrecisionSpin.blockSignals(true);
        angleUnitCombo.blockSignals(true);
        anglePrecisionSpin.blockSignals(true);
        coordinateSystemCombo.blockSignals(true);
    }

    int numLengthUnits = lengthUnitCombo.count();
    for(int i=0; i < numLengthUnits; ++i){
        if(lengthUnitCombo.itemData(i).toInt() == displayValueFormat->lengthUnit()){
            lengthUnitCombo.setCurrentIndex(i);
            break;
        }
    }
    lengthPrecisionSpin.setValue(displayValueFormat->lengthDecimals());

    int numAngleUnits = angleUnitCombo.count();
    for(int i=0; i < numAngleUnits; ++i){
        if(angleUnitCombo.itemData(i).toInt() == displayValueFormat->angleUnit()){
            angleUnitCombo.setCurrentIndex(i);
            break;
        }
    }
    anglePrecisionSpin.setValue(displayValueFormat->angleDecimals());

    int numCoordinateSystems = coordinateSystemCombo.count();
    for(int i=0; i < numCoordinateSystems; ++i){
        if(coordinateSystemCombo.itemData(i).toInt() == displayValueFormat->coordinateSystem()){
            coordinateSystemCombo.setCurrentIndex(i);
            break;
        }
    }
    
    if(doBlockSignals){
        lengthUnitCombo.blockSignals(false);
        lengthPrecisionSpin.blockSignals(false);
        angleUnitCombo.blockSignals(false);
        anglePrecisionSpin.blockSignals(false);
        coordinateSystemCombo.blockSignals(false);
    }
}


void DisplayValueFormatBar::Impl::updateLengthUnit(int unitType)
{
    displayValueFormat->setLengthUnit(static_cast<DisplayValueFormat::LengthUnit>(unitType));

    lengthPrecisionSpin.blockSignals(true);
    lengthPrecisionSpin.setValue(displayValueFormat->lengthDecimals());
    lengthPrecisionSpin.blockSignals(false);

    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}


void DisplayValueFormatBar::Impl::updateLengthDecimals(int decimals)
{
    displayValueFormat->setLengthDecimals(decimals);
    displayValueFormat->setLengthStep(pow(10.0, -decimals));
    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}


void DisplayValueFormatBar::Impl::updateAngleUnit(int unitType)
{
    displayValueFormat->setAngleUnit(static_cast<DisplayValueFormat::AngleUnit>(unitType));

    anglePrecisionSpin.blockSignals(true);
    anglePrecisionSpin.setValue(displayValueFormat->angleDecimals());
    anglePrecisionSpin.blockSignals(false);

    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}


void DisplayValueFormatBar::Impl::updateAngleDecimals(int decimals)
{
    displayValueFormat->setAngleDecimals(decimals);
    displayValueFormat->setAngleStep(pow(10.0, -decimals));
    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}


void DisplayValueFormatBar::Impl::updateCoordinateSystem(int system)
{
    displayValueFormat->setCoordinateSystem(static_cast<DisplayValueFormat::CoordinateSystem>(system));
    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}
