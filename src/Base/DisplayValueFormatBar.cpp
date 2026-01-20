#include "DisplayValueFormatBar.h"
#include "ExtensionManager.h"
#include "DisplayValueFormat.h"
#include "ProjectManager.h"
#include "Archive.h"
#include <cnoid/ComboBox>
#include <cnoid/SpinBox>
#include <cnoid/ConnectionSet>
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

    bool hasProjectSettings;
    ScopedConnectionSet projectManagerConnections;

    Impl(DisplayValueFormatBar* self);
    void onDisplayValueFormatChanged(bool doBlockSignals);
    void updateLengthUnit(int unitType);
    void updateLengthDecimals(int decimals);
    void updateAngleUnit(int unitType);
    void updateAngleDecimals(int decimals);
    void updateCoordinateSystem(int system);
    void onProjectCleared();
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
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
    hasProjectSettings = false;

    displayValueFormatConnection =
        displayValueFormat->sigFormatChanged().connect(
            [this](){ onDisplayValueFormatChanged(true); });

    projectManagerConnections.add(
        ProjectManager::instance()->sigProjectCleared().connect(
            [this](){ onProjectCleared(); }));
    
    lengthUnitCombo.addItem(_("Meter"), DisplayValueFormat::Meter);
    lengthUnitCombo.addItem(_("Millimeter"), DisplayValueFormat::Millimeter);
    lengthUnitCombo.addItem(_("Kilometer"), DisplayValueFormat::Kilometer);
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
    hasProjectSettings = true;
    displayValueFormat->setLengthUnit(static_cast<DisplayValueFormat::LengthUnit>(unitType));

    lengthPrecisionSpin.blockSignals(true);
    lengthPrecisionSpin.setValue(displayValueFormat->lengthDecimals());
    lengthPrecisionSpin.blockSignals(false);

    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}


void DisplayValueFormatBar::Impl::updateLengthDecimals(int decimals)
{
    hasProjectSettings = true;
    displayValueFormat->setLengthDecimals(decimals);
    displayValueFormat->setLengthStep(pow(10.0, -decimals));
    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}


void DisplayValueFormatBar::Impl::updateAngleUnit(int unitType)
{
    hasProjectSettings = true;
    displayValueFormat->setAngleUnit(static_cast<DisplayValueFormat::AngleUnit>(unitType));

    anglePrecisionSpin.blockSignals(true);
    anglePrecisionSpin.setValue(displayValueFormat->angleDecimals());
    anglePrecisionSpin.blockSignals(false);

    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}


void DisplayValueFormatBar::Impl::updateAngleDecimals(int decimals)
{
    hasProjectSettings = true;
    displayValueFormat->setAngleDecimals(decimals);
    displayValueFormat->setAngleStep(pow(10.0, -decimals));
    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}


void DisplayValueFormatBar::Impl::updateCoordinateSystem(int system)
{
    hasProjectSettings = true;
    displayValueFormat->setCoordinateSystem(static_cast<DisplayValueFormat::CoordinateSystem>(system));
    auto block = displayValueFormatConnection.scopedBlock();
    displayValueFormat->notifyFormatChange();
}


void DisplayValueFormatBar::Impl::onProjectCleared()
{
    hasProjectSettings = false;
}


bool DisplayValueFormatBar::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool DisplayValueFormatBar::Impl::storeState(Archive& archive)
{
    if(!hasProjectSettings){
        return true;
    }

    const char* lengthUnitSymbol = nullptr;
    switch(displayValueFormat->lengthUnit()){
    case DisplayValueFormat::Meter:      lengthUnitSymbol = "meter"; break;
    case DisplayValueFormat::Millimeter: lengthUnitSymbol = "millimeter"; break;
    case DisplayValueFormat::Kilometer:  lengthUnitSymbol = "kilometer"; break;
    }
    if(lengthUnitSymbol){
        archive.write("length_unit", lengthUnitSymbol);
    }

    archive.write("length_decimals", displayValueFormat->lengthDecimals());
    archive.write("length_step", displayValueFormat->lengthStep());

    const char* angleUnitSymbol = nullptr;
    switch(displayValueFormat->angleUnit()){
    case DisplayValueFormat::Degree: angleUnitSymbol = "degree"; break;
    case DisplayValueFormat::Radian: angleUnitSymbol = "radian"; break;
    }
    if(angleUnitSymbol){
        archive.write("angle_unit", angleUnitSymbol);
    }

    archive.write("angle_decimals", displayValueFormat->angleDecimals());
    archive.write("angle_step", displayValueFormat->angleStep());

    const char* coordinateSystemSymbol = nullptr;
    switch(displayValueFormat->coordinateSystem()){
    case DisplayValueFormat::RightHanded: coordinateSystemSymbol = "right_handed"; break;
    case DisplayValueFormat::LeftHanded:  coordinateSystemSymbol = "left_handed"; break;
    }
    if(coordinateSystemSymbol){
        archive.write("coordinate_system", coordinateSystemSymbol);
    }

    return true;
}


bool DisplayValueFormatBar::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool DisplayValueFormatBar::Impl::restoreState(const Archive& archive)
{
    bool hasSettings = false;
    bool formatChanged = false;

    string lengthUnitSymbol;
    if(archive.read("length_unit", lengthUnitSymbol)){
        int newUnit = displayValueFormat->lengthUnit();
        if(lengthUnitSymbol == "meter"){
            newUnit = DisplayValueFormat::Meter;
            hasSettings = true;
        } else if(lengthUnitSymbol == "millimeter"){
            newUnit = DisplayValueFormat::Millimeter;
            hasSettings = true;
        } else if(lengthUnitSymbol == "kilometer"){
            newUnit = DisplayValueFormat::Kilometer;
            hasSettings = true;
        }
        if(hasSettings && newUnit != displayValueFormat->lengthUnit()){
            displayValueFormat->setLengthUnit(static_cast<DisplayValueFormat::LengthUnit>(newUnit));
            formatChanged = true;
        }
    }

    int lengthDecimals;
    if(archive.read("length_decimals", lengthDecimals)){
        hasSettings = true;
        if(lengthDecimals != displayValueFormat->lengthDecimals()){
            displayValueFormat->setLengthDecimals(lengthDecimals);
            formatChanged = true;
        }
    }

    double lengthStep;
    if(archive.read("length_step", lengthStep)){
        hasSettings = true;
        if(lengthStep != displayValueFormat->lengthStep()){
            displayValueFormat->setLengthStep(lengthStep);
            formatChanged = true;
        }
    }

    string angleUnitSymbol;
    if(archive.read("angle_unit", angleUnitSymbol)){
        int newUnit = displayValueFormat->angleUnit();
        bool validUnit = false;
        if(angleUnitSymbol == "degree"){
            newUnit = DisplayValueFormat::Degree;
            validUnit = true;
        } else if(angleUnitSymbol == "radian"){
            newUnit = DisplayValueFormat::Radian;
            validUnit = true;
        }
        if(validUnit){
            hasSettings = true;
            if(newUnit != displayValueFormat->angleUnit()){
                displayValueFormat->setAngleUnit(static_cast<DisplayValueFormat::AngleUnit>(newUnit));
                formatChanged = true;
            }
        }
    }

    int angleDecimals;
    if(archive.read("angle_decimals", angleDecimals)){
        hasSettings = true;
        if(angleDecimals != displayValueFormat->angleDecimals()){
            displayValueFormat->setAngleDecimals(angleDecimals);
            formatChanged = true;
        }
    }

    double angleStep;
    if(archive.read("angle_step", angleStep)){
        hasSettings = true;
        if(angleStep != displayValueFormat->angleStep()){
            displayValueFormat->setAngleStep(angleStep);
            formatChanged = true;
        }
    }

    string coordinateSystemSymbol;
    if(archive.read("coordinate_system", coordinateSystemSymbol)){
        int newSystem = displayValueFormat->coordinateSystem();
        bool validSystem = false;
        if(coordinateSystemSymbol == "right_handed"){
            newSystem = DisplayValueFormat::RightHanded;
            validSystem = true;
        } else if(coordinateSystemSymbol == "left_handed"){
            newSystem = DisplayValueFormat::LeftHanded;
            validSystem = true;
        }
        if(validSystem){
            hasSettings = true;
            if(newSystem != displayValueFormat->coordinateSystem()){
                displayValueFormat->setCoordinateSystem(static_cast<DisplayValueFormat::CoordinateSystem>(newSystem));
                formatChanged = true;
            }
        }
    }

    if(hasSettings){
        hasProjectSettings = true;
    }
    if(formatChanged){
        displayValueFormat->notifyFormatChange();
    }

    return true;
}
