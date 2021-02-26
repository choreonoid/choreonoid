#include "PositionWidget.h"
#include "DisplayedValueFormatManager.h"
#include "MenuManager.h"
#include "Archive.h"
#include "Buttons.h"
#include "SpinBox.h"
#include "Separator.h"
#include <cnoid/EigenUtil>
#include <cnoid/ConnectionSet>
#include <QLabel>
#include <QGridLayout>
#include <QMouseEvent>
#include <fmt/format.h>
#include <bitset>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

enum InputElement {
    TX, TY, TZ,
    RX, RY, RZ,
    QX, QY, QZ, QW,
    NumInputElements
};

typedef std::bitset<NumInputElements> InputElementSet;

const char* normalStyle = "font-weight: normal";
const char* errorStyle = "font-weight: bold; color: red";

}

namespace cnoid {

class PositionWidget::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PositionWidget* self;

    Isometry3 T_last;
    std::function<bool(const Isometry3& T)> callbackOnPositionInput;
    std::function<void()> callbackOnPositionInputFinished;
    //vector<QWidget*> inputPanelWidgets;
    vector<DoubleSpinBox*> inputSpins;
    vector<bool> inputSpinErrorStates;
    ScopedConnectionSet userInputConnections;

    DisplayedValueFormatManager* valueFormatManager;
    ScopedConnection valueFormatManagerConnection;
    double lengthRatio;
    double angleRatio;
    
    QVBoxLayout* mainvbox;
    DoubleSpinBox xyzSpin[3];
    enum AttitudeMode { RollPitchYawMode, QuaternionMode };
    AttitudeMode lastInputAttitudeMode;
    bool isUserInputValuePriorityMode;
    bool isRpyEnabled;
    bool isUniqueRpyMode;
    bool isQuaternionEnabled;
    bool isRotationMatrixEnabled;
    Vector3 referenceRpy;
    DoubleSpinBox rpySpin[3];
    vector<QWidget*> rpyWidgets;
    DoubleSpinBox quatSpin[4];
    vector<QWidget*> quatWidgets;
    QWidget rotationMatrixPanel;
    QLabel rotationMatrixElementLabel[3][3];

    Impl(PositionWidget* self);
    void updateValueFormat(bool doRefresh);
    void setOptionMenuTo(MenuManager& menu);
    void setRpyEnabled(bool on);
    void setQuaternionEnabled(bool on);
    void setRotationMatrixEnabled(bool on);
    void resetInputWidgetStyles();
    void clearPosition();
    void refreshPosition();
    void setPosition(const Isometry3& T);
    void updateRotationMatrix(const Matrix3& R);
    Vector3 getRpyInput();
    void onPositionInput(InputElementSet inputElements);
    void onPositionInputRpy(InputElementSet inputElements);
    void onPositionInputQuaternion(InputElementSet inputElements);
    void notifyPositionInput(const Isometry3& T, InputElementSet inputElements);
    void onPositionInputFinished();
    void storeState(Archive& archive);
    void restoreState(const Archive& archive);
};

}


PositionWidget::PositionWidget(QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this);
}


PositionWidget::~PositionWidget()
{
    delete impl;
}


PositionWidget::Impl::Impl(PositionWidget* self)
    : self(self)
{
    valueFormatManager = DisplayedValueFormatManager::instance();
    valueFormatManagerConnection =
        valueFormatManager->sigFormatChanged().connect(
            [&](){ updateValueFormat(true); });

    mainvbox = new QVBoxLayout;
    mainvbox->setContentsMargins(0, 0, 0, 0);
    self->setLayout(mainvbox);

    auto grid = new QGridLayout;
    int row = 0;

    static const char* xyzLabels[] = { "X", "Y", "Z" };
    for(int i=0; i < 3; ++i){
        // Translation spin boxes
        auto spin = &xyzSpin[i];
        spin->setAlignment(Qt::AlignCenter);
        spin->setUndoRedoKeyInputEnabled(true);

        InputElementSet s;
        s.set(TX + i);
        userInputConnections.add(
            spin->sigValueChanged().connect(
                [this, s](double){ onPositionInput(s); }));
        userInputConnections.add(
            spin->sigEditingFinishedWithValueChange().connect(
                [this](){ onPositionInputFinished(); }));

        auto label = new QLabel(xyzLabels[i]);
        grid->addWidget(label, row, i * 2, Qt::AlignCenter);
        grid->addWidget(spin, row, i * 2 + 1);

        grid->setColumnStretch(i * 2, 1);
        grid->setColumnStretch(i * 2 + 1, 10);

        inputSpins.push_back(spin);
        //inputPanelWidgets.push_back(label);
        //inputPanelWidgets.push_back(spin);
    }
    ++row;

    static const char* rpyLabelChar[] = { "R", "P", "Y" };
    
    for(int i=0; i < 3; ++i){
        // Roll-pitch-yaw spin boxes
        auto spin = &rpySpin[i];
        spin->setAlignment(Qt::AlignCenter);
        spin->setUndoRedoKeyInputEnabled(true);

        InputElementSet s;
        s.set(RX + 1);
        userInputConnections.add(
            spin->sigValueChanged().connect(
                [this, s](double){ onPositionInputRpy(s); }));
        userInputConnections.add(
            spin->sigEditingFinishedWithValueChange().connect(
                [this](){ onPositionInputFinished(); }));

        auto label = new QLabel(rpyLabelChar[i]);
        grid->addWidget(label, row, i * 2, Qt::AlignCenter);
        grid->addWidget(spin, row, i * 2 + 1);

        inputSpins.push_back(spin);
        rpyWidgets.push_back(label);
        rpyWidgets.push_back(spin);
        //inputPanelWidgets.push_back(label);
        //inputPanelWidgets.push_back(spin);
    }
    ++row;
    
    mainvbox->addLayout(grid);

    grid = new QGridLayout;
    static const char* quatLabelChar[] = {"QX", "QY", "QZ", "QW"};
    for(int i=0; i < 4; ++i){
        auto spin = &quatSpin[i];
        spin->setAlignment(Qt::AlignCenter);
        spin->setDecimals(4);
        spin->setRange(-1.0000, 1.0000);
        spin->setSingleStep(0.0001);
        spin->setUndoRedoKeyInputEnabled(true);

        InputElementSet s;
        s.set(QX + i);
        userInputConnections.add(
            spin->sigValueChanged().connect(
                [this, s](double){ onPositionInputQuaternion(s); }));
        userInputConnections.add(
            spin->sigEditingFinishedWithValueChange().connect(
                [this](){ onPositionInputFinished(); }));
        
        auto label = new QLabel(quatLabelChar[i]);
        grid->addWidget(label, 0, i * 2, Qt::AlignRight);
        grid->addWidget(&quatSpin[i], 0, i * 2 + 1);
        grid->setColumnStretch(i * 2, 1);
        grid->setColumnStretch(i * 2 + 1, 10);

        inputSpins.push_back(spin);
        quatWidgets.push_back(label);
        quatWidgets.push_back(spin);
        //inputPanelWidgets.push_back(label);
        //inputPanelWidgets.push_back(spin);
    }
    inputSpinErrorStates.resize(inputSpins.size(), false);
    mainvbox->addLayout(grid);

    auto hbox = new QHBoxLayout;
    rotationMatrixPanel.setLayout(hbox);
    hbox->addStretch();
    //hbox->addWidget(new QLabel("R = "));
    auto separator = new VSeparator;
    hbox->addWidget(separator);

    grid = new QGridLayout();
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(4);
    for(int i=0; i < 3; ++i){
        for(int j=0; j < 3; ++j){
            auto label = &rotationMatrixElementLabel[i][j];
            QFont font("Monospace");
            font.setStyleHint(QFont::TypeWriter);
            label->setFont(font);
            label->setTextInteractionFlags(Qt::TextSelectableByMouse);
            grid->addWidget(label, i, j);
        }
    }
    hbox->addLayout(grid);
    separator = new VSeparator;
    hbox->addWidget(separator);
    hbox->addStretch();

    isUserInputValuePriorityMode = false;
    isRpyEnabled = true;
    isUniqueRpyMode = false;
    referenceRpy.setZero();
    isQuaternionEnabled = true;
    setQuaternionEnabled(false);
    isRotationMatrixEnabled = true;
    setRotationMatrixEnabled(false);
    
    mainvbox->addWidget(&rotationMatrixPanel);
    //inputPanelWidgets.push_back(&rotationMatrixPanel);

    T_last.setIdentity();
    lastInputAttitudeMode = RollPitchYawMode;

    updateValueFormat(false);
    clearPosition();
}


void PositionWidget::Impl::updateValueFormat(bool doRefresh)
{
    int lunit = valueFormatManager->lengthUnit();
    double lmax;
    if(lunit == DisplayedValueFormatManager::Millimeter){
        lengthRatio = 1000.0;
        lmax = 100000.0;
    } else {
        lengthRatio = 1.0;
        lmax = 100.0;
    }
    int ldecimals = valueFormatManager->lengthDecimals();
    lmax -= pow(10.0, -ldecimals);
    double lstep = valueFormatManager->lengthStep();

    int aunit = valueFormatManager->angleUnit();
    double amax;
    if(aunit == DisplayedValueFormatManager::Degree){
        angleRatio = 180.0 / PI;
        amax = 1000.0;
    } else {
        angleRatio = 1.0;
        amax = 10.0;
    }
    int adecimals = valueFormatManager->angleDecimals();
    amax -= pow(10.0, -adecimals);
    double astep = valueFormatManager->angleStep();

    for(int i=0; i < 3; ++i){
        auto& tspin = xyzSpin[i];
        tspin.setDecimals(ldecimals);
        tspin.setRange(-lmax, lmax);
        tspin.setSingleStep(lstep);

        auto& aspin = rpySpin[i];
        aspin.setDecimals(adecimals);
        aspin.setRange(-amax, amax);
        aspin.setSingleStep(astep);
    }

    if(doRefresh){
        refreshPosition();
    }
}


void PositionWidget::setOptionMenuTo(MenuManager& menuManager)
{
    impl->setOptionMenuTo(menuManager);
}


void PositionWidget::Impl::setOptionMenuTo(MenuManager& menu)
{
    auto rpyCheck = menu.addCheckItem(_("Roll-pitch-yaw"));
    rpyCheck->setChecked(isRpyEnabled);
    rpyCheck->sigToggled().connect(
        [&](bool on){ setRpyEnabled(on); refreshPosition(); });

    auto uniqueRpyCheck = menu.addCheckItem(_("Fetch as a unique RPY value"));
    uniqueRpyCheck->setChecked(isUniqueRpyMode);
    uniqueRpyCheck->sigToggled().connect(
        [&](bool on){ isUniqueRpyMode = on; refreshPosition(); });
    
    auto quaternionCheck = menu.addCheckItem(_("Quoternion"));
    quaternionCheck->setChecked(isQuaternionEnabled);
    quaternionCheck->sigToggled().connect(
        [&](bool on){ setQuaternionEnabled(on); refreshPosition(); });

    auto rotationMatrixCheck = menu.addCheckItem(_("Rotation matrix"));
    rotationMatrixCheck->setChecked(isRotationMatrixEnabled);
    rotationMatrixCheck->sigToggled().connect(
        [&](bool on){ setRotationMatrixEnabled(on); refreshPosition(); });
}


void PositionWidget::setEditable(bool on)
{
    for(auto& spin : impl->inputSpins){
        spin->setReadOnly(!on);
    }
    /*
    for(auto& widget : impl->inputPanelWidgets){
        widget->setEnabled(on);
    }
    */
}


void PositionWidget::setUserInputValuePriorityMode(bool on)
{
    impl->isUserInputValuePriorityMode = on;
}
    

void PositionWidget::setCallbacks
(std::function<bool(const Isometry3& T)> callbackOnPositionInput,
 std::function<void()> callbackOnPositionInputFinished)
{
    impl->callbackOnPositionInput = callbackOnPositionInput;
    impl->callbackOnPositionInputFinished = callbackOnPositionInputFinished;
}


void PositionWidget::setPositionCallback(std::function<bool(const Isometry3& T)> callback)
{
    impl->callbackOnPositionInput = callback;
}


void PositionWidget::Impl::setRpyEnabled(bool on)
{
    if(on != isRpyEnabled){
        isRpyEnabled = on;
        for(auto& widget : rpyWidgets){
            widget->setVisible(on);
        }
    }
}


void PositionWidget::Impl::setQuaternionEnabled(bool on)
{
    if(on != isQuaternionEnabled){
        isQuaternionEnabled = on;
        for(auto& widget : quatWidgets){
            widget->setVisible(on);
        }
    }
}


void PositionWidget::Impl::setRotationMatrixEnabled(bool on)
{
    isRotationMatrixEnabled = on;
    rotationMatrixPanel.setVisible(on);
}


void PositionWidget::Impl::resetInputWidgetStyles()
{
    for(size_t i=0; i < inputSpins.size(); ++i){
        inputSpins[i]->setStyleSheet(normalStyle);
        inputSpinErrorStates[i] = false;
    }
}


void PositionWidget::clearPosition()
{
    impl->resetInputWidgetStyles();
    impl->clearPosition();
}


void PositionWidget::Impl::clearPosition()
{
    userInputConnections.block();
    
    for(int i=0; i < 3; ++i){
        xyzSpin[i].setValue(0.0);
        rpySpin[i].setValue(0.0);
        quatSpin[i].setValue(0.0);
    }
    quatSpin[3].setValue(1.0);
    updateRotationMatrix(Matrix3::Identity());

    userInputConnections.unblock();
}


void PositionWidget::refreshPosition()
{
    impl->refreshPosition();
}


void PositionWidget::Impl::refreshPosition()
{
    setPosition(T_last);
}


void PositionWidget::setReferenceRpy(const Vector3& rpy)
{
    impl->referenceRpy = rpy;
}


void PositionWidget::setPosition(const Isometry3& T)
{
    impl->setPosition(T);
}


void PositionWidget::Impl::setPosition(const Isometry3& T)
{
    userInputConnections.block();

    Vector3 p = T.translation();
    for(int i=0; i < 3; ++i){
        auto& spin = xyzSpin[i];
        if(!isUserInputValuePriorityMode || !spin.hasFocus()){
            spin.setValue(lengthRatio * p[i]);
        }
    }

    Matrix3 R = T.linear();
    if(isRpyEnabled){
        Vector3 rpy;
        if(isUniqueRpyMode){
            rpy = rpyFromRot(R);
        } else {
            if(T.linear().isApprox(rotFromRpy(referenceRpy))){
                rpy = rpyFromRot(R, referenceRpy);
            } else {
                rpy = rpyFromRot(R, getRpyInput());
            }
            referenceRpy = rpy;
        }
        for(int i=0; i < 3; ++i){
            rpySpin[i].setValue(angleRatio * rpy[i]);
        }
    }
    
    if(isQuaternionEnabled){
        bool skipUpdate = false;
        if(isUserInputValuePriorityMode){
            for(int i=0; i < 4; ++i){
                if(quatSpin[i].hasFocus()){
                    skipUpdate = true;
                    break;
                }
            }
        }
        if(!skipUpdate){
            Eigen::Quaterniond quat(R);
            quatSpin[0].setValue(quat.x());
            quatSpin[1].setValue(quat.y());
            quatSpin[2].setValue(quat.z());
            quatSpin[3].setValue(quat.w());
        }
    }
    
    if(isRotationMatrixEnabled){
        updateRotationMatrix(R);
    }

    resetInputWidgetStyles();

    userInputConnections.unblock();

    T_last = T;
}


void PositionWidget::Impl::updateRotationMatrix(const Matrix3& R)
{
    for(int i=0; i < 3; ++i){
        for(int j=0; j < 3; ++j){
            rotationMatrixElementLabel[i][j].setText(
                format("{: .6f}", R(i, j)).c_str());
        }
    }
}


Vector3 PositionWidget::getRpyInput() const
{
    return impl->getRpyInput();
}


Vector3 PositionWidget::Impl::getRpyInput()
{
    Vector3 rpy;
    for(int i=0; i < 3; ++i){
        rpy[i] = rpySpin[i].value() / angleRatio;
    }
    return rpy;
}


void PositionWidget::applyPositionInput()
{
    impl->onPositionInput(InputElementSet(0));
}


void PositionWidget::Impl::onPositionInput(InputElementSet inputElements)
{
    if(lastInputAttitudeMode == RollPitchYawMode && isRpyEnabled){
        onPositionInputRpy(inputElements);
    } else if(lastInputAttitudeMode == QuaternionMode && isQuaternionEnabled){
        onPositionInputQuaternion(inputElements);
    } else if(isQuaternionEnabled){
        onPositionInputQuaternion(inputElements);
    } else {
        onPositionInputRpy(inputElements);
    }
}


void PositionWidget::Impl::onPositionInputRpy(InputElementSet inputElements)
{
    Isometry3 T;
    Vector3 rpy;

    for(int i=0; i < 3; ++i){
        T.translation()[i] = xyzSpin[i].value() / lengthRatio;
        rpy[i] = rpySpin[i].value() / angleRatio;
    }
    T.linear() = rotFromRpy(rpy);
    
    notifyPositionInput(T, inputElements);

    lastInputAttitudeMode = RollPitchYawMode;
}


void PositionWidget::Impl::onPositionInputQuaternion(InputElementSet inputElements)
{
    Isometry3 T;

    for(int i=0; i < 3; ++i){
        T.translation()[i] = xyzSpin[i].value() / lengthRatio;
    }
    
    Eigen::Quaterniond quat =
        Eigen::Quaterniond(
            quatSpin[3].value(), quatSpin[0].value(), quatSpin[1].value(), quatSpin[2].value());

    if(quat.norm() > 1.0e-6){
        quat.normalize();
        T.linear() = quat.toRotationMatrix();
        notifyPositionInput(T, inputElements);
    }

    lastInputAttitudeMode = QuaternionMode;
}


void PositionWidget::Impl::notifyPositionInput(const Isometry3& T, InputElementSet inputElements)
{
    bool accepted = callbackOnPositionInput(T);

    if(!accepted){
        for(size_t i=0; i < inputSpins.size(); ++i){
            if(inputElements[i]){
                if(!inputSpinErrorStates[i]){
                    inputSpins[i]->setStyleSheet(errorStyle);
                    inputSpinErrorStates[i] = true;
                }
            } else {
                if(inputSpinErrorStates[i]){
                    inputSpins[i]->setStyleSheet(normalStyle);
                    inputSpinErrorStates[i] = false;
                }
            }
        }
    }
}


void PositionWidget::Impl::onPositionInputFinished()
{
    if(callbackOnPositionInputFinished){
        callbackOnPositionInputFinished();
    }
}


void PositionWidget::setErrorHighlight(bool on)
{
    for(size_t i=0; i < impl->inputSpins.size(); ++i){
        if(on){
            if(!impl->inputSpinErrorStates[i]){
                impl->inputSpins[i]->setStyleSheet(errorStyle);
                impl->inputSpinErrorStates[i] = true;
            }
        } else {
            if(impl->inputSpinErrorStates[i]){
                impl->inputSpins[i]->setStyleSheet(normalStyle);
                impl->inputSpinErrorStates[i] = false;
            }
        }
    }
}


void PositionWidget::storeState(Archive& archive)
{
    impl->storeState(archive);
}


void PositionWidget::Impl::storeState(Archive& archive)
{
    archive.write("show_rpy", isRpyEnabled);
    archive.write("unique_rpy", isUniqueRpyMode);
    archive.write("show_quoternion", isQuaternionEnabled);
    archive.write("show_rotation_matrix", isRotationMatrixEnabled);
}


void PositionWidget::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


void PositionWidget::Impl::restoreState(const Archive& archive)
{
    userInputConnections.block();
    
    setRpyEnabled(archive.get("show_rpy", isRpyEnabled));
    archive.read("unique_rpy", isUniqueRpyMode);
    setQuaternionEnabled(archive.get("show_quoternion", isQuaternionEnabled));
    setRotationMatrixEnabled(archive.get("show_rotation_matrix", isRotationMatrixEnabled));

    userInputConnections.unblock();
}
