#include "PositionWidget.h"
#include <cnoid/MenuManager>
#include <cnoid/EigenUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <cnoid/Buttons>
#include <cnoid/SpinBox>
#include <cnoid/Separator>
#include <QLabel>
#include <QGridLayout>
#include <QStyle>
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

    Position T_last;
    std::function<bool(const Position& T)> positionCallback;
    vector<QWidget*> inputElementWidgets;
    ScopedConnectionSet userInputConnections;

    QLabel caption;

    DoubleSpinBox xyzSpin[3];

    enum AttitudeMode { RollPitchYawMode, QuaternionMode };
    AttitudeMode lastInputAttitudeMode;
    
    bool isRpyEnabled;
    bool isUniqueRpyMode;
    Vector3 referenceRpy;
    DoubleSpinBox rpySpin[3];
    vector<QWidget*> rpyWidgets;
    
    bool isQuaternionEnabled;
    DoubleSpinBox quatSpin[4];
    vector<QWidget*> quatWidgets;
    
    bool isRotationMatrixEnabled;
    QWidget rotationMatrixPanel;
    QLabel rotationMatrixElementLabel[3][3];

    Impl(PositionWidget* self);
    void setOptionMenu(MenuManager& menu);
    void setRpyEnabled(bool on);
    void setQuaternionEnabled(bool on);
    void setRotationMatrixEnabled(bool on);
    void resetInputWidgetStyles();
    void clearPosition();
    void refreshPosition();
    void updatePosition(const Position& T);
    void updateRotationMatrix(const Matrix3& R);
    Vector3 getRpySpinValue();
    void onPositionInput(InputElementSet inputElements);
    void onPositionInputRpy(InputElementSet inputElements);
    void onPositionInputQuaternion(InputElementSet inputElements);
    void notifyPositionInput(const Position& T, InputElementSet inputElements);
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
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

    auto style = self->style();
    int lmargin = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int tmargin = style->pixelMetric(QStyle::PM_LayoutTopMargin);
    int rmargin = style->pixelMetric(QStyle::PM_LayoutRightMargin);
    int bmargin = style->pixelMetric(QStyle::PM_LayoutBottomMargin);

    auto mainvbox = new QVBoxLayout;
    mainvbox->setContentsMargins(lmargin / 2, tmargin / 2, rmargin / 2, bmargin / 2);
    self->setLayout(mainvbox);

    auto grid = new QGridLayout;
    int row = 0;

    caption.setStyleSheet("font-weight: bold");
    grid->addWidget(&caption, row++, 1, 1, 5);

    static const char* xyzLabels[] = { "X", "Y", "Z" };
    for(int i=0; i < 3; ++i){
        // Translation spin boxes
        xyzSpin[i].setAlignment(Qt::AlignCenter);
        xyzSpin[i].setDecimals(3);
        xyzSpin[i].setRange(-99.999, 99.999);
        xyzSpin[i].setSingleStep(0.001);

        InputElementSet s;
        s.set(TX + i);
        userInputConnections.add(
            xyzSpin[i].sigValueChanged().connect(
                [this, s](double){ onPositionInput(s); }));

        grid->addWidget(new QLabel(xyzLabels[i]), row, i * 2, Qt::AlignCenter);
        grid->addWidget(&xyzSpin[i], row, i * 2 + 1);

        grid->setColumnStretch(i * 2, 1);
        grid->setColumnStretch(i * 2 + 1, 10);
    }
    ++row;

    static const char* rpyLabelChar[] = { "R", "P", "Y" };
    for(int i=0; i < 3; ++i){
        // Roll-pitch-yaw spin boxes
        rpySpin[i].setAlignment(Qt::AlignCenter);
        rpySpin[i].setDecimals(1);
        rpySpin[i].setRange(-9999.0, 9999.0);
        rpySpin[i].setSingleStep(0.1);

        InputElementSet s;
        s.set(RX + 1);
        userInputConnections.add(
            rpySpin[i].sigValueChanged().connect(
                [this, s](double){ onPositionInputRpy(s); }));

        auto label = new QLabel(rpyLabelChar[i]);
        grid->addWidget(label, row, i * 2, Qt::AlignCenter);
        rpyWidgets.push_back(label);
        grid->addWidget(&rpySpin[i], row, i * 2 + 1);
        rpyWidgets.push_back(&rpySpin[i]);
    }
    ++row;
    
    mainvbox->addLayout(grid);

    grid = new QGridLayout;
    static const char* quatLabelChar[] = {"QX", "QY", "QZ", "QW"};
    for(int i=0; i < 4; ++i){
        quatSpin[i].setAlignment(Qt::AlignCenter);
        quatSpin[i].setDecimals(4);
        quatSpin[i].setRange(-1.0000, 1.0000);
        quatSpin[i].setSingleStep(0.0001);

        InputElementSet s;
        s.set(QX + i);
        userInputConnections.add(
            quatSpin[i].sigValueChanged().connect(
                [this, s](double){ onPositionInputQuaternion(s); }));
        
        auto label = new QLabel(quatLabelChar[i]);
        grid->addWidget(label, 0, i * 2, Qt::AlignRight);
        quatWidgets.push_back(label);
        grid->addWidget(&quatSpin[i], 0, i * 2 + 1);
        quatWidgets.push_back(&quatSpin[i]);

        grid->setColumnStretch(i * 2, 1);
        grid->setColumnStretch(i * 2 + 1, 10);
    }
    mainvbox->addLayout(grid);

    inputElementWidgets = {
        &xyzSpin[0], &xyzSpin[1], &xyzSpin[2],
        &rpySpin[0], &rpySpin[1], &rpySpin[2],
        &quatSpin[0], &quatSpin[1], &quatSpin[2], &quatSpin[3]
    };

    auto hbox = new QHBoxLayout;
    rotationMatrixPanel.setLayout(hbox);
    hbox->addStretch();
    //hbox->addWidget(new QLabel("R = "));
    hbox->addWidget(new VSeparator);

    grid = new QGridLayout();
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(4);
    for(int i=0; i < 3; ++i){
        for(int j=0; j < 3; ++j){
            auto& label = rotationMatrixElementLabel[i][j];
            QFont font("Monospace");
            font.setStyleHint(QFont::TypeWriter);
            label.setFont(font);
            label.setTextInteractionFlags(Qt::TextSelectableByMouse);
            grid->addWidget(&label, i, j);
        }
    }
    hbox->addLayout(grid);
    hbox->addWidget(new VSeparator);
    hbox->addStretch();

    isRpyEnabled = true;
    isUniqueRpyMode = false;
    isQuaternionEnabled = true;
    setQuaternionEnabled(false);
    isRotationMatrixEnabled = true;
    setRotationMatrixEnabled(false);
    
    mainvbox->addWidget(&rotationMatrixPanel);
    mainvbox->addStretch();

    T_last.setIdentity();
    
    lastInputAttitudeMode = RollPitchYawMode;

    clearPosition();
}


void PositionWidget::setCaption(const std::string& caption)
{
    impl->caption.setText(caption.c_str());
}


void PositionWidget::setOptionMenu(MenuManager& menuManager)
{
    impl->setOptionMenu(menuManager);
}


void PositionWidget::Impl::setOptionMenu(MenuManager& menu)
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


void PositionWidget::setPositionCallback(std::function<bool(const Position& T)> callback)
{
    impl->positionCallback = callback;
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
    for(auto& widget : inputElementWidgets){
        widget->setStyleSheet(normalStyle);
    }
}


void PositionWidget::clearPosition()
{
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
    updatePosition(T_last);
}


void PositionWidget::updatePosition(const Position& T)
{
    impl->updatePosition(T);
}


void PositionWidget::Impl::updatePosition(const Position& T)
{
    userInputConnections.block();

    Vector3 p = T.translation();
    for(int i=0; i < 3; ++i){
        auto& spin = xyzSpin[i];
        if(!spin.hasFocus()){
            spin.setValue(p[i]);
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
                rpy = rpyFromRot(R, getRpySpinValue());
            }
            referenceRpy = rpy;
        }
        for(int i=0; i < 3; ++i){
            rpySpin[i].setValue(degree(rpy[i]));
        }
    }
    
    if(isQuaternionEnabled){
        if(!quatSpin[0].hasFocus() &&
           !quatSpin[1].hasFocus() &&
           !quatSpin[2].hasFocus() &&
           !quatSpin[3].hasFocus()){
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


Vector3 PositionWidget::Impl::getRpySpinValue()
{
    Vector3 rpy;
    for(int i=0; i < 3; ++i){
        rpy[i] = radian(rpySpin[i].value());
    }
    return rpy;
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
    Position T;
    Vector3 rpy;

    for(int i=0; i < 3; ++i){
        T.translation()[i] = xyzSpin[i].value();
        rpy[i] = radian(rpySpin[i].value());
    }
    T.linear() = rotFromRpy(rpy);
    
    notifyPositionInput(T, inputElements);

    lastInputAttitudeMode = RollPitchYawMode;
}


void PositionWidget::Impl::onPositionInputQuaternion(InputElementSet inputElements)
{
    Position T;

    for(int i=0; i < 3; ++i){
        T.translation()[i] = xyzSpin[i].value();
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


void PositionWidget::Impl::notifyPositionInput(const Position& T, InputElementSet inputElements)
{
    if(!positionCallback(T)){
        for(size_t i=0; i < inputElementWidgets.size(); ++i){
            if(inputElements[i]){
                inputElementWidgets[i]->setStyleSheet(errorStyle);
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
    archive.write("showRPY", isRpyEnabled);
    archive.write("uniqueRPY", isUniqueRpyMode);
    archive.write("showQuoternion", isQuaternionEnabled);
    archive.write("showRotationMatrix", isRotationMatrixEnabled);
}


void PositionWidget::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


void PositionWidget::Impl::restoreState(const Archive& archive)
{
    userInputConnections.block();
    
    setRpyEnabled(archive.get("showRPY", isRpyEnabled));
    archive.read("uniqueRPY", isUniqueRpyMode);
    setQuaternionEnabled(archive.get("showQuoternion", isQuaternionEnabled));
    setRotationMatrixEnabled(archive.get("showRotationMatrix", isRotationMatrixEnabled));

    userInputConnections.unblock();
}
