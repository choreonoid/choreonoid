/** \file
    \author Shin'ichiro Nakaoka
*/

#include "LinkPositionView.h"
#include "BodyBar.h"
#include "LinkSelectionView.h"
#include <cnoid/Body>
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/JointPathConfigurationHandler>
#include <cnoid/CompositeBodyIK>
#include <cnoid/EigenUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/RootItem>
#include <cnoid/Archive>
#include <cnoid/Buttons>
#include <cnoid/SpinBox>
#include <cnoid/CheckBox>
#include <cnoid/ComboBox>
#include <cnoid/Separator>
#include <cnoid/ButtonGroup>
#include <cnoid/Selection>
#include <QLabel>
#include <QGridLayout>
#include <QGroupBox>
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
    NUM_INPUT_ELEMENTS
};

typedef std::bitset<NUM_INPUT_ELEMENTS> InputElementSet;

const char* normalStyle = "font-weight: normal";
const char* errorStyle = "font-weight: bold; color: red";

}

namespace cnoid {

class LinkPositionViewImpl
{
public:
    LinkPositionView* self;
            
    BodyItemPtr targetBodyItem;
    Link* targetLink;
    shared_ptr<InverseKinematics> inverseKinematics;
    shared_ptr<JointPathConfigurationHandler> jointPathConfigurationHandler;

    LinkSelectionView* linkSelectionView;

    ToolButton menuButton;
    MenuManager menuManager;
    QLabel targetLabel;
    QLabel configurationLabel;
    QLabel resultLabel;
    enum { BASE_COORD, ROOT_COORD, OBJECT_COORD, NUM_COORD_MODES };
    Selection coordinateMode;
    ButtonGroup coordinateModeGroup;
    RadioButton baseCoordRadio;
    RadioButton rootCoordRadio;
    RadioButton objectCoordRadio;
    DoubleSpinBox xyzSpin[3];
    Action* rpyCheck;
    Action* uniqueRpyCheck;
    DoubleSpinBox rpySpin[3];
    vector<QWidget*> rpyWidgets;
    Action* quaternionCheck;
    DoubleSpinBox quatSpin[4];
    vector<QWidget*> quatWidgets;
    Action* rotationMatrixCheck;
    QWidget rotationMatrixPanel;
    QLabel rotationMatrixElementLabel[3][3];
    Action* disableCustomIKCheck;

    vector<QWidget*> inputElementWidgets;
        
    enum AttitudeType { ROLL_PITCH_YAW, QUATERNION };
    AttitudeType lastInputAttitudeType;

    ComboBox userCoordCombo;
    ComboBox toolCoordCombo;
    ComboBox configurationCombo;
    CheckBox requireConfigurationCheck;
    vector<QWidget*> configurationWidgets;

    ScopedConnection bodyBarConnection;
    ScopedConnectionSet bodyItemConnections;
    ScopedConnectionSet userInputConnections;
    ScopedConnectionSet settingConnections;

    LinkPositionViewImpl(LinkPositionView* self);
    ~LinkPositionViewImpl();
    void createPanel();
    void resetInputWidgetStyles();
    void onMenuButtonClicked();
    void setRpySpinsVisible(bool on);
    void setQuaternionSpinsVisible(bool on);
    void setBodyItem(BodyItem* bodyItem);
    void updateTargetLink(Link* link = nullptr);
    void updatePanel();
    Link* findUniqueEndLink(Body* body) const;
    void updateRotationMatrixPanel(const Matrix3& R);
    void updateConfigurationPanel();
    void onPositionInput(InputElementSet inputElements);
    void onPositionInputRpy(InputElementSet inputElements);
    void onPositionInputQuaternion(InputElementSet inputElements);
    void onConfigurationInput(int index);
    void findSolution(const Position& T_input, InputElementSet inputElements);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}


void LinkPositionView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LinkPositionView>(
        "LinkPositionView", N_("Link Position"), ViewManager::SINGLE_OPTIONAL);
}


LinkPositionView::LinkPositionView()
{
    impl = new LinkPositionViewImpl(this);
}


LinkPositionViewImpl::LinkPositionViewImpl(LinkPositionView* self)
    : self(self),
      coordinateMode(NUM_COORD_MODES, CNOID_GETTEXT_DOMAIN_NAME)
{
    self->setDefaultLayoutArea(View::CENTER);
    createPanel();
    self->setEnabled(false);

    linkSelectionView = LinkSelectionView::instance();
}


LinkPositionView::~LinkPositionView()
{
    delete impl;
}


LinkPositionViewImpl::~LinkPositionViewImpl()
{

}


void LinkPositionViewImpl::createPanel()
{
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

    auto style = self->style();
    int lmargin = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int rmargin = style->pixelMetric(QStyle::PM_LayoutRightMargin);
    int tmargin = style->pixelMetric(QStyle::PM_LayoutTopMargin);
    int bmargin = style->pixelMetric(QStyle::PM_LayoutBottomMargin);
    
    auto topvbox = new QVBoxLayout;
    self->setLayout(topvbox);
    
    auto mainvbox = new QVBoxLayout;
    mainvbox->setContentsMargins(lmargin / 2, rmargin / 2, tmargin / 2, bmargin / 2);
    topvbox->addLayout(mainvbox);

    auto hbox = new QHBoxLayout;
    hbox->addStretch(2);
    targetLabel.setStyleSheet("font-weight: bold");
    targetLabel.setAlignment(Qt::AlignLeft);
    hbox->addWidget(&targetLabel);
    hbox->addStretch(1);
    configurationLabel.setAlignment(Qt::AlignLeft);
    hbox->addWidget(&configurationLabel);
    hbox->addStretch(10);
    
    menuButton.setText("*");
    menuButton.setToolTip(_("Option"));
    menuButton.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    menuButton.sigClicked().connect([&](){ onMenuButtonClicked(); });
    hbox->addWidget(&menuButton);
    mainvbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    resultLabel.setFrameStyle(QFrame::Box | QFrame::Sunken);
    resultLabel.setAlignment(Qt::AlignCenter);
    hbox->addWidget(&resultLabel, 1);
    auto actualButton = new PushButton(_("Fetch"));
    actualButton->sigClicked().connect([&](){ updatePanel(); });
    hbox->addWidget(actualButton);
    auto applyButton = new PushButton(_("Apply"));
    InputElementSet s;
    s.set();
    applyButton->sigClicked().connect([this, s](){ onPositionInput(s); });
    hbox->addWidget(applyButton);
    mainvbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Coordinate:")));
    
    coordinateMode.setSymbol(BASE_COORD, "base");
    baseCoordRadio.setText(_("Base"));
    baseCoordRadio.setChecked(true);
    hbox->addWidget(&baseCoordRadio);
    coordinateModeGroup.addButton(&baseCoordRadio, BASE_COORD);

    coordinateMode.setSymbol(ROOT_COORD, "root");
    rootCoordRadio.setText(_("Root"));
    hbox->addWidget(&rootCoordRadio);
    coordinateModeGroup.addButton(&rootCoordRadio, ROOT_COORD);
    
    coordinateMode.setSymbol(OBJECT_COORD, "object");
    objectCoordRadio.setText(_("Object"));
    hbox->addWidget(&objectCoordRadio);
    coordinateModeGroup.addButton(&objectCoordRadio, OBJECT_COORD);
    hbox->addStretch();
    mainvbox->addLayout(hbox);

    auto grid = new QGridLayout;

    static const char* xyzLabels[] = { "X", "Y", "Z" };
    for(int i=0; i < 3; ++i){
        // Translation spin boxes
        xyzSpin[i].setAlignment(Qt::AlignCenter);
        xyzSpin[i].setDecimals(4);
        xyzSpin[i].setRange(-99.9999, 99.9999);
        xyzSpin[i].setSingleStep(0.0001);

        InputElementSet s;
        s.set(TX + i);
        userInputConnections.add(
            xyzSpin[i].sigValueChanged().connect(
                [this, s](double){ onPositionInput(s); }));

        grid->addWidget(new QLabel(xyzLabels[i]), 0, i * 2, Qt::AlignCenter);
        grid->addWidget(&xyzSpin[i], 0, i * 2 + 1);

        grid->setColumnStretch(i * 2, 1);
        grid->setColumnStretch(i * 2 + 1, 10);
    }

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
        grid->addWidget(label, 1, i * 2, Qt::AlignCenter);
        rpyWidgets.push_back(label);
        grid->addWidget(&rpySpin[i], 1, i * 2 + 1);
        rpyWidgets.push_back(&rpySpin[i]);
    }
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

    hbox = new QHBoxLayout;
    rotationMatrixPanel.setLayout(hbox);
    hbox->addStretch();
    hbox->addWidget(new QLabel("R = "));
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
    updateRotationMatrixPanel(Matrix3::Identity());
    hbox->addLayout(grid);
    hbox->addWidget(new VSeparator);
    hbox->addStretch();
    
    mainvbox->addWidget(&rotationMatrixPanel);

    grid = new QGridLayout;
    grid->setColumnStretch(1, 1);

    grid->addWidget(new QLabel(_("Base")), 0, 0, Qt::AlignLeft);
    userCoordCombo.addItem(_("World"));
    grid->addWidget(&userCoordCombo, 0, 1);
    grid->addWidget(new QPushButton(_("Edit")), 0, 2);

    grid->addWidget(new QLabel(_("Target")), 1, 0, Qt::AlignLeft);
    toolCoordCombo.addItem(_("Link Origin"));
    grid->addWidget(&toolCoordCombo, 1, 1);
    grid->addWidget(new QPushButton(_("Edit")), 1, 2);

    auto label = new QLabel(_("Config"));
    grid->addWidget(label, 2, 0, Qt::AlignLeft);
    configurationWidgets.push_back(label);
    grid->addWidget(&configurationCombo, 2, 1);
    configurationWidgets.push_back(&configurationCombo);
    userInputConnections.add(
        configurationCombo.sigActivated().connect(
            [this](int index){ onConfigurationInput(index); }));

    requireConfigurationCheck.setText(_("Require"));
    grid->addWidget(&requireConfigurationCheck, 2, 2);
    configurationWidgets.push_back(&requireConfigurationCheck);
    userInputConnections.add(
        requireConfigurationCheck.sigToggled().connect(
            [this](bool){ onConfigurationInput(configurationCombo.currentIndex()); }));

    mainvbox->addLayout(grid);
    mainvbox->addStretch();

    menuManager.setNewPopupMenu(self);
    
    rpyCheck = menuManager.addCheckItem(_("Roll-pitch-yaw"));
    rpyCheck->setChecked(true);
    settingConnections.add(
        rpyCheck->sigToggled().connect(
            [&](bool on){ setRpySpinsVisible(on); }));

    uniqueRpyCheck = menuManager.addCheckItem(_("Fetch as a unique RPY value"));
    uniqueRpyCheck->setChecked(false);
    settingConnections.add(
        uniqueRpyCheck->sigToggled().connect(
            [&](bool on){ updatePanel(); }));
    
    quaternionCheck = menuManager.addCheckItem(_("Quoternion"));
    quaternionCheck->setChecked(false);
    setQuaternionSpinsVisible(false);
    settingConnections.add(
        quaternionCheck->sigToggled().connect(
            [&](bool on){ setQuaternionSpinsVisible(on); }));

    rotationMatrixCheck = menuManager.addCheckItem(_("Rotation matrix"));
    rotationMatrixCheck->setChecked(false);
    rotationMatrixPanel.setVisible(false);
    settingConnections.add(
        rotationMatrixCheck->sigToggled().connect(
            [&](bool on){
                rotationMatrixPanel.setVisible(on);
                updatePanel(); }));

    disableCustomIKCheck = menuManager.addCheckItem(_("Disable custom IK"));
    disableCustomIKCheck->setChecked(false);
    settingConnections.add(
        disableCustomIKCheck->sigToggled().connect(
            [&](bool){
                updateTargetLink(targetLink);
                updatePanel();
            }));

    lastInputAttitudeType = ROLL_PITCH_YAW;
}


void LinkPositionView::onActivated()
{
    auto bb = BodyBar::instance();

    impl->bodyBarConnection = 
        bb->sigCurrentBodyItemChanged().connect(
            [&](BodyItem* bodyItem){ impl->setBodyItem(bodyItem); });

    impl->setBodyItem(bb->currentBodyItem());
}


void LinkPositionView::onDeactivated()
{
    impl->bodyBarConnection.disconnect();
    impl->bodyItemConnections.disconnect();
    impl->targetBodyItem.reset();
    impl->inverseKinematics.reset();
    impl->jointPathConfigurationHandler.reset();
}


void LinkPositionViewImpl::resetInputWidgetStyles()
{
    for(auto& widget : inputElementWidgets){
        widget->setStyleSheet(normalStyle);
    }
}
    

void LinkPositionViewImpl::onMenuButtonClicked()
{
    menuManager.popupMenu()->popup(menuButton.mapToGlobal(QPoint(0,0)));
}


void LinkPositionViewImpl::setRpySpinsVisible(bool on)
{
    for(auto& widget : rpyWidgets){
        widget->setVisible(on);
    }
}


void LinkPositionViewImpl::setQuaternionSpinsVisible(bool on)
{
    for(auto& widget : quatWidgets){
        widget->setVisible(on);
    }
}


void LinkPositionViewImpl::setBodyItem(BodyItem* bodyItem)
{
    if(bodyItem != targetBodyItem){

        resetInputWidgetStyles();
        bodyItemConnections.disconnect();
    
        if(bodyItem){
            bodyItemConnections.add(
                bodyItem->sigNameChanged().connect(
                    [&](const std::string&){ updateTargetLink(targetLink); }));

            bodyItemConnections.add(
                linkSelectionView->sigSelectionChanged(bodyItem).connect(
                    [&](){ updateTargetLink(); updatePanel(); }));

            bodyItemConnections.add(
                bodyItem->sigKinematicStateChanged().connect(
                    [&](){ updatePanel(); }));
        }
        
        targetBodyItem = bodyItem;
        updateTargetLink();
        updatePanel();
    }
}


void LinkPositionViewImpl::updateTargetLink(Link* link)
{
    targetLink = link;
    inverseKinematics.reset();
    jointPathConfigurationHandler.reset();
    
    if(!targetBodyItem){
        targetLabel.setText("------");
        targetLink = nullptr;

    } else {
        auto body = targetBodyItem->body();

        if(!targetLink){
            auto selectedLinkIndex = linkSelectionView->selectedLinkIndex(targetBodyItem);
            if(selectedLinkIndex >= 0){
                targetLink = body->link(selectedLinkIndex);
            } else {
                targetLink = findUniqueEndLink(body);
                if(!targetLink){
                    targetLink = body->rootLink();
                }
            }
        }
        
        targetLabel.setText(format("{0} / {1}", body->name(), targetLink->name()).c_str());

        inverseKinematics = targetBodyItem->getCurrentIK(targetLink);
        if(inverseKinematics){
            if(auto compositeBodyIK = dynamic_pointer_cast<CompositeBodyIK>(inverseKinematics)){
                jointPathConfigurationHandler =
                    dynamic_pointer_cast<JointPathConfigurationHandler>(
                        compositeBodyIK->getParentBodyIK());
            } else {
                jointPathConfigurationHandler =
                    dynamic_pointer_cast<JointPathConfigurationHandler>(inverseKinematics);
            }
            if(disableCustomIKCheck->isChecked()){
                if(auto jointPath = dynamic_pointer_cast<JointPath>(inverseKinematics)){
                    // Use the non-customized, numerical IK
                    jointPath->setNumericalIKenabled(true);
                }
            }
        }
    }

    self->setEnabled(inverseKinematics != nullptr);
    resultLabel.setText("");

    configurationCombo.clear();
    bool isConfigurationInputActive = (jointPathConfigurationHandler != nullptr) && !disableCustomIKCheck->isChecked();
    for(auto& widget : configurationWidgets){
        widget->setEnabled(isConfigurationInputActive);
    }
    
    if(jointPathConfigurationHandler){
        int n = jointPathConfigurationHandler->getNumConfigurations();
        for(int i=0; i < n; ++i){
            configurationCombo.addItem(
                jointPathConfigurationHandler->getConfigurationName(i).c_str());
        }
        if(jointPathConfigurationHandler->checkConfiguration(0)){
            configurationCombo.setCurrentIndex(0);
        } else {
            configurationCombo.setCurrentIndex(
                jointPathConfigurationHandler->getCurrentConfiguration());
        }
    }
}


Link* LinkPositionViewImpl::findUniqueEndLink(Body* body) const
{
    Link* endLink = nullptr;
    Link* link = body->rootLink();
    while(true){
        if(!link->child()){
            endLink = link;
            break;
        }
        if(link->child()->sibling()){
            break;
        }
        link = link->child();
    }
    return endLink;
}


void LinkPositionViewImpl::updatePanel()
{
    if(!inverseKinematics){
        self->setEnabled(false);
        resultLabel.setText("");

    } else {
        self->setEnabled(true);
        
        userInputConnections.block();

        Vector3 p = targetLink->p();
        for(int i=0; i < 3; ++i){
            auto& spin = xyzSpin[i];
            if(!spin.hasFocus()){
                spin.setValue(p[i]);
            }
        }
        Matrix3 R = targetLink->attitude();
        if(rpyCheck->isChecked()){
            Vector3 prevRPY;
            for(int i=0; i < 3; ++i){
                prevRPY[i] = radian(rpySpin[i].value());
            }
            Vector3 rpy;
            if(uniqueRpyCheck->isChecked()){
                rpy = rpyFromRot(R);
            } else {
                rpy = rpyFromRot(R, prevRPY);
            }
            for(int i=0; i < 3; ++i){
                rpySpin[i].setValue(degree(rpy[i]));
            }
        }
        if(quaternionCheck->isChecked()){
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
        if(rotationMatrixCheck->isChecked()){
            updateRotationMatrixPanel(R);
        }

        resetInputWidgetStyles();

        updateConfigurationPanel();

        userInputConnections.unblock();

        resultLabel.setText(_("Actual State"));
        resultLabel.setStyleSheet(normalStyle);
    }
}


void LinkPositionViewImpl::updateRotationMatrixPanel(const Matrix3& R)
{
    for(int i=0; i < 3; ++i){
        for(int j=0; j < 3; ++j){
            rotationMatrixElementLabel[i][j].setText(
                format("{: .6f}", R(i, j)).c_str());
        }
    }
}


void LinkPositionViewImpl::updateConfigurationPanel()
{
    if(!jointPathConfigurationHandler){
        configurationLabel.setText("");
    } else {
        int preferred = configurationCombo.currentIndex();
        if(requireConfigurationCheck.isChecked() &&
           !jointPathConfigurationHandler->checkConfiguration(preferred)){
            configurationCombo.setStyleSheet(errorStyle);
        } else {
            configurationCombo.setStyleSheet("font-weight: normal");
        }
        int actual = jointPathConfigurationHandler->getCurrentConfiguration();
        configurationLabel.setText(QString("( %1 )").arg(configurationCombo.itemText(actual)));
    }
}


void LinkPositionViewImpl::onPositionInput(InputElementSet inputElements)
{
    if(lastInputAttitudeType == ROLL_PITCH_YAW && rpyCheck->isChecked()){
        onPositionInputRpy(inputElements);
    } else if(lastInputAttitudeType == QUATERNION && quaternionCheck->isChecked()){
        onPositionInputQuaternion(inputElements);
    } else if(quaternionCheck->isChecked()){
        onPositionInputQuaternion(inputElements);
    } else {
        onPositionInputRpy(inputElements);
    }
}


void LinkPositionViewImpl::onPositionInputRpy(InputElementSet inputElements)
{
    Position T;
    Vector3 rpy;

    for(int i=0; i < 3; ++i){
        T.translation()[i] = xyzSpin[i].value();
        rpy[i] = radian(rpySpin[i].value());
    }
    T.linear() = rotFromRpy(rpy);
    
    findSolution(T, inputElements);

    lastInputAttitudeType = ROLL_PITCH_YAW;
}


void LinkPositionViewImpl::onPositionInputQuaternion(InputElementSet inputElements)
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
        findSolution(T, inputElements);
    }

    lastInputAttitudeType = QUATERNION;
}


void LinkPositionViewImpl::onConfigurationInput(int index)
{
    if(jointPathConfigurationHandler){
        jointPathConfigurationHandler->setPreferredConfiguration(index);
    }
    onPositionInput(InputElementSet(0));
}


void LinkPositionViewImpl::findSolution(const Position& T_input, InputElementSet inputElements)
{
    if(inverseKinematics){

        Position T;
        T.translation() = T_input.translation();
        T.linear() = targetLink->calcRfromAttitude(T_input.linear());
        targetBodyItem->beginKinematicStateEdit();
        bool solved = inverseKinematics->calcInverseKinematics(T);

        if(!solved){
            for(size_t i=0; i < inputElementWidgets.size(); ++i){
                if(inputElements[i]){
                    inputElementWidgets[i]->setStyleSheet(errorStyle);
                }
            }
        } else {
            if(jointPathConfigurationHandler && requireConfigurationCheck.isChecked() &&
               !disableCustomIKCheck->isChecked()){
                int preferred = configurationCombo.currentIndex();
                if(!jointPathConfigurationHandler->checkConfiguration(preferred)){
                    configurationCombo.setStyleSheet(errorStyle);
                    solved = false;
                }
            }
        }

        if(solved){       
            targetBodyItem->notifyKinematicStateChange();
            targetBodyItem->acceptKinematicStateEdit();
            resultLabel.setText(_("Solved"));
            resultLabel.setStyleSheet(normalStyle);
        } else {
            targetBodyItem->cancelKinematicStateEdit();
            resultLabel.setText(_("Not Solved"));
            resultLabel.setStyleSheet("font-weight: bold; color: red");
        }
    }
}


bool LinkPositionView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool LinkPositionViewImpl::storeState(Archive& archive)
{
    archive.write("coordinateMode", coordinateMode.selectedSymbol());
    archive.write("showRPY", rpyCheck->isChecked());
    archive.write("uniqueRPY", uniqueRpyCheck->isChecked());
    archive.write("showQuoternion", quaternionCheck->isChecked());
    archive.write("showRotationMatrix", rotationMatrixCheck->isChecked());
    archive.write("disableCustomIK", disableCustomIKCheck->isChecked());
    archive.write("configuration", configurationCombo.currentText().toStdString());
    archive.write("isConfigurationRequired", requireConfigurationCheck.isChecked());
    return true;
}


bool LinkPositionView::restoreState(const Archive& archive)
{
    archive.addPostProcess([&](){ impl->restoreState(archive); });
    return true;
}


bool LinkPositionViewImpl::restoreState(const Archive& archive)
{
    settingConnections.block();
    userInputConnections.block();
    
    string symbol;
    if(archive.read("coordinateMode", symbol)){
        if(coordinateMode.select(symbol)){
            coordinateModeGroup.button(coordinateMode.which())->setChecked(true);
        }
    }
    rpyCheck->setChecked(archive.get("showRPY", rpyCheck->isChecked()));
    uniqueRpyCheck->setChecked(archive.get("uniqueRPY", uniqueRpyCheck->isChecked()));
    quaternionCheck->setChecked(archive.get("showQuoternion", quaternionCheck->isChecked()));
    rotationMatrixCheck->setChecked(archive.get("showRotationMatrix", rotationMatrixCheck->isChecked()));
    disableCustomIKCheck->setChecked(archive.get("disableCustomIK", disableCustomIKCheck->isChecked()));

    if(archive.read("configuration", symbol)){
        configurationCombo.setCurrentText(symbol.c_str());
    }

    userInputConnections.unblock();
    settingConnections.unblock();

    updateTargetLink();
    onConfigurationInput(configurationCombo.currentIndex());
    
    return true;
}
