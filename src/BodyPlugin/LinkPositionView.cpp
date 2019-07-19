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
#include <QLabel>
#include <QGridLayout>
#include <QGroupBox>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class LinkPositionViewImpl
{
public:
    LinkPositionView* self;
            
    BodyItemPtr targetBodyItem;
    Link* targetLink;
    shared_ptr<JointPath> jointPath;
    shared_ptr<JointPathConfigurationHandler> jointPathConfigurationHandler;

    LinkSelectionView* linkSelectionView;

    ToolButton menuButton;
    MenuManager menuManager;
    QLabel targetLabel;
    QLabel resultLabel;
    DoubleSpinBox xyzSpin[3];
    Action* rpyCheck;
    DoubleSpinBox rpySpin[3];
    vector<QWidget*> rpyWidgets;
    Action* quaternionCheck;
    DoubleSpinBox quatSpin[4];
    vector<QWidget*> quatWidgets;
    Action* rotationMatrixCheck;
    QWidget rotationMatrixPanel;
    QLabel rotationMatrixElementLabel[3][3];

    enum AttitudeType { ROLL_PITCH_YAW, QUATERNION };
    AttitudeType lastInputAttitudeType;

    ComboBox userCoordCombo;
    ComboBox toolCoordCombo;
    ComboBox configurationCombo;
    CheckBox forceConfigurationCheck;
    vector<QWidget*> configurationWidgets;

    ScopedConnection bodyBarConnection;
    ScopedConnectionSet bodyItemConnections;
    ScopedConnectionSet userInputConnections;

    LinkPositionViewImpl(LinkPositionView* self);
    ~LinkPositionViewImpl();
    void createPanel();
    void onMenuButtonClicked();
    void setRpySpinsVisible(bool on);
    void setQuaternionSpinsVisible(bool on);
    void setBodyItem(BodyItem* bodyItem);
    void updateTarget();
    void updatePanel();
    Link* findUniqueEndLink(Body* body) const;
    void updateRotationMatrixPanel(const Matrix3& R);
    void updateConfigurationPanel();
    void onPositionInput();
    void onPositionInputRpy();
    void onPositionInputQuaternion();
    void onConfigurationInput(int index);
    void doInverseKinematics(const Position& T_input);
};

}


void LinkPositionView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LinkPositionView>(
        "LinkPositionView", N_("Link Position"), ViewManager::SINGLE_DEFAULT);
}


LinkPositionView::LinkPositionView()
{
    impl = new LinkPositionViewImpl(this);
}


LinkPositionViewImpl::LinkPositionViewImpl(LinkPositionView* self)
    : self(self)
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
    const int margin = 8;
    
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    
    auto topvbox = new QVBoxLayout;
    self->setLayout(topvbox);
    
    auto mainvbox = new QVBoxLayout;
    mainvbox->setContentsMargins(margin, margin, margin, margin);
    topvbox->addLayout(mainvbox);

    auto hbox = new QHBoxLayout;
    targetLabel.setStyleSheet("font-weight: bold");
    targetLabel.setAlignment(Qt::AlignCenter);
    hbox->addWidget(&targetLabel, 1);
    
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
    applyButton->sigClicked().connect([&](){ onPositionInput(); });
    hbox->addWidget(applyButton);
    mainvbox->addLayout(hbox);

    auto grid = new QGridLayout;
    static const char* xyzLabels[] = { "X", "Y", "Z" };
    static const char* rpyLabelChar[] = {"RX", "RY", "RZ"};

    for(int i=0; i < 3; ++i){
        // Translation spin boxes
        xyzSpin[i].setAlignment(Qt::AlignCenter);
        xyzSpin[i].setDecimals(4);
        xyzSpin[i].setRange(-99.9999, 99.9999);
        xyzSpin[i].setSingleStep(0.0001);
        
        userInputConnections.add(
            xyzSpin[i].sigValueChanged().connect(
                [&](double){ onPositionInput(); }));

        grid->addWidget(new QLabel(xyzLabels[i]), 0, i * 2, Qt::AlignRight);
        grid->addWidget(&xyzSpin[i], 0, i * 2 + 1);
        
        // Roll-pitch-yaw spin boxes
        rpySpin[i].setAlignment(Qt::AlignCenter);
        rpySpin[i].setDecimals(1);
        rpySpin[i].setRange(-360.0, 360.0);
        rpySpin[i].setSingleStep(0.1);

        userInputConnections.add(
            rpySpin[i].sigValueChanged().connect(
                [&](double){ onPositionInputRpy(); }));

        auto label = new QLabel(rpyLabelChar[i]);
        grid->addWidget(label, 1, i * 2, Qt::AlignRight);
        rpyWidgets.push_back(label);
        grid->addWidget(&rpySpin[i], 1, i * 2 + 1);
        rpyWidgets.push_back(&rpySpin[i]);

        grid->setColumnStretch(i * 2, 1);
        grid->setColumnStretch(i * 2 + 1, 10);
    }
    mainvbox->addLayout(grid);

    grid = new QGridLayout;
    static const char* quatLabelChar[] = {"QX", "QY", "QZ", "QW"};

    for(int i=0; i < 4; ++i){
        quatSpin[i].setAlignment(Qt::AlignCenter);
        quatSpin[i].setDecimals(4);
        quatSpin[i].setRange(-1.0000, 1.0000);
        quatSpin[i].setSingleStep(0.0001);

        userInputConnections.add(
            quatSpin[i].sigValueChanged().connect(
                [&](double){ onPositionInputQuaternion(); }));
        
        auto label = new QLabel(quatLabelChar[i]);
        grid->addWidget(label, 0, i * 2, Qt::AlignRight);
        quatWidgets.push_back(label);
        grid->addWidget(&quatSpin[i], 0, i * 2 + 1);
        quatWidgets.push_back(&quatSpin[i]);

        grid->setColumnStretch(i * 2, 1);
        grid->setColumnStretch(i * 2 + 1, 10);
    }
    mainvbox->addLayout(grid);

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

    grid->addWidget(new QLabel(_("Local")), 1, 0, Qt::AlignLeft);
    toolCoordCombo.addItem(_("Link Origin"));
    grid->addWidget(&toolCoordCombo, 1, 1);
    grid->addWidget(new QPushButton(_("Edit")), 1, 2);

    auto label = new QLabel(_("Config"));
    grid->addWidget(label, 2, 0, Qt::AlignLeft);
    configurationWidgets.push_back(label);
    grid->addWidget(&configurationCombo, 2, 1);
    configurationWidgets.push_back(&configurationCombo);
    forceConfigurationCheck.setText(_("Forced"));
    grid->addWidget(&forceConfigurationCheck, 2, 2);
    configurationWidgets.push_back(&forceConfigurationCheck);
    userInputConnections.add(
        configurationCombo.sigActivated().connect(
            [this](int index){ onConfigurationInput(index); }));

    mainvbox->addLayout(grid);
    mainvbox->addStretch();

    menuManager.setNewPopupMenu(self);
    
    rpyCheck = menuManager.addCheckItem(_("Roll-pitch-yaw"));
    rpyCheck->setChecked(true);
    rpyCheck->sigToggled().connect(
        [&](bool on){ setRpySpinsVisible(on); });
    
    quaternionCheck = menuManager.addCheckItem(_("Quoternion"));
    quaternionCheck->setChecked(false);
    setQuaternionSpinsVisible(false);
    quaternionCheck->sigToggled().connect(
        [&](bool on){ setQuaternionSpinsVisible(on); });

    rotationMatrixCheck = menuManager.addCheckItem(_("Rotation matrix"));
    rotationMatrixCheck->setChecked(false);
    rotationMatrixPanel.setVisible(false);
    rotationMatrixCheck->sigToggled().connect(
        [&](bool on){ rotationMatrixPanel.setVisible(on); });

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
    impl->jointPath.reset();
    impl->jointPathConfigurationHandler.reset();
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

        bodyItemConnections.disconnect();
    
        if(bodyItem){
            bodyItemConnections.add(
                bodyItem->sigNameChanged().connect(
                    [&](const std::string&){ updateTarget(); }));

            bodyItemConnections.add(
                linkSelectionView->sigSelectionChanged(bodyItem).connect(
                    [&](){ updateTarget(); updatePanel(); }));

            bodyItemConnections.add(
                bodyItem->sigKinematicStateChanged().connect(
                    [&](){ updatePanel(); }));
        }
        
        targetBodyItem = bodyItem;
        updateTarget();
        updatePanel();
    }
}


void LinkPositionViewImpl::updateTarget()
{
    targetLink = nullptr;
    jointPath = nullptr;
    jointPathConfigurationHandler = nullptr;
    
    if(!targetBodyItem){
        targetLabel.setText("------");

    } else {
        auto body = targetBodyItem->body();
        
        auto selectedLinkIndex = linkSelectionView->selectedLinkIndex(targetBodyItem);
        if(selectedLinkIndex >= 0){
            targetLink = body->link(selectedLinkIndex);
        } else {
            targetLink = findUniqueEndLink(body);
            if(!targetLink){
                targetLink = body->rootLink();
            }
        }
        auto baseLink = targetBodyItem->currentBaseLink();
        if(!baseLink){
            baseLink = body->rootLink();
        }
        
        targetLabel.setText(format("{0} / {1}", body->name(), targetLink->name()).c_str());

        jointPath = getCustomJointPath(body, baseLink, targetLink);
        if(jointPath){
            jointPathConfigurationHandler = dynamic_pointer_cast<JointPathConfigurationHandler>(jointPath);
        } else {
            jointPath = make_shared<JointPath>(baseLink, targetLink);
        }
    }

    self->setEnabled(jointPath != nullptr);
    resultLabel.setText("");

    configurationCombo.clear();
    for(auto& widget : configurationWidgets){
        widget->setEnabled(jointPathConfigurationHandler != nullptr);
    }
    
    if(jointPathConfigurationHandler){
        int n = jointPathConfigurationHandler->getNumConfigurations();
        for(int i=0; i < n; ++i){
            configurationCombo.addItem(
                jointPathConfigurationHandler->getConfigurationName(i).c_str());
        }
        configurationCombo.setCurrentIndex(
            jointPathConfigurationHandler->getCurrentConfiguration());
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
    if(!jointPath){
        self->setEnabled(false);
        resultLabel.setText("");

    } else {
        self->setEnabled(true);
        
        userInputConnections.block();

        auto endLink = jointPath->endLink();

        Vector3 p = endLink->p();
        for(int i=0; i < 3; ++i){
            auto& spin = xyzSpin[i];
            if(!spin.hasFocus()){
                spin.setValue(p[i]);
            }
        }
        Matrix3 R = endLink->attitude();
        if(rpyCheck->isChecked()){
            Vector3 rpy = rpyFromRot(R);
            for(int i=0; i < 3; ++i){
                auto& spin = rpySpin[i];
                if(!spin.hasFocus()){
                    spin.setValue(degree(rpy[i]));
                }
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

        updateConfigurationPanel();

        userInputConnections.unblock();

        resultLabel.setText(_("Consistent with display"));
        resultLabel.setStyleSheet("");
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
    if(jointPathConfigurationHandler){
        auto index = jointPathConfigurationHandler->getCurrentConfiguration();
        userInputConnections.block();
        configurationCombo.setCurrentIndex(index);
        userInputConnections.unblock();
    }
}


void LinkPositionViewImpl::onPositionInput()
{
    if(lastInputAttitudeType == ROLL_PITCH_YAW && rpyCheck->isChecked()){
        onPositionInputRpy();
    } else if(lastInputAttitudeType == QUATERNION && quaternionCheck->isChecked()){
        onPositionInputQuaternion();
    } else if(quaternionCheck->isChecked()){
        onPositionInputQuaternion();
    } else {
        onPositionInputRpy();
    }
}


void LinkPositionViewImpl::onPositionInputRpy()
{
    Position T;
    Vector3 rpy;

    for(int i=0; i < 3; ++i){
        T.translation()[i] = xyzSpin[i].value();
        rpy[i] = radian(rpySpin[i].value());
    }
    T.linear() = rotFromRpy(rpy);
    
    doInverseKinematics(T);

    lastInputAttitudeType = ROLL_PITCH_YAW;
}


void LinkPositionViewImpl::onPositionInputQuaternion()
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
        doInverseKinematics(T);
    }

    lastInputAttitudeType = QUATERNION;
}


void LinkPositionViewImpl::onConfigurationInput(int index)
{
    if(jointPathConfigurationHandler){
        jointPathConfigurationHandler->setPreferredConfiguration(index);
    }
    onPositionInput();
}


void LinkPositionViewImpl::doInverseKinematics(const Position& T_input)
{
    if(jointPath){
        Position T;
        T.translation() = T_input.translation();
        T.linear() = jointPath->endLink()->calcRfromAttitude(T_input.linear());
        
        targetBodyItem->beginKinematicStateEdit();
        bool solved = jointPath->calcInverseKinematics(T);

        if(solved){
            if(jointPathConfigurationHandler && forceConfigurationCheck.isChecked()){
                int conf = jointPathConfigurationHandler->getCurrentConfiguration();
                if(conf != configurationCombo.currentIndex()){
                    solved = false;
                }
            }
        }
        if(solved){       
            targetBodyItem->notifyKinematicStateChange(true);
            targetBodyItem->acceptKinematicStateEdit();
            resultLabel.setText(_("Solved"));
            resultLabel.setStyleSheet("");
        } else {
            targetBodyItem->cancelKinematicStateEdit();
            resultLabel.setText(_("Not Solved"));
            resultLabel.setStyleSheet("font-weight: bold; color: red");
        }
        updateConfigurationPanel();
    }
}


bool LinkPositionView::storeState(Archive& archive)
{
    return true;
}


bool LinkPositionView::restoreState(const Archive& archive)
{
    return true;
}
