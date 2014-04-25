/** \file
    \author Shin'ichiro Nakaoka
*/

#include "BodyLinkView.h"
#include "BodyBar.h"
#include "BodyItem.h"
#include "LinkSelectionView.h"
#include "WorldItem.h"
#include "KinematicsBar.h"
#include <cnoid/EigenUtil>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/PinDragIK>
#include <cnoid/PenetrationBlocker>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include <cnoid/Slider>
#include <cnoid/Separator>
#include <cnoid/LazyCaller>
#include <cnoid/ViewManager>
#include <QScrollArea>
#include <QGridLayout>
#include <QGroupBox>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
static const double sliderResolution = 1000000.0;
}

namespace cnoid {

class BodyLinkViewImpl : public boost::signals::trackable
{
public:
    BodyLinkViewImpl(BodyLinkView* self);
    ~BodyLinkViewImpl();
            
    BodyLinkView* self;
            
    QScrollArea scrollArea;

    QLabel nameLabel;
    QLabel linkIndexLabel;
    QLabel jointIdLabel;
    QLabel jointTypeLabel;
    QLabel jointAxisLabel;

    QGroupBox qGroup;
    DoubleSpinBox qSpin;
    QLabel qMinLabel;
    QLabel qMaxLabel;
    Slider qSlider;

    QGroupBox dqGroup;
    QLabel dqLabel;
    DoubleSpinBox dqMinSpin;
    DoubleSpinBox dqMaxSpin;

    DoubleSpinBox xyzSpin[3];
    DoubleSpinBox rpySpin[3];
    CheckBox attMatrixCheck;
    QWidget attMatrixBox;
    QLabel attLabels[3][3];

    DoubleSpinBox zmpXyzSpin[3];

    QString selfCollisionString;
    QLabel selfCollisionsLabel;
    QString worldCollisionString;
    QLabel worldCollisionsLabel;

    WorldItem* currentWorldItem;
    BodyItemPtr currentBodyItem;
    Link* currentLink;

    ConnectionSet bodyItemConnections;
    LazyCaller updateKinematicStateLater;
    ConnectionSet propertyWidgetConnections;
    ConnectionSet stateWidgetConnections;

    void setupWidgets();
    void onAttMatrixCheckToggled();
    void onCurrentBodyItemChanged(BodyItem* bodyItem);
    void activateCurrentBodyItem(bool on);
    void update();
    void updateLink();
    void updateKinematicState(bool blockSignals);
    void updateCollisions();
    void addSelfCollision(const CollisionLinkPair& collisionPair, QString& collisionString);
    void addWorldCollision(const CollisionLinkPair& collisionPair, QString& collisionString);
    void on_qSpinChanged(double value);
    void on_qSliderChanged(int value);
    void on_qChanged(double q);
    void on_dqLimitChanged(bool isMin);
    void onXyzChanged();
    void onRpyChanged();
    void doInverseKinematics(Vector3 p, Matrix3 R);
    void onZmpXyzChanged();
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};
}


namespace {
// margins
const int M1 = 2;
const int M2 = 4;
const int M3 = 8;
const int M4 = 16;
}


void BodyLinkView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<BodyLinkView>(
        "BodyLinkView", N_("Body / Link"), ViewManager::SINGLE_DEFAULT);
}


BodyLinkView::BodyLinkView()
{
    impl = new BodyLinkViewImpl(this);
}


BodyLinkViewImpl::BodyLinkViewImpl(BodyLinkView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::CENTER);

    currentWorldItem = 0;
    currentBodyItem = 0;
    currentLink = 0;

    setupWidgets();

    updateKinematicStateLater.setFunction(bind(&BodyLinkViewImpl::updateKinematicState, this, true));
    updateKinematicStateLater.setPriority(LazyCaller::PRIORITY_LOW);

    BodyBar::instance()->sigCurrentBodyItemChanged().connect(
        bind(&BodyLinkViewImpl::onCurrentBodyItemChanged, this, _1));
    
    self->sigActivated().connect(bind(&BodyLinkViewImpl::activateCurrentBodyItem, this, true));
    self->sigDeactivated().connect(bind(&BodyLinkViewImpl::activateCurrentBodyItem, this, false));
}


BodyLinkView::~BodyLinkView()
{
    delete impl;
}


BodyLinkViewImpl::~BodyLinkViewImpl()
{
    bodyItemConnections.disconnect();
}


void BodyLinkViewImpl::setupWidgets()
{
    QHBoxLayout* hbox;
    QVBoxLayout* vbox;
    QGridLayout* grid;

    QWidget* topWidget = new QWidget();
    topWidget->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    
    QVBoxLayout* topVBox = new QVBoxLayout();
    //topVBox->setContentsMargins(4);
    topWidget->setLayout(topVBox);

    scrollArea.setFrameShape(QFrame::NoFrame);
    scrollArea.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea.setWidget(topWidget);
    QVBoxLayout* baseLayout = new QVBoxLayout();
    scrollArea.setWidgetResizable(true);
    baseLayout->addWidget(&scrollArea);
    self->setLayout(baseLayout);

    //nameLabel.setAlignment(Qt::AlignCenter);
    topVBox->addWidget(&nameLabel, 0, Qt::AlignCenter);
    topVBox->addSpacing(4);

    QFrame* frame = new QFrame();
    topVBox->addWidget(frame);

    grid = new QGridLayout();
    //grid->setContentsMargins(4);
    grid->setVerticalSpacing(4);
    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(3, 1);
    grid->addWidget(new QLabel(_("Index:")), 0, 0);
    grid->addWidget(&linkIndexLabel, 0, 1);
    grid->addWidget(new QLabel(_("Joint ID:")), 0, 2);
    grid->addWidget(&jointIdLabel, 0, 3);
    grid->addWidget(new QLabel(_("Joint Type:")), 1, 0);
    grid->addWidget(&jointTypeLabel, 1, 1, 1, 3);
    grid->addWidget(new QLabel(_("Joint Axis:")), 2, 0);
    grid->addWidget(&jointAxisLabel, 2, 1, 1, 3);
    frame->setLayout(grid);

    topVBox->addSpacing(4);

    qGroup.setAlignment(Qt::AlignHCenter);
    topVBox->addWidget(&qGroup);
    
    vbox = new QVBoxLayout();
    //vbox->setContentsMargins(4);
    qGroup.setLayout(vbox);
    hbox = new QHBoxLayout();
    vbox->addLayout(hbox);
    hbox->addStretch();
    hbox->addWidget(&qMinLabel);
    qSpin.setAlignment(Qt::AlignCenter);
    hbox->addWidget(&qSpin);
    hbox->addWidget(&qMaxLabel);
    hbox->addStretch();

    qSlider.setOrientation(Qt::Horizontal);
    vbox->addWidget(&qSlider);

    stateWidgetConnections.add(
        qSpin.sigValueChanged().connect(
            bind(&BodyLinkViewImpl::on_qSpinChanged, this, _1)));
    
    stateWidgetConnections.add(
        qSlider.sigValueChanged().connect(
            bind(&BodyLinkViewImpl::on_qSliderChanged, this, _1)));

    topVBox->addSpacing(4);
    
    dqGroup.setAlignment(Qt::AlignHCenter);
    topVBox->addWidget(&dqGroup);
    hbox = new QHBoxLayout();
    //hbox->setContentsMargins(4);

    // min velocity spin
    hbox->addStretch();
    hbox->addWidget(new QLabel(_("min")));
    dqMaxSpin.setAlignment(Qt::AlignCenter);
    hbox->addWidget(&dqMinSpin);

    // velocity label
    hbox->addWidget(&dqLabel);

    // max velocity spin
    dqMinSpin.setAlignment(Qt::AlignCenter);
    hbox->addWidget(&dqMaxSpin);
    hbox->addWidget(new QLabel(_("max")));
    hbox->addStretch();
    dqGroup.setLayout(hbox);
    
    propertyWidgetConnections.add(
        dqMinSpin.sigValueChanged().connect(
            bind(&BodyLinkViewImpl::on_dqLimitChanged, this, true)));
    propertyWidgetConnections.add(
        dqMaxSpin.sigValueChanged().connect(
            bind(&BodyLinkViewImpl::on_dqLimitChanged, this, false)));
    
    topVBox->addSpacing(4);

    frame = new QFrame();
    vbox = new QVBoxLayout();
    //vbox->setContentsMargins(4);
    vbox->setSpacing(4);
    frame->setLayout(vbox);
    topVBox->addWidget(frame);
    
    vbox->addWidget(new QLabel(_("Link Position [m],[deg]")));
    
    grid = new QGridLayout();
    //grid->setContentsMargins(4);
    grid->setVerticalSpacing(4);
    vbox->addLayout(grid);
   
    static const char* xyzLabels[] = {"X", "Y", "Z"};

    for(int i=0; i < 3; ++i){
        grid->addWidget(new QLabel(xyzLabels[i], frame), 0, i, Qt::AlignCenter);
        grid->addWidget(&xyzSpin[i], 1, i);

        //xyzSpin[i].set_width_chars(7);
        xyzSpin[i].setAlignment(Qt::AlignCenter);
        xyzSpin[i].setDecimals(4);
        xyzSpin[i].setRange(-99.9999, 99.9999);
        xyzSpin[i].setSingleStep(0.0001);
        
        stateWidgetConnections.add(
            xyzSpin[i].sigValueChanged().connect(
                bind(&BodyLinkViewImpl::onXyzChanged, this)));
    }

    static const char* rpyLabels[] = {"R", "P", "Y"};

    for(int i=0; i < 3; ++i){
        grid->addWidget(new QLabel(rpyLabels[i], frame), 2, i, Qt::AlignCenter);
        grid->addWidget(&rpySpin[i], 3, i);

        rpySpin[i].setAlignment(Qt::AlignCenter);
        rpySpin[i].setDecimals(1);
        rpySpin[i].setRange(-360.0, 360.0);
        rpySpin[i].setSingleStep(0.1);

        stateWidgetConnections.add(
            rpySpin[i].sigValueChanged().connect(
                bind(&BodyLinkViewImpl::onRpyChanged, this)));
    }

    attMatrixCheck.setText(_("Matrix"));
    attMatrixCheck.sigToggled().connect(
        bind(&BodyLinkViewImpl::onAttMatrixCheckToggled, this));
    vbox->addWidget(&attMatrixCheck, 0, Qt::AlignCenter);

    grid = new QGridLayout();
    grid->setHorizontalSpacing(10);
    //grid->setContentsMargins(4);
    grid->setVerticalSpacing(4);

    hbox = new QHBoxLayout();
    attMatrixBox.setLayout(hbox);
    vbox->addWidget(&attMatrixBox);

    hbox->addStretch();
    hbox->addWidget(new QLabel("R = "));
    hbox->addWidget(new VSeparator());
    hbox->addLayout(grid);
    hbox->addWidget(new VSeparator());
    hbox->addStretch();
    
    for(int i=0; i < 3; ++i){
        for(int j=0; j < 3; ++j){
            grid->addWidget(&attLabels[i][j], i, j);
            attLabels[i][j].setText("0.0");
        }
    }

    topVBox->addSpacing(4);

    QGroupBox* group = new QGroupBox();
    group->setTitle(_("ZMP [m]"));
    group->setAlignment(Qt::AlignCenter);
    
    grid = new QGridLayout();
    //grid->setContentsMargins(4);
    grid->setVerticalSpacing(2);
    group->setLayout(grid);
   
    for(int i=0; i < 3; ++i){
        grid->addWidget(new QLabel(xyzLabels[i], group), 0, i, Qt::AlignCenter);
        grid->addWidget(&zmpXyzSpin[i], 1, i);

        //zmpXyzSpin[i].set_width_chars(7);
        zmpXyzSpin[i].setAlignment(Qt::AlignCenter);
        zmpXyzSpin[i].setDecimals(4);
        zmpXyzSpin[i].setRange(-99.9999, 99.9999);
        zmpXyzSpin[i].setSingleStep(0.0001);

        stateWidgetConnections.add(
            zmpXyzSpin[i].sigValueChanged().connect(
                bind(&BodyLinkViewImpl::onZmpXyzChanged, this)));
    }

    topVBox->addWidget(group);
    topVBox->addSpacing(4);

    group = new QGroupBox();
    group->setTitle(_("Collisions"));
    group->setAlignment(Qt::AlignCenter);

    vbox = new QVBoxLayout();
    //vbox->setContentsMargins(4);
    vbox->setSpacing(4);

    worldCollisionsLabel.setAlignment(Qt::AlignCenter);
    worldCollisionsLabel.setWordWrap(true);
    vbox->addWidget(&worldCollisionsLabel);

    hbox = new QHBoxLayout();
    hbox->setSpacing(4);
    hbox->addWidget(new HSeparator());
    hbox->addWidget(new QLabel(_("Self-Collisions")));
    hbox->addWidget(new HSeparator());
    vbox->addLayout(hbox);
    
    selfCollisionsLabel.setAlignment(Qt::AlignCenter);
    selfCollisionsLabel.setWordWrap(true);
    vbox->addWidget(&selfCollisionsLabel);

    topVBox->addSpacing(4);

    group->setLayout(vbox);
    topVBox->addWidget(group);

    attMatrixBox.hide();
}


void BodyLinkViewImpl::onAttMatrixCheckToggled()
{
    if(attMatrixCheck.isChecked()){
        attMatrixBox.show();
        updateKinematicState(true);
    } else {
        attMatrixBox.hide();
    }
}


void BodyLinkViewImpl::onCurrentBodyItemChanged(BodyItem* bodyItem)
{
    if(bodyItem != currentBodyItem){

        activateCurrentBodyItem(false);
        
        currentBodyItem = bodyItem;
        currentLink = 0;
    
        activateCurrentBodyItem(true);
    }
}


void BodyLinkViewImpl::activateCurrentBodyItem(bool on)
{
    bodyItemConnections.disconnect();
    
    if(on){
        if(self->isActive() && currentBodyItem){

            bodyItemConnections.add(
                LinkSelectionView::mainInstance()->sigSelectionChanged(currentBodyItem).connect(
                    bind(&BodyLinkViewImpl::update, this)));

            bodyItemConnections.add(
                currentBodyItem->sigKinematicStateChanged().connect(updateKinematicStateLater));
            
            bodyItemConnections.add(
                currentBodyItem->sigUpdated().connect(bind(&BodyLinkViewImpl::update, this)));

            bodyItemConnections.add(
                currentBodyItem->sigCollisionsUpdated().connect(
                    bind(&BodyLinkViewImpl::updateCollisions, this)));
            
            update();
        }
    }
}


void BodyLinkViewImpl::update()
{
    currentLink = 0;
    
    if(!currentBodyItem){
        nameLabel.setText("");
        return;
    }

    propertyWidgetConnections.block();
    stateWidgetConnections.block();
    
    BodyPtr body = currentBodyItem->body();
    const vector<int>& selectedLinkIndices =
        LinkSelectionView::mainInstance()->getSelectedLinkIndices(currentBodyItem);

    if(selectedLinkIndices.empty()){
        currentLink = body->rootLink();
    } else {
        currentLink = body->link(selectedLinkIndices.front());
    }

    if(currentLink){
        nameLabel.setText(QString("%1 / %2").arg(body->name().c_str()).arg(currentLink->name().c_str()));
        updateLink();
    } else {
        nameLabel.setText(body->name().c_str());
    }

    updateKinematicState(false);
            
    updateCollisions();

    stateWidgetConnections.unblock();
    propertyWidgetConnections.unblock();
}


void BodyLinkViewImpl::updateLink()
{
    BodyPtr body = currentBodyItem->body();
    
    linkIndexLabel.setText(QString::number(currentLink->index()));
    jointIdLabel.setText(QString::number(currentLink->jointId()));

    Vector3 a(currentLink->Rs().transpose() * currentLink->a());
    jointAxisLabel.setText(QString("(%1 %2 %3)").arg(a[0], 0, 'f', 4).arg(a[1], 0, 'f', 4).arg(a[2], 0, 'f', 4));

    if(currentLink->isFreeJoint()){
        jointTypeLabel.setText(_("Free"));
        
    } else if(currentLink->isFixedJoint()){
        jointTypeLabel.setText(_("Fixed"));

    } else if(currentLink->jointType() == Link::CRAWLER_JOINT){
        jointTypeLabel.setText(_("Crawler"));

    } else if(currentLink->isRotationalJoint()){

        jointTypeLabel.setText(_("Rotation"));

        qGroup.setTitle(_("Joint Angle [deg]"));

        double qmin = degree(currentLink->q_lower());
        double qmax = degree(currentLink->q_upper());
    
        qSpin.setDecimals(1);
        //qSpin.setRange(qmin, qmax);
        qSpin.setRange(-360.0, 360.0); // Limit over values should be shown
        qSpin.setSingleStep(0.1);
        qMinLabel.setText(QString::number(qmin, 'f', 1));
        qMaxLabel.setText(QString::number(qmax, 'f', 1));

        qSlider.setRange(qmin * sliderResolution, qmax * sliderResolution);
        qSlider.setSingleStep(0.1 * sliderResolution);

        dqGroup.setTitle(_("Joint Velocity [deg/s]"));

        dqMinSpin.setDecimals(1);
        dqMinSpin.setRange(-9999.9, 0.0);
        dqMinSpin.setSingleStep(0.1);
        dqMinSpin.setValue(degree(currentLink->dq_lower()));

        dqMaxSpin.setDecimals(1);
        dqMaxSpin.setRange(0.0, 9999.9);
        dqMaxSpin.setSingleStep(0.1);
        dqMaxSpin.setValue(degree(currentLink->dq_upper()));
            
    } else if(currentLink->isSlideJoint()){
            
        jointTypeLabel.setText(_("Slide"));

        qGroup.setTitle(_("Joint Translation [m]:"));
        
        double qmin = currentLink->q_lower();
        double qmax = currentLink->q_upper();
        
        qSpin.setDecimals(4);
        qSpin.setRange(-999.9999, 999.9999);
        qSpin.setSingleStep(0.0001);
        qMinLabel.setText(QString::number(qmin, 'f', 3));
        qMaxLabel.setText(QString::number(qmax, 'f', 3));
            
        qSlider.setRange(qmin * sliderResolution, qmax * sliderResolution);
        qSlider.setSingleStep(0.001 * sliderResolution);
        
        dqGroup.setTitle(_("Joint Velocity [m/s]"));

        dqMinSpin.setDecimals(3);
        dqMinSpin.setRange(-999.999, 0.0);
        dqMinSpin.setSingleStep(0.001);
        dqMinSpin.setValue(currentLink->dq_lower());

        dqMaxSpin.setDecimals(3);
        dqMaxSpin.setRange(0.0, 999.999);
        dqMaxSpin.setSingleStep(0.001);
        dqMaxSpin.setValue(currentLink->dq_upper());
    }
}


void BodyLinkViewImpl::updateKinematicState(bool blockSignals)
{
    if(currentBodyItem){

        if(blockSignals){
            stateWidgetConnections.block();
        }
            
        if(currentLink){

            if(currentLink->isRotationalJoint()){
                double q = degree(currentLink->q());
                qSpin.setValue(q);
                qSlider.setValue(q * sliderResolution);
                dqLabel.setText(QString::number(degree(currentLink->dq()), 'f', 1));
                
            } else if(currentLink->isSlideJoint()){
                qSpin.setValue(currentLink->q());
                qSlider.setValue(currentLink->q() * sliderResolution);
                dqLabel.setText(QString::number(currentLink->dq(), 'f', 1));
            }

            const Matrix3 R = currentLink->attitude();
            const Vector3 rpy = rpyFromRot(R);
            for(int i=0; i < 3; ++i){
                xyzSpin[i].setValue(currentLink->p()[i]);
                rpySpin[i].setValue(degree(rpy[i]));
            }
            if(attMatrixCheck.isChecked()){
                for(int i=0; i < 3; ++i){
                    for(int j=0; j < 3; ++j){
                        attLabels[i][j].setText(QString::number(R(i,j), 'f', 6));
                    }
                }
            }
        }

        const Vector3& zmp = currentBodyItem->zmp();
        for(int i=0; i < 3; ++i){
            zmpXyzSpin[i].setValue(zmp[i]);
        }
        
        if(blockSignals){
            stateWidgetConnections.unblock();
        }
    }
}


void BodyLinkViewImpl::updateCollisions()
{
    selfCollisionString.clear();
    worldCollisionString.clear();

    if(currentLink){
        const std::vector<CollisionLinkPairPtr>& collisions =
            currentBodyItem->collisionsOfLink(currentLink->index());
        for(size_t i=0; i < collisions.size(); ++i){
            const CollisionLinkPair& collisionPair = *collisions[i];
            if(collisionPair.isSelfCollision()){
                addSelfCollision(collisionPair, selfCollisionString);
            } else {
                addWorldCollision(collisionPair, worldCollisionString);
            }
        }
    }
    
    selfCollisionsLabel.setText(selfCollisionString);
    worldCollisionsLabel.setText(worldCollisionString);
}


void BodyLinkViewImpl::addSelfCollision(const CollisionLinkPair& collisionPair, QString& collisionString)
{
    Link* oppositeLink;
    if(collisionPair.link[0] == currentLink){
        oppositeLink = collisionPair.link[1];
    } else {
        oppositeLink = collisionPair.link[0];
    }
    if(!collisionString.isEmpty()){
        collisionString += " ";
    }
    collisionString += oppositeLink->name().c_str();
}


void BodyLinkViewImpl::addWorldCollision(const CollisionLinkPair& collisionPair, QString& collisionString)
{
    int opposite = (collisionPair.link[0] == currentLink) ? 1 : 0;

    if(!collisionString.isEmpty()){
        collisionString += " ";
    }
    collisionString += collisionPair.body[opposite]->name().c_str();
    collisionString += " / ";
    collisionString += collisionPair.link[opposite]->name().c_str();
}


void BodyLinkViewImpl::on_qSpinChanged(double value)
{
    on_qChanged(value);
}


void BodyLinkViewImpl::on_qSliderChanged(int value)
{
    on_qChanged(value / sliderResolution);
}


void BodyLinkViewImpl::on_qChanged(double q)
{
    if(currentLink){
        if(currentLink->isRotationalJoint()){
            q = radian(q);
        }
        currentLink->q() = q;
        currentBodyItem->notifyKinematicStateChange(true);
    }
}


void BodyLinkViewImpl::on_dqLimitChanged(bool isMin)
{
    if(currentLink){
        
        DoubleSpinBox& targetSpin = isMin ? dqMinSpin : dqMaxSpin;
        double targetVal = isMin ? currentLink->dq_lower() : currentLink->dq_upper();
        double oppositeVal = isMin ? currentLink->dq_upper() : currentLink->dq_lower();
    
        double limit = targetSpin.value();
        if(currentLink->isRotationalJoint()){
            limit = radian(limit);
        }
        
        if(currentLink->dq_upper() == -currentLink->dq_lower()){
            oppositeVal = -limit;
        }
        targetVal = limit;

        currentBodyItem->notifyUpdate();
    }
}
    

void BodyLinkViewImpl::onXyzChanged()
{
    if(currentLink){
        Vector3 p;
        for(int i=0; i < 3; ++i){
            p[i] = xyzSpin[i].value();
        }
        doInverseKinematics(p, currentLink->R());
    }
}


void BodyLinkViewImpl::onRpyChanged()
{
    if(currentLink){
        Vector3 rpy;
        for(int i=0; i < 3; ++i){
            rpy[i] = radian(rpySpin[i].value());
        }
        doInverseKinematics(currentLink->p(), currentLink->calcRfromAttitude(rotFromRpy(rpy)));
    }
}


void BodyLinkViewImpl::doInverseKinematics(Vector3 p, Matrix3 R)
{
    InverseKinematicsPtr ik = currentBodyItem->getCurrentIK(currentLink);

    if(ik){
        currentBodyItem->beginKinematicStateEdit();

        if(KinematicsBar::instance()->isPenetrationBlockMode()){
            PenetrationBlockerPtr blocker = currentBodyItem->createPenetrationBlocker(currentLink, true);
            if(blocker){
                Position T;
                T.translation() = p;
                T.linear() = R;
                if(blocker->adjust(T, Vector3(p - currentLink->p()))){
                    p = T.translation();
                    R = T.linear();
                }
            }
        }
        if(ik->calcInverseKinematics(p, R)){
            currentBodyItem->notifyKinematicStateChange(true);
            currentBodyItem->acceptKinematicStateEdit();
        }
    }
}


void BodyLinkViewImpl::onZmpXyzChanged()
{
    if(currentBodyItem){
        Vector3 zmp;
        for(int i=0; i < 3; ++i){
            zmp[i] = zmpXyzSpin[i].value();
        }
        currentBodyItem->setZmp(zmp);
        currentBodyItem->notifyKinematicStateChange(false);
    }
}


bool BodyLinkView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool BodyLinkViewImpl::storeState(Archive& archive)
{
    archive.write("showRotationMatrix", attMatrixCheck.isChecked());
    return true;
}


bool BodyLinkView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool BodyLinkViewImpl::restoreState(const Archive& archive)
{
    attMatrixCheck.setChecked(archive.get("showRotationMatrix", attMatrixCheck.isChecked()));
    return true;
}
