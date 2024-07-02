#include "LinkMassSummaryView.h"
#include "BodySelectionManager.h"
#include "BodyItem.h"
#include <cnoid/ViewManager>
#include <cnoid/ConnectionSet>
#include <cnoid/SceneMarkers>
#include <cnoid/SceneView>
#include <cnoid/RadioButton>
#include <cnoid/ButtonGroup>
#include <cnoid/CheckBox>
#include <cnoid/Format>
#include <QBoxLayout>
#include <QLabel>
#include <QStyle>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class LinkMassSummaryView::Impl
{
public:
    LinkMassSummaryView* self;
    QLabel targetLabel;
    QLabel massLabel;
    QLabel cmLabels[3];
    RadioButton globalRadio;
    RadioButton localRadio;
    ButtonGroup coordinateRadioGroup;
    CheckBox markerCheck;

    BodySelectionManager* bodySelectionManager;
    ScopedConnection bodySelectionConnection;
    ScopedConnection linkSelectionConnection;

    BodyPtr targetBody;
    vector<LinkPtr> targetLinks;
    Vector3 centerOfMass;

    CrossMarkerPtr cmMarker;
    
    Impl(LinkMassSummaryView* self);
    void onActivated();
    void setTargetBody(BodyItem* bodyItem);
    void setTargetLinks(const std::vector<bool>& selection);
    void updateMassParameters();
    void showMarker(bool on);
};

}


void LinkMassSummaryView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LinkMassSummaryView>(
        N_("LinkMassSummaryView"), N_("Link Mass Summary"));
}


LinkMassSummaryView* LinkMassSummaryView::instance()
{
    static LinkMassSummaryView* instance_ = ViewManager::getOrCreateView<LinkMassSummaryView>();
    return instance_;
}


LinkMassSummaryView::LinkMassSummaryView()
{
    impl = new Impl(this);
}


LinkMassSummaryView::Impl::Impl(LinkMassSummaryView* self)
    : self(self)
{
    self->setDefaultLayoutArea(BottomLeftArea);

    auto vbox = new QVBoxLayout;
    self->setLayout(vbox, 1.0);

    vbox->addWidget(&targetLabel);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Mass:")));
    hbox->addWidget(&massLabel);
    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new QLabel(_("Center of mass:")));

    int hs = self->style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);

    hbox = new QHBoxLayout;
    hbox->addSpacing(hs);
    hbox->addWidget(new QLabel("X:"));
    hbox->addWidget(&cmLabels[0]);
    hbox->addSpacing(hs);
    hbox->addWidget(new QLabel("Y:"));
    hbox->addWidget(&cmLabels[1]);
    hbox->addSpacing(hs);
    hbox->addWidget(new QLabel("Z:"));
    hbox->addWidget(&cmLabels[2]);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    globalRadio.setText(_("Global"));
    globalRadio.setChecked(true);
    hbox->addWidget(&globalRadio);
    localRadio.setText(_("Local"));
    hbox->addWidget(&localRadio);

    coordinateRadioGroup.addButton(&globalRadio);
    coordinateRadioGroup.addButton(&localRadio);
    coordinateRadioGroup.sigIdToggled().connect(
        [this](int /* id */, bool checked){
            if(checked){
                updateMassParameters();
            }
        });
    
    hbox->addStretch();
    vbox->addLayout(hbox);

    markerCheck.setText(_("Show 3D Marker"));
    markerCheck.sigToggled().connect(
        [this](bool on){
            showMarker(on);
        });
    vbox->addWidget(&markerCheck);

    vbox->addStretch();

    bodySelectionManager = nullptr;
    centerOfMass.setZero();
}


LinkMassSummaryView::~LinkMassSummaryView()
{
    delete impl;
}


void LinkMassSummaryView::onActivated()
{
    impl->onActivated();
}


void LinkMassSummaryView::Impl::onActivated()
{
    if(!bodySelectionManager){
        bodySelectionManager = BodySelectionManager::instance();
    }
    
    bodySelectionConnection =
        bodySelectionManager->sigCurrentBodyItemChanged().connect(
            [this](BodyItem* bodyItem){
                setTargetBody(bodyItem);
            });

    setTargetBody(bodySelectionManager->currentBodyItem());
}


void LinkMassSummaryView::onDeactivated()
{
    impl->bodySelectionConnection.disconnect();
    impl->setTargetBody(nullptr);
}


void LinkMassSummaryView::Impl::setTargetBody(BodyItem* bodyItem)
{
    Body* body = nullptr;
    if(bodyItem){
        body = bodyItem->body();
    }
    if(body == targetBody){
        return;
    }

    if(!body){
        targetBody.reset();
        vector<bool> selection;
        setTargetLinks(selection);
        linkSelectionConnection.disconnect();
        
    } else {
        targetBody = body;
        linkSelectionConnection =
            bodySelectionManager->sigLinkSelectionChanged(bodyItem).connect(
                [this](const std::vector<bool>& selection){
                    setTargetLinks(selection);
                });
        setTargetLinks(bodySelectionManager->linkSelection(bodyItem));
    }
}


void LinkMassSummaryView::Impl::setTargetLinks(const std::vector<bool>& selection)
{
    targetLinks.clear();

    if(targetBody){
        int numLinks = targetBody->numLinks();
        int m = selection.size();
        int n = std::min(numLinks, m);
        for(int i=0; i < n; ++i){
            if(selection[i]){
                targetLinks.push_back(targetBody->link(i));
            }
        }
    }

    if(targetLinks.empty()){
        targetLabel.setText("------");
    } else {
        int numTargetLinks = targetLinks.size();
        if(numTargetLinks == 1){
            targetLabel.setText(targetLinks.front()->name().c_str());
        } else {
            targetLabel.setText(
                formatC("{0} ... ({1} links)", targetLinks[0]->name(), numTargetLinks).c_str());
        }
    }

    updateMassParameters();
}


void LinkMassSummaryView::Impl::updateMassParameters()
{
    double m = 0.0;
    
    if(targetLinks.empty()){
        centerOfMass.setZero();
    } else {
        Vector3 mc = Vector3::Zero();
        for(auto& link : targetLinks){
            Vector3 wc = link->R() * link->c() + link->p();
            mc += link->m() * wc;
            m += link->m();
        }
        centerOfMass = mc / m;
    }

    if(cmMarker){
        cmMarker->setTranslation(centerOfMass);
        cmMarker->notifyUpdate();
    }
    
    if(!targetLinks.empty() && localRadio.isChecked()){
        centerOfMass = targetBody->rootLink()->T().inverse() * centerOfMass;
    }

    massLabel.setText(QString::number(m));
    for(int i=0; i < 3; ++i){
        cmLabels[i].setText(QString::number(centerOfMass[i]));
    }
}


void LinkMassSummaryView::Impl::showMarker(bool on)
{
    if(on){
        if(!cmMarker){
            cmMarker = new CrossMarker(0.2, Vector3f(0.0f, 1.0f, 0.0f), 2.0);
            cmMarker->setTranslation(centerOfMass);
            cmMarker->setName("CenterOfMass");
        }
        for(auto& sceneView : SceneView::instances()){
            sceneView->systemNodeGroup()->addChild(cmMarker, true);
        }
    } else {
        if(cmMarker){
            for(auto& sceneView : SceneView::instances()){
                sceneView->systemNodeGroup()->removeChild(cmMarker, true);
            }
        }
    }
}
