/**
   @author Shin'ichiro Nakaoka
*/

#include "LinkPropertyView.h"
#include "BodySelectionManager.h"
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/ViewManager>
#include <cnoid/AppConfig>
#include <cnoid/DisplayValueFormat>
#include <cnoid/ConnectionSet>
#include <cnoid/Format>
#include <cnoid/MathUtil>
#include <QTableWidget>
#include <QHeaderView>
#include <QBoxLayout>
#include <QKeyEvent>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class LinkPropertyView::Impl : public QTableWidget
{
public:
    LinkPropertyView* self;
    ScopedConnection bodySelectionManagerConnection;
    ScopedConnection modelUpdateConnection;
    ScopedConnection dvFormatConnection;
    BodyItemPtr currentBodyItem;
    LinkPtr currentLink;
    int fontPointSizeDiff;

    Impl(LinkPropertyView* self);
    void onCurrentLinkChanged(BodyItem* bodyItem, Link* link);
    void updateLinkProperties(Link* link);
    void addProperty(const string& name, const QString& value);
    void addProperty(const string& name, const string& value);
    void addProperty(const string& name, const char* value);
    void addProperty(const string& name, int value);
    void addProperty(const string& name, double value);
    void addProperty(const string& name, const Vector3& value);
    void addProperty(const string& name, const AngleAxis& rotation);
    void addProperty(const string& name, const Matrix3& M);
    void zoomFontSize(int pointSizeDiff);
};

}

void LinkPropertyView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LinkPropertyView>(
        N_("LinkPropertyView"), N_("Link Properties"));
}


LinkPropertyView::LinkPropertyView()
{
    impl = new Impl(this);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(impl);
    setLayout(vbox);

    setDefaultLayoutArea(BottomLeftArea);
}


LinkPropertyView::Impl::Impl(LinkPropertyView* self)
    : self(self)
{
    setFrameShape(QFrame::NoFrame);
    setColumnCount(2);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::NoSelection);

    QHeaderView* hh = horizontalHeader();
    QHeaderView* vh = verticalHeader();
    hh->hide();
    vh->hide();
    hh->setSectionResizeMode(QHeaderView::Stretch);
    vh->setSectionResizeMode(QHeaderView::ResizeToContents);
    hh->setStretchLastSection(true);

    fontPointSizeDiff = 0;
    MappingPtr config = AppConfig::archive()->openMapping("LinkPropertyView");
    int storedFontPointSizeDiff;
    if(config->read("fontZoom", storedFontPointSizeDiff)){
        zoomFontSize(storedFontPointSizeDiff);
    }

    dvFormatConnection =
        DisplayValueFormat::master()->sigFormatChanged().connect(
            [this]{
                if(currentLink){
                    updateLinkProperties(currentLink);
                }
            });
}


LinkPropertyView::~LinkPropertyView()
{
    delete impl;
}


void LinkPropertyView::onActivated()
{
    auto bsm = BodySelectionManager::instance();

    impl->bodySelectionManagerConnection =
        bsm->sigCurrentChanged().connect(
            [this, bsm](BodyItem* bodyItem, Link* link){
                if(!link){
                    link = bsm->currentLink();
                }
                impl->onCurrentLinkChanged(bodyItem, link);
            });
    
    impl->onCurrentLinkChanged(bsm->currentBodyItem(), bsm->currentLink());
}


void LinkPropertyView::onDeactivated()
{
    impl->bodySelectionManagerConnection.disconnect();
    impl->onCurrentLinkChanged(nullptr, nullptr);
}


void LinkPropertyView::Impl::onCurrentLinkChanged(BodyItem* bodyItem, Link* link)
{
    if(bodyItem != currentBodyItem){
        if(!bodyItem){
            modelUpdateConnection.disconnect();
        } else {
            modelUpdateConnection =
                bodyItem->sigModelUpdated().connect(
                    [this](int flags){
                        if(flags & BodyItem::LinkSpecUpdate){
                            updateLinkProperties(currentLink);
                        }
                    });
        }
        currentBodyItem = bodyItem;
    }
    
    if(link != currentLink){
        updateLinkProperties(link);
        currentLink = link;
    }
}


void LinkPropertyView::Impl::updateLinkProperties(Link* link)
{
    setRowCount(0); // clear

    if(!link){
        return;
    }

    auto dvf = DisplayValueFormat::master();
    const double lengthRatio = dvf->ratioToDisplayLength();
    const char* lengthSymbol = dvf->lengthUnitSymbol();
    const bool isDegree = dvf->isDegree();
    const char* angleSymbol = isDegree ? "deg" : "rad";
    const char* angleVelocitySymbol = isDegree ? "deg/s" : "rad/s";

    addProperty(_("Name"), link->name());
    addProperty(_("Index"), link->index());
    addProperty(
        formatR(_("Offset translation [{0}]"), lengthSymbol),
        Vector3(link->offsetTranslation() * lengthRatio));
    addProperty(_("Offset rotation"), AngleAxis(link->offsetRotation()));
    addProperty(
        formatR(_("Center of mass [{0}]"), lengthSymbol),
        Vector3(link->centerOfMass() * lengthRatio));
    addProperty(_("Mass"), link->mass());
    addProperty(_("Inertia tensor"), link->I());
    addProperty(_("Material"), link->materialName());
    addProperty(_("Joint type"), link->jointTypeLabel());

    if(link->hasActualJoint()){
        addProperty(_("Joint axis"), link->jointAxis());

        double q_lower = link->q_lower();
        double q_upper = link->q_upper();
        double dq_lower = link->dq_lower();
        double dq_upper = link->dq_upper();
        string qUnit;
        string dqUnit;
        if(link->isRevoluteJoint()){
            if(isDegree){
                q_lower = degree(q_lower);
                q_upper = degree(q_upper);
                dq_lower = degree(dq_lower);
                dq_upper = degree(dq_upper);
            }
            qUnit = angleSymbol;
            dqUnit = angleVelocitySymbol;
        } else {
            q_lower *= lengthRatio;
            q_upper *= lengthRatio;
            dq_lower *= lengthRatio;
            dq_upper *= lengthRatio;
            qUnit = lengthSymbol;
            dqUnit = formatC("{0}/s", lengthSymbol);
        }
        addProperty(formatR(_("Lower joint limit [{0}]"), qUnit), q_lower);
        addProperty(formatR(_("Upper joint limit [{0}]"), qUnit), q_upper);
        addProperty(formatR(_("Lower joint velocity [{0}]"), dqUnit), dq_lower);
        addProperty(formatR(_("Upper joint velocity [{0}]"), dqUnit), dq_upper);

        addProperty(_("Joint inertia"), link->Jm2());
    }

    /*
    Mapping* info = link->info();
    for(Mapping::iterator p = info->begin(); p != info->end(); ++p){
        const string& key = p->first;
        ValueNode* node = p->second;
        if(node->isScalar()){
            addProperty(key, node->toString());
        } else if(node->isListing()){
            addProperty(key, node->toListing());
        }
    }
    */
}


void LinkPropertyView::Impl::addProperty(const string& name, const QString& value)
{
    int row = rowCount();
    setRowCount(row + 1);
    QTableWidgetItem* nameItem = new QTableWidgetItem(name.c_str());
    nameItem->setFlags(Qt::ItemIsEnabled);
    setItem(row, 0, nameItem);
    QTableWidgetItem* valueItem = new QTableWidgetItem(value);
    setItem(row, 1, valueItem);
}


void LinkPropertyView::Impl::addProperty(const string& name, const string& value)
{
    addProperty(name, QString(value.c_str()));
}


void LinkPropertyView::Impl::addProperty(const string& name, const char* value)
{
    addProperty(name, QString(value));
}


void LinkPropertyView::Impl::addProperty(const string& name, int value)
{
    addProperty(name, QString::number(value));
}


void LinkPropertyView::Impl::addProperty(const string& name, double value)
{
    addProperty(name, QString::number(value));
}


void LinkPropertyView::Impl::addProperty(const string& name, const Vector3& value)
{
    static const QString format("%1 %2 %3");
    addProperty(name, format.arg(value[0]).arg(value[1]).arg(value[2]));
}


void LinkPropertyView::Impl::addProperty(const string& name, const AngleAxis& rotation)
{
    static const QString format("%1 %2 %3 %4");
    const Vector3 a = rotation.axis();
    addProperty(name, format.arg(a[0]).arg(a[1]).arg(a[2]).arg(rotation.angle()));
}


void LinkPropertyView::Impl::addProperty(const string& name, const Matrix3& M)
{
    static const QString format("%1 %2 %3 %4 %5 %6 %7 %8 %9");
    addProperty(
        name,
        format
        .arg(M(0,0)).arg(M(0, 1)).arg(M(0, 2))
        .arg(M(1,0)).arg(M(1, 1)).arg(M(1, 2))
        .arg(M(2,0)).arg(M(2, 1)).arg(M(2, 2)));
}


void LinkPropertyView::keyPressEvent(QKeyEvent* event)
{
    if(event->modifiers() & Qt::ControlModifier){
        switch(event->key()){
        case Qt::Key_Plus:
        case Qt::Key_Semicolon:
            impl->zoomFontSize(1);
            return;
        case Qt::Key_Minus:
            impl->zoomFontSize(-1);
            return;
        }
    }
    View::keyPressEvent(event);
}


void LinkPropertyView::Impl::zoomFontSize(int pointSizeDiff)
{
    QFont f = font();
    f.setPointSize(f.pointSize() + pointSizeDiff);
    setFont(f);
    fontPointSizeDiff += pointSizeDiff;
    AppConfig::archive()->openMapping("LinkPropertyView")->write("fontZoom", fontPointSizeDiff);
}
