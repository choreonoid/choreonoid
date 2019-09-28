/**
   @author Shin'ichiro Nakaoka
*/

#include "LinkPropertyView.h"
#include "LinkSelectionView.h"
#include "BodyBar.h"
#include <cnoid/Link>
#include <cnoid/ViewManager>
#include <cnoid/AppConfig>
#include <cnoid/ConnectionSet>
#include <QTableWidget>
#include <QHeaderView>
#include <QBoxLayout>
#include <QKeyEvent>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;


namespace cnoid {

class LinkPropertyViewImpl : public QTableWidget
{
public:
    LinkPropertyView* self;
    LinkSelectionView* linkSelectionView;
    ConnectionSet connections;
    int fontPointSizeDiff;

    LinkPropertyViewImpl(LinkPropertyView* self);
    ~LinkPropertyViewImpl();
    void onCurrentBodyItemChanged(BodyItem* bodyItem);
    void clear();
    void updateProperties();
    void updateLinkProperties(Link* link);
    void addProperty(const string& name, const QString& value);
    void addProperty(const string& name, const string& value);
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
        "LinkPropertyView", N_("Link Properties"), ViewManager::SINGLE_OPTIONAL);
}


LinkPropertyView::LinkPropertyView()
{
    impl = new LinkPropertyViewImpl(this);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(impl);
    setLayout(vbox);

    setDefaultLayoutArea(View::LEFT_BOTTOM);
}


LinkPropertyViewImpl::LinkPropertyViewImpl(LinkPropertyView* self)
    : self(self)
{
    linkSelectionView = LinkSelectionView::mainInstance();
    
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

    connections.add(
        BodyBar::instance()->sigCurrentBodyItemChanged().connect(
            std::bind(&LinkPropertyViewImpl::onCurrentBodyItemChanged, this, _1)));

    connections.add(
        linkSelectionView->sigSelectionChanged().connect(
            std::bind(&LinkPropertyViewImpl::updateProperties, this)));
}


LinkPropertyView::~LinkPropertyView()
{
    delete impl;
}


LinkPropertyViewImpl::~LinkPropertyViewImpl()
{
    connections.disconnect();
}


void LinkPropertyViewImpl::onCurrentBodyItemChanged(BodyItem* bodyItem)
{
    if(!bodyItem){
        clear();
    }
}


void LinkPropertyViewImpl::clear()
{
    setRowCount(0);
}


void LinkPropertyViewImpl::updateProperties()
{
    clear();
        
    BodyItem* bodyItem = linkSelectionView->currentBodyItem();
    if(bodyItem){
        Body* body = bodyItem->body();
        int linkIndex = linkSelectionView->selectedLinkIndex();
        if(linkIndex >= 0){
            Link* link = body->link(linkIndex);
            if(link){
                updateLinkProperties(link);
            }
        }
    }
}


void LinkPropertyViewImpl::updateLinkProperties(Link* link)
{
    addProperty(_("Name"), link->name());
    addProperty(_("Index"), link->index());
    addProperty(_("Offset translation"), Vector3(link->offsetTranslation()));
    addProperty(_("Offset rotation"), AngleAxis(link->offsetRotation()));
    addProperty(_("Rs"), link->Rs());
    addProperty(_("Center of mass"), link->centerOfMass());
    addProperty(_("Mass"), link->mass());
    addProperty(_("Inertia tensor"), link->I());
    addProperty(_("Joint type"), link->jointTypeString());
    if(link->isRotationalJoint() || link->isSlideJoint()){
        addProperty(_("Joint axis"), link->jointAxis());
        addProperty(_("Upper joint limit"), link->q_upper());
        addProperty(_("Lower joint limit"), link->q_lower());
        addProperty(_("Upper joint velocity"), link->dq_upper());
        addProperty(_("Lower joint velocity"), link->dq_lower());
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


void LinkPropertyViewImpl::addProperty(const string& name, const QString& value)
{
    int row = rowCount();
    setRowCount(row + 1);
    QTableWidgetItem* nameItem = new QTableWidgetItem(name.c_str());
    nameItem->setFlags(Qt::ItemIsEnabled);
    setItem(row, 0, nameItem);
    QTableWidgetItem* valueItem = new QTableWidgetItem(value);
    setItem(row, 1, valueItem);
}


void LinkPropertyViewImpl::addProperty(const string& name, const string& value)
{
    addProperty(name, QString(value.c_str()));
}


void LinkPropertyViewImpl::addProperty(const string& name, int value)
{
    addProperty(name, QString::number(value));
}


void LinkPropertyViewImpl::addProperty(const string& name, double value)
{
    addProperty(name, QString::number(value));
}


void LinkPropertyViewImpl::addProperty(const string& name, const Vector3& value)
{
    static const QString format("%1 %2 %3");
    addProperty(name, format.arg(value[0]).arg(value[1]).arg(value[2]));
}


void LinkPropertyViewImpl::addProperty(const string& name, const AngleAxis& rotation)
{
    static const QString format("%1 %2 %3 %4");
    const Vector3 a = rotation.axis();
    addProperty(name, format.arg(a[0]).arg(a[1]).arg(a[2]).arg(rotation.angle()));
}


void LinkPropertyViewImpl::addProperty(const string& name, const Matrix3& M)
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


void LinkPropertyViewImpl::zoomFontSize(int pointSizeDiff)
{
    QFont f = font();
    f.setPointSize(f.pointSize() + pointSizeDiff);
    setFont(f);
    fontPointSizeDiff += pointSizeDiff;
    AppConfig::archive()->openMapping("LinkPropertyView")->write("fontZoom", fontPointSizeDiff);
}
