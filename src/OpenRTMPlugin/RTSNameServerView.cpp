/**
   @author Shin'ichiro Nakaoka
   @author Hisashi Ikari
*/
#include "RTSNameServerView.h"
#include <cnoid/CorbaUtil>
#include <cnoid/ViewManager>
#include <cnoid/TreeWidget>
#include <cnoid/Button>
#include <cnoid/SpinBox>
#include <cnoid/LineEdit>
#include <QBoxLayout>
#include <QIcon>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
//#include "RTSPropertiesView.h"
//#include "RTSDiagramView.h"
//#include "RTSCorbaUtil.h"
//#include "RTSCommonImpl.h"
#include "gettext.h"

#undef _HOST_CXT_VERSION

using namespace boost;

namespace cnoid {

class RTSNameServerViewImpl
{
public:
	RTSNameServerViewImpl(RTSNameServerView* self);
	~RTSNameServerViewImpl();
	void updateObjectList();
	void updateObjectList(const NamingContextHelper::ObjectInfoList& objects, QTreeWidgetItem* parent);
	void setConnection();
	void selectedItem();
#ifdef _HOST_CXT_VERSION
	void extendDiagram(const NamingContextHelper::ObjectInfo& info, QTreeWidgetItem* parent);
#endif
	void clearDiagram();
	TreeWidget treeWidget;
	LineEdit hostAddressBox;
	SpinBox portNumberSpin;

	NamingContextHelper ncHelper;
};

}


void RTSNameServerView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<RTSNameServerView>(
        "RTSNameServerView", N_("RTC List"), ViewManager::SINGLE_OPTIONAL);
}


RTSNameServerView* RTSNameServerView::instance()
{
    return ViewManager::findView<RTSNameServerView>();
}

#if 0
void RTSNameServerView::onActivated()
{
    try {
        impl->setConnection();
    } catch (...) { /* ignore the exception */ }
}


TreeWidget* RTSNameServerView::getTreeWidget()
{
    return &(impl->treeWidget);
}


void RTSNameServerView::setConnection()
{
    impl->setConnection();
}
#endif

RTSNameServerView::RTSNameServerView()
{
    impl = new RTSNameServerViewImpl(this);
}


RTSNameServerViewImpl::RTSNameServerViewImpl(RTSNameServerView* self)
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);

    QVBoxLayout* vbox = new QVBoxLayout();

    QHBoxLayout* hbox = new QHBoxLayout();
    hostAddressBox.setText("localhost");
    hostAddressBox.sigEditingFinished().connect
        (bind(&RTSNameServerViewImpl::updateObjectList, this));
    hbox->addWidget(&hostAddressBox);

    portNumberSpin.setRange(0, 65535);
    portNumberSpin.setValue(2809);
    portNumberSpin.sigEditingFinished().connect
        (bind(&RTSNameServerViewImpl::updateObjectList, this));
    hbox->addWidget(&portNumberSpin);

    PushButton* updateButton = new PushButton(_("Update"));
    updateButton->sigClicked().connect(bind(&RTSNameServerViewImpl::updateObjectList, this));
    hbox->addWidget(updateButton);

    vbox->addLayout(hbox);

    treeWidget.setHeaderLabel(_("Object Name"));
    treeWidget.setDragEnabled(true);
    treeWidget.setDropIndicatorShown(true);
    //treeWidget.sigItemClicked().connect(bind(&RTSNameServerViewImpl::selectedItem, this));
    treeWidget.setSelectionMode(QAbstractItemView::ExtendedSelection);
    treeWidget.header()->close();

    vbox->addWidget(&treeWidget);
    self->setLayout(vbox);
}

#if 0
void RTSNameServerViewImpl::setConnection()
{
    QString addressText = hostAddressBox.text();
    string address = string(qtos(addressText));
    int port = portNumberSpin.value(); 
    ncHelper->setLocation(address, port);

    // It set the information to connect to the view of all.
    try {
        vector<View*> views = ViewManager::allViews();
        for (vector<View*>::iterator iterv = views.begin(); iterv != views.end(); iterv++) {
            const string& viewName = (*iterv)->name();
            if (iequals(viewName, N_("RTC Properties"))) {
                RTSPropertiesView* propView = static_cast<RTSPropertiesView*>(*iterv);    
                propView->setConnection(address, port);
            }   
            if (iequals(viewName, N_("RTC Diagram"))) {
                RTSDiagramView* diagramView = static_cast<RTSDiagramView*>(*iterv);    
                diagramView->setConnection(address, port);
            }
        }   
    } catch (...) {
        // ignore the exception, just catching for non crash.
    }
}


void RTSNameServerViewImpl::clearDiagram()
{
    // It clear the all diagram on diagram-view.
    vector<View*> views = ViewManager::allViews();
    for (vector<View*>::iterator iterv = views.begin(); iterv != views.end(); iterv++) {
        const string& viewName = (*iterv)->name();
        if (iequals(viewName, N_("RTC Diagram"))) {
            RTSDiagramView* diagramView = static_cast<RTSDiagramView*>(*iterv);    
            diagramView->clearDiagram();
            return;
        }
    }   
}


void RTSNameServerViewImpl::selectedItem()
{
    QModelIndex index = treeWidget.currentIndex();
    QString temp = treeWidget.model()->data(index).toString();
    string item(qtos(temp));

    // It set the information to connect to the view of all.
    try {
        vector<View*> views = ViewManager::allViews();
        for (vector<View*>::iterator iterv = views.begin(); iterv != views.end(); iterv++) {
            const string& viewName = (*iterv)->name();
            if (iequals(viewName, N_("RTC Properties"))) {
                RTSPropertiesView* propView = static_cast<RTSPropertiesView*>(*iterv);    
                propView->showProperties(item);
                return;
            }   
        }   
    } catch (...) {
        // ignore the exception, just catching for non crash.
    }
}
#endif

RTSNameServerView::~RTSNameServerView()
{
    delete impl;
}


RTSNameServerViewImpl::~RTSNameServerViewImpl()
{

}


void RTSNameServerViewImpl::updateObjectList()
{
    try {
        treeWidget.clear();

        // Update to connect information
        ncHelper.setLocation(hostAddressBox.string(), portNumberSpin.value());

        // Connection information updates to all views.
        //setConnection();

        // Clear information update to all views.
        //clearDiagram();
        
        if(ncHelper.isAlive()){
            NamingContextHelper::ObjectInfoList objects = ncHelper.getObjectList();
            updateObjectList(objects, NULL);
            treeWidget.expandAll();
        }

    } catch (...) {
        // ignore the exception for non crash.
    }
}


void RTSNameServerViewImpl::updateObjectList(const NamingContextHelper::ObjectInfoList& objects, QTreeWidgetItem* parent)
{
    for(size_t i = 0; i < objects.size(); ++i){
        const NamingContextHelper::ObjectInfo& info = objects[i];
        if (iequals(info.kind, "rtc") || iequals(info.kind, "host_cxt")) {
            if (parent == NULL && iequals(info.kind, "host_cxt")) {
            #ifdef _HOST_CXT_VERSION
                extendDiagram(info, parent);
            } else if (parent && iequals(info.kind, "rtc")){
            #else
            } else if (iequals(info.kind, "rtc")){
            #endif
                QTreeWidgetItem* item = new QTreeWidgetItem();
                QString name = info.id.c_str();
                item->setText(0, name);
                item->setIcon(0, info.isAlive ? QIcon(":/Corba/icons/NSRTC.png") :
                		QIcon(":/Corba/icons/NSZombi.png"));
                if (parent == NULL) treeWidget.addTopLevelItem(item);
                else parent->addChild(item);
            }
        }
    }
}


#ifdef _HOST_CXT_VERSION
void RTSNameServerViewImpl::extendDiagram(const NamingContextHelper::ObjectInfo& info, QTreeWidgetItem* parent)
{
    // view the host context.
    QTreeWidgetItem* item = new QTreeWidgetItem();
    QString name = info.id.c_str();
    string rtcName = string(qtos(name));
    item->setIcon(0, QIcon(":/Corba/icons/NSHostCxt.png"));
    item->setText(0, QString(string(rtcName).c_str()));
    if (parent == NULL) treeWidget.addTopLevelItem(item);
    else parent->addChild(item);

    // view the rtc in host context.
    NamingContextHelperPtr helper = NamingContextHelperPtr(new NamingContextHelper);
    QString addressText = hostAddressBox.text();
    string address = string(qtos(addressText));
    int port = portNumberSpin.value(); 
    helper->setLocation(address, port);
    if (helper->isAlive()) {
        CORBA::Object_var obj = helper->findObject(rtcName, "host_cxt");
        helper->bindObject(obj, rtcName, "host_cxt");
        NamingContextHelper::ObjectInfoList objects = helper->getObjectList();
        updateObjectList(objects, item);
        //RTS_RELEASE(obj, "release");
        if (!CORBA::is_nil(obj)) CORBA::release(obj);
    }

}
#endif
