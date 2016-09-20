/**
   @author Shin'ichiro Nakaoka
   @author Hisashi Ikari
*/
#include "RTSNameServerView.h"
//#include "RTSPropertiesView.h"
#include <cnoid/ViewManager>
#include <cnoid/TreeWidget>
#include <cnoid/Buttons>
#include <cnoid/SpinBox>
#include <cnoid/LineEdit>
#include <QBoxLayout>
#include <QIcon>
#include <QMimeData>
#include <QDrag>
#include <boost/algorithm/string.hpp>
//#include "RTSDiagramView.h"
//#include "RTSCorbaUtil.h"
//#include "RTSCommonImpl.h"
#include "gettext.h"

#undef _HOST_CXT_VERSION

namespace cnoid {

class RTSVItem : public QTreeWidgetItem
{
public :
    RTSVItem(const NamingContextHelper::ObjectInfo& info){
        info_ = info;
        QString name = info_.id.c_str();
        setText(0, name);
        setIcon(0, info_.isAlive ? QIcon(":/Corba/icons/NSRTC.png") :
                QIcon(":/Corba/icons/NSZombi.png"));
    }
    NamingContextHelper::ObjectInfo info_;
};

class RTSNameTreeWidget : public TreeWidget
{
private :
    void mouseMoveEvent(QMouseEvent *event)
    {
        QByteArray itemData;
        QMimeData *mimeData = new QMimeData;
        mimeData->setData("application/RTSNameServerItem", itemData);
        QDrag *drag = new QDrag(this);
        drag->setPixmap(QPixmap(":/Corba/icons/NSRTC.png"));
        drag->setMimeData(mimeData);
        drag->start(Qt::MoveAction);
    }
};

class RTSNameServerViewImpl
{
public:
    RTSNameServerViewImpl(RTSNameServerView* self);
    ~RTSNameServerViewImpl();
    void updateObjectList();
    void updateObjectList(const NamingContextHelper::ObjectInfoList& objects, QTreeWidgetItem* parent);
    void setConnection();
    void onSelectionChanged();
    //void selectedItem();
#ifdef _HOST_CXT_VERSION
    void extendDiagram(const NamingContextHelper::ObjectInfo& info, QTreeWidgetItem* parent);
#endif
    void clearDiagram();
    void setSelection(string RTCName);

    Signal<void(const list<NamingContextHelper::ObjectInfo>&)> sigSelectionChanged;
    Signal<void(string, int)> sigLocationChanged;
    RTSNameTreeWidget treeWidget;
    LineEdit hostAddressBox;
    SpinBox portNumberSpin;

    NamingContextHelper ncHelper;
    list<NamingContextHelper::ObjectInfo> selectedItemList;
};

}


void RTSNameServerView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<RTSNameServerView>(
        "RTSNameServerView", N_("RTC List"), ViewManager::SINGLE_DEFAULT);
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


SignalProxy<void(const list<NamingContextHelper::ObjectInfo>&)> RTSNameServerView::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


SignalProxy<void(string, int)> RTSNameServerView::sigLocationChanged()
{
    return impl->sigLocationChanged;
}


list<NamingContextHelper::ObjectInfo> RTSNameServerView::getSelection()
{
    return impl->selectedItemList;
}


RTSNameServerViewImpl::RTSNameServerViewImpl(RTSNameServerView* self)
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);

    QVBoxLayout* vbox = new QVBoxLayout();

    QHBoxLayout* hbox = new QHBoxLayout();
    hostAddressBox.setText("localhost");
    hostAddressBox.sigEditingFinished().connect
        (std::bind(
            static_cast<void(RTSNameServerViewImpl::*)()>(&RTSNameServerViewImpl::updateObjectList), this));
    hbox->addWidget(&hostAddressBox);

    portNumberSpin.setRange(0, 65535);
    portNumberSpin.setValue(2809);
    portNumberSpin.sigEditingFinished().connect
        (std::bind(
            static_cast<void(RTSNameServerViewImpl::*)()>(&RTSNameServerViewImpl::updateObjectList), this));
    hbox->addWidget(&portNumberSpin);

    PushButton* updateButton = new PushButton(_("Update"));
    updateButton->sigClicked().connect(
        std::bind(
            static_cast<void(RTSNameServerViewImpl::*)()>(&RTSNameServerViewImpl::updateObjectList), this));
    hbox->addWidget(updateButton);

    vbox->addLayout(hbox);

    treeWidget.setHeaderLabel(_("Object Name"));
    treeWidget.setDragEnabled(true);
    treeWidget.setDropIndicatorShown(true);

    //treeWidget.sigItemClicked().connect(std::bind(&RTSNameServerViewImpl::selectedItem, this));
    treeWidget.sigItemSelectionChanged().connect(std::bind(&RTSNameServerViewImpl::onSelectionChanged, this));
    treeWidget.setSelectionMode(QAbstractItemView::ExtendedSelection);
    treeWidget.header()->close();

    vbox->addWidget(&treeWidget);
    self->setLayout(vbox);

    /*
    RTSPropertiesView* propertiesView = RTSPropertiesView::instance();
    if(propertiesView){
        Connection connection = sigSelectionChanged.connect(
                boost::bind(&RTSPropertiesView::onItemSelectionChanged, propertiesView, _1));
        propertiesView->setSelectionChangedConnection(connection);
        connection = sigLocationChanged.connect(
                boost::bind(&RTSPropertiesView::onLocationChanged, propertiesView, _1, _2));
        propertiesView->setLocationChangedConnection(connection);
    }
    */
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
{/*
    RTSPropertiesView* propertiesView = RTSPropertiesView::instance();
    if(propertiesView){
        propertiesView->clearSelectionChangedConnection();
        propertiesView->clearLocationChangedConnection();
    }*/
}


const string RTSNameServerView::getHost()
{
    return impl->hostAddressBox.string();
}


int RTSNameServerView::getPort()
{
    return impl->portNumberSpin.value();
}


void RTSNameServerViewImpl::updateObjectList()
{
    try {
        treeWidget.clear();

        // Update to connect information
        ncHelper.setLocation(hostAddressBox.string(), portNumberSpin.value());

        // Connection information updates to all views.
        sigLocationChanged(hostAddressBox.string(), portNumberSpin.value());

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
        if (boost::iequals(info.kind, "rtc") || boost::iequals(info.kind, "host_cxt")) {
            if (parent == NULL && boost::iequals(info.kind, "host_cxt")) {
            #ifdef _HOST_CXT_VERSION
                extendDiagram(info, parent);
            } else if (parent && boost::iequals(info.kind, "rtc")){
            #else
            } else if (boost::iequals(info.kind, "rtc")){
            #endif
                RTSVItem* item = new RTSVItem(info);
                //QTreeWidgetItem* item = new QTreeWidgetItem();
                //QString name = info.id.c_str();
                //item->setText(0, name);
                //item->setIcon(0, info.isAlive ? QIcon(":/Corba/icons/NSRTC.png") :
                //        QIcon(":/Corba/icons/NSZombi.png"));
                if (parent == NULL) treeWidget.addTopLevelItem(item);
                else parent->addChild(item);
            }
        }
    }
}


void RTSNameServerViewImpl::onSelectionChanged()
{
    selectedItemList.clear();

    QList<QTreeWidgetItem*> selected = treeWidget.selectedItems();
    for(int i=0; i < selected.size(); ++i){
        RTSVItem* item = dynamic_cast<RTSVItem*>(selected[i]);
        if(item){
            selectedItemList.push_back(item->info_);
        }
    }

    sigSelectionChanged(selectedItemList);
}


void RTSNameServerView::setSelection(string RTCName)
{
    impl->setSelection(RTCName);
}


void RTSNameServerViewImpl::setSelection(string RTCName)
{
    if(RTCName.empty()){
        treeWidget.clearSelection();
        return;
    }

    QList<QTreeWidgetItem*> items = treeWidget.findItems(QString(RTCName.c_str()), Qt::MatchFixedString);
    if(!items.empty())
        treeWidget.setCurrentItem(items[0]);

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
