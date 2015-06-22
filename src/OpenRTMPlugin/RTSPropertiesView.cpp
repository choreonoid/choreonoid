/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
#include "RTSPropertiesView.h"
#include "RTSCommonUtil.h"
#include "RTSNameServerView.h"
#include <cnoid/ViewManager>
#include <cnoid/TreeWidget>
#include <cnoid/ConnectionSet>
#include <QVBoxLayout>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <rtm/idl/RTC.hh>
#include <rtm/NVUtil.h>
#include <coil/Properties.h>

#if 0
#include <QComboBox>
#include <QFontMetrics>

#include <rtm/CorbaNaming.h>
#include <rtm/RTObject.h>
#include <rtm/CorbaConsumer.h>                




#include "RTSPropertiesItem.h"
#include "RTSDiagramView.h"
#include "RTSCommonUtil.h"
#include "RTSCorbaUtil.h"
#include "RTSCommonImpl.h"
#endif
#include "gettext.h"

using namespace RTC;
using namespace cnoid;
using namespace std;

namespace {
    static const string RTC_CONTEXT_KIND [3] = {_("PERIODIC"), _("EVENT_DRIVEN"), _("OTHER")};
    static const string RTC_CONTEXT_STATE[4] = {_("CREATED"), _("INACTIVE"), _("ACTIVE"), _("ERROR")};

    static const string RTC_PROPERTY_DATAFLOW    [2] = {_("push"), _("pull")};
    static const string RTC_PROPERTY_INTERFACE   [1] = {_("corba_cdr")};
    static const string RTC_PROPERTY_SUBSCRIPTION[3] = {_("flush"), _("new"), _("periodic")};
};

namespace cnoid {
    //typedef shared_ptr<NamingContextHelper> NamingContextHelperPtr;

    //#define IC static_cast<RTSComboBoxItem*>
    //#define IE static_cast<RTSPlainTextItem*>

class RTSPropertyTreeWidget : public TreeWidget
{
    /*
    Q_OBJECT
    friend class RTSPlainTextItem;
    friend class RTSPropertiesView;
    public:
        RTSPropertyTreeWidget() : selection(NULL) {
            connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
                          SLOT(onCurrentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)));
            connect(this, SIGNAL(itemActivated(QTreeWidgetItem*, int)),
                          SLOT(onItemActivated(QTreeWidgetItem*, int)));
            connect(this, SIGNAL(itemChanged(QTreeWidgetItem*, int)),
                          SLOT(onItemChanged(QTreeWidgetItem*, int)));
            connect(this, SIGNAL(itemEntered(QTreeWidgetItem*, int)),
                          SLOT(onItemEntered(QTreeWidgetItem*, int)));
            connect(this, SIGNAL(itemPressed(QTreeWidgetItem*, int)),
                          SLOT(onItemPressed(QTreeWidgetItem*, int)));
        }
        ~RTSPropertyTreeWidget() {
            if (selection) delete selection;
        }
        int decideIndex(const std::string* values, int count, std::string target);
        RTCConnection* updateConnection(std::string id, std::string name,
                                        std::string dataflow, std::string interface, std::string subscription);
        void showProperties(RTCConnection* conn);
        RTCConnection* refreshProperties(RTSConnectionItem* item);

    protected Q_SLOTS:
        void onCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
        void onItemActivated(QTreeWidgetItem* item, int column);
        void onItemChanged(QTreeWidgetItem* item, int column);
        void onItemEntered(QTreeWidgetItem* item, int column);
        void onItemPressed(QTreeWidgetItem* item, int column);

    protected:
        QWidget* selection;
        RTSConnectionItemPtr connItem;
        */
};

class RTSPropertiesViewImpl
{
public:
    RTSPropertiesViewImpl(RTSPropertiesView* self);
    ~RTSPropertiesViewImpl();

    RTSPropertiesView* self;
    Connection selectionChangedConnection;
    Connection locationChangedConnection;

    //void setSelectionChangedConnection(Connection& connection);
    //void clearSelectionChangedConnection();
    void onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items);
    //void setLocationChangedConnection(Connection& connection);
    //void clearLocationChangedConnection();
    void onLocationChanged(std::string host, int port);
    void showProperties();
    void showPort(ComponentProfile_var cprofile, QTreeWidgetItem* item);
    void showPortConcrete(PortProfile* portprofile, QTreeWidgetItem* item);
    void showProfile(ComponentProfile_var cprofile, QTreeWidgetItem* item);
    //void showConnection      (ComponentProfile_var cprofile, string port, string id, QTreeWidgetItem* item);
    void showExecutionContext(RTC::RTObject_ptr rtc, QTreeWidgetItem* item);
    void showExecutionContext(RTC::RTObject_ptr rtc, ExecutionContextList_var exeContList, QTreeWidgetItem* item);
    void showConnectionProperties(PortService_var port, string id);
    void showConnection(PortService_var port, string id, QTreeWidgetItem* item);

protected:
    //QTreeWidgetItem* judgementType(string name);

public:
    RTSPropertyTreeWidget  treeWidget;
    const NamingContextHelper::ObjectInfo* currentItem;
    NamingContextHelper ncHelper;

    // Holds the displayed data for updating the properties.
    //RTSConnectionItemPtr connItem;
};

#if 0
    class RTSConnectionItem
    {
    public:
        RTSConnectionItem() {}
        RTSConnectionItem(string id, string dataflow, string interface, string subscription) 
            : id(id), dataflow(dataflow), interface(interface), subscription(subscription) {}
    public:
        string id;
        string name;
        string dataflow;
        string interface;
        string subscription;
    };
#endif
}


void RTSPropertiesView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<RTSPropertiesView>(
            "RTSPropertiesView", N_("RTC Properties"), ViewManager::SINGLE_OPTIONAL);
}
 

RTSPropertiesView* RTSPropertiesView::instance()
{
    return ViewManager::findView<RTSPropertiesView>();
}


RTSPropertiesView::RTSPropertiesView()
{
    impl = new RTSPropertiesViewImpl(this);
}


RTSPropertiesView::~RTSPropertiesView()
{
    delete impl;
}


void RTSPropertiesView::showConnectionProperties(PortService_var port, string id)
{
    impl->showConnectionProperties(port, id);
}


RTSPropertiesViewImpl::RTSPropertiesViewImpl(RTSPropertiesView* self)
    : self(self)
{
    QStringList columns;
    columns << _("Object Name") << _("Object Value");
    treeWidget.setColumnCount(2);
    treeWidget.setHeaderLabels(columns);

    //impl->helper = NamingContextHelperPtr(new NamingContextHelper);

    // It colored a different color for each row.
    treeWidget.setAlternatingRowColors(true);
    //impl->connItem = RTSConnectionItemPtr(new RTSConnectionItem);
    //impl->treeWidget.connItem = impl->connItem;

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(&treeWidget);
    self->setLayout(vbox);

    RTSNameServerView* nsView = RTSNameServerView::instance();
    if(nsView){
        if(!selectionChangedConnection.connected()){
            selectionChangedConnection = nsView->sigSelectionChanged().connect(
                boost::bind(&RTSPropertiesViewImpl::onItemSelectionChanged, this, _1));
        }
        if(!locationChangedConnection.connected()){
            locationChangedConnection = nsView->sigLocationChanged().connect(
                boost::bind(&RTSPropertiesViewImpl::onLocationChanged, this, _1, _2));
            ncHelper.setLocation(nsView->getHost(), nsView->getPort());
        }
    }

}


RTSPropertiesViewImpl::~RTSPropertiesViewImpl()
{
    selectionChangedConnection.disconnect();
    locationChangedConnection.disconnect();
}


void RTSPropertiesViewImpl::onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items)
{
    const NamingContextHelper::ObjectInfo* item = items.size()==1 ? &items.front() : 0;
    if(item != currentItem){
        currentItem = item;
        showProperties();
    }
}


void RTSPropertiesViewImpl::onLocationChanged(string host, int port)
{
    ncHelper.setLocation(host, port);
}


void RTSPropertiesViewImpl::showProperties()
{
    treeWidget.clear();

    if(currentItem && currentItem->isAlive){
        RTC::RTObject_ptr rtc = ncHelper.findObject<RTC::RTObject>(currentItem->id, "rtc");
        ComponentProfile_var cprofile = rtc->get_component_profile();
        QTreeWidgetItem *item = new QTreeWidgetItem;

        showProfile(cprofile, item);
        showExecutionContext(rtc, item);
        showPort(cprofile, item);
        treeWidget.insertTopLevelItem(0, item);
    }

    treeWidget.expandAll();
}


void RTSPropertiesViewImpl::showConnectionProperties(PortService_var port, string id)
{
    currentItem = 0;

    treeWidget.clear();
    QTreeWidgetItem *item = new QTreeWidgetItem;
    showConnection(port, id, item);
    treeWidget.insertTopLevelItem(0, item);
    treeWidget.expandAll();

}

#if 0

    void RTSPropertiesView::showProperties(string srtc, string sport, string trtc, string tport, string id)
    {
        impl->treeWidget.clear();

        if (impl->helper->isAlive()) {
            CORBA::Object_var obj = RTCCorbaUtil::findObject(impl->helper, srtc);
            
            if (!CORBA::is_nil(obj)) {
                CorbaConsumer<RTObject> rtc;
                rtc.setObject(obj);

                try {
                    ComponentProfile_var cprofile = rtc->get_component_profile(); 
                    QTreeWidgetItem *item = new QTreeWidgetItem;
                    impl->showConnection(cprofile, sport, id, item);
                    impl->treeWidget.insertTopLevelItem(0, item);
                    impl->connItem->id = id;

                } catch (...) {
                    // Error message without display, It will display a blank.
                }
            
            } else {
                // If the RTC is dismissed from another tool, it is valid. I will ignore the error.
                return;
            }
        }

        // It will expand all.
        impl->treeWidget.expandAll();
    }


#endif


void RTSPropertiesViewImpl::showProfile(ComponentProfile_var cprofile, QTreeWidgetItem* item)
{
    item->setIcon(0, QIcon(":/Corba/icons/NSRTC.png"));
    item->setText(0, QString(string(cprofile->instance_name).c_str()));

    QTreeWidgetItem *rtcChild = new QTreeWidgetItem;
    rtcChild->setText(0, _("Instance Name"));
    rtcChild->setText(1, QString(string(cprofile->instance_name).c_str()));
    item->addChild(rtcChild);

    rtcChild = new QTreeWidgetItem;
    rtcChild->setText(0, _("Type Name"));
    rtcChild->setText(1, QString(string(cprofile->type_name).c_str()));
    item->addChild(rtcChild);

    rtcChild = new QTreeWidgetItem;
    rtcChild->setText(0, _("Description"));
    rtcChild->setText(1, QString(string(cprofile->description).c_str()));
    item->addChild(rtcChild);

    rtcChild = new QTreeWidgetItem;
    rtcChild->setText(0, _("Version"));
    rtcChild->setText(1, QString(string(cprofile->version).c_str()));
    item->addChild(rtcChild);

    rtcChild = new QTreeWidgetItem;
    rtcChild->setText(0, _("Vendor"));
    rtcChild->setText(1, QString(string(cprofile->vendor).c_str()));
    item->addChild(rtcChild);

    rtcChild = new QTreeWidgetItem;
    rtcChild->setText(0, _("Category"));
    rtcChild->setText(1, QString(string(cprofile->category).c_str()));
    item->addChild(rtcChild);

    // Get the rtc properties.
    QTreeWidgetItem* rtcProp = new QTreeWidgetItem;
    rtcProp->setText(0, _("Properties"));
    item->addChild(rtcProp);

    coil::Properties cproperties = NVUtil::toProperties(cprofile->properties);
    vector<string> rtcNames = cproperties.propertyNames();
    for (vector<string>::iterator iter = rtcNames.begin(); iter != rtcNames.end(); iter++) {
        QTreeWidgetItem* rtcPropChild = new QTreeWidgetItem;
        rtcPropChild->setText(0, QString((*iter).c_str()));
        rtcPropChild->setText(1, QString((cproperties[*iter]).c_str()));
        rtcProp->addChild(rtcPropChild);
    }
}


void RTSPropertiesViewImpl::showExecutionContext(RTC::RTObject_ptr rtc, QTreeWidgetItem* item)
{
    QTreeWidgetItem* ownedChild = new QTreeWidgetItem;
    ownedChild->setText(0, _("Owned"));
    item->addChild(ownedChild);

    ExecutionContextList_var exeContList = rtc->get_owned_contexts();
    showExecutionContext(rtc, exeContList, ownedChild);

    exeContList = rtc->get_participating_contexts();
    if(exeContList->length()){
        QTreeWidgetItem* child = new QTreeWidgetItem;
        child->setText(0, _("Participate"));
        item->addChild(child);
        showExecutionContext(rtc, exeContList, child);
    }
}


void RTSPropertiesViewImpl::showExecutionContext(RTC::RTObject_ptr rtc, ExecutionContextList_var exeContList, QTreeWidgetItem* item)
{
    CORBA::ULong elen(exeContList->length());
    for (CORBA::ULong e = 0; e < elen; e++) {

        ExecutionContext_var context = exeContList[e];

        QTreeWidgetItem* ownedProp = new QTreeWidgetItem;
        ownedProp->setText(0, "ExecutionContext");
        ownedProp->setIcon(0, QIcon(":/RTSystemEditor/icons/IconExecContext.png"));
        item->addChild(ownedProp);

        QTreeWidgetItem* ownedPropChild = new QTreeWidgetItem;
        ownedPropChild->setText(0, _("ID"));
        ownedPropChild->setText(1, QString((boost::lexical_cast<string>(e)).c_str()));
        ownedProp->addChild(ownedPropChild);

        ownedPropChild = new QTreeWidgetItem;
        ownedPropChild->setText(0, _("RTC State"));
        if (!CORBA::is_nil(rtc)) {
            ownedPropChild->setText(1, QString(RTC_CONTEXT_STATE[context->get_component_state(rtc)].c_str()));
        }
        ownedProp->addChild(ownedPropChild);

        ownedPropChild = new QTreeWidgetItem;
        ownedPropChild->setText(0, _("State"));
        ownedPropChild->setText(1, QString(context->is_running() ? _("RUNNING") : _("UNKNOWN")));
        ownedProp->addChild(ownedPropChild);

        ownedPropChild = new QTreeWidgetItem;
        ownedPropChild->setText(0, _("Kind"));
        ownedPropChild->setText(1, QString(RTC_CONTEXT_KIND[context->get_kind()].c_str()));
        ownedProp->addChild(ownedPropChild);

        ownedPropChild = new QTreeWidgetItem;
        ownedPropChild->setText(0, _("Rate"));
        ownedPropChild->setText(1, QString((boost::lexical_cast<string>(context->get_rate())).c_str()));
        ownedProp->addChild(ownedPropChild);

    }
}


void RTSPropertiesViewImpl::showPortConcrete(PortProfile* portprofile, QTreeWidgetItem* item)
{
    coil::Properties pproperties = NVUtil::toProperties(portprofile->properties);

    string portName = string(portprofile->name);
    RTCCommonUtil::splitPortName(portName);
    string portType = pproperties["port.port_type"];

    QTreeWidgetItem* port = new QTreeWidgetItem;
    if (boost::iequals(portType, "CorbaPort")) {
        port->setText(0, QString("ServicePort"));
        port->setIcon(0, QIcon(":/RTSystemEditor/icons/IconServicePort.png"));
    } else {
        port->setText(0, QString(portType.substr(4).c_str()));
        port->setIcon(0, QIcon(boost::iequals(portType, "DataOutPort") ?
                ":/RTSystemEditor/icons/IconOutPort.png" :
                ":/RTSystemEditor/icons/IconInPort.png"));
    }
    item->addChild(port);

    QTreeWidgetItem* portChild = new QTreeWidgetItem;
    portChild->setText(0, _("Name"));
    portChild->setText(1, QString(portName.c_str()));
    port->addChild(portChild);

    if (boost::iequals(portType, "CorbaPort")) {
        RTC::PortInterfaceProfileList iflist = portprofile->interfaces;
        for (CORBA::ULong i = 0; i < iflist.length(); i++) {
            QTreeWidgetItem* ifport = new QTreeWidgetItem;
            ifport->setText(0, QString("PortInterfaceProfile"));
            ifport->setIcon(0, QIcon(":/RTSystemEditor/icons/IconPIP.png"));
            port->addChild(ifport);

            QTreeWidgetItem* ifportc = new QTreeWidgetItem;
            ifportc->setText(0, QString("Interface Name"));
            ifportc->setText(1, QString(iflist[i].instance_name));
            ifport->addChild(ifportc);

            ifportc = new QTreeWidgetItem;
            ifportc->setText(0, QString("Type Name"));
            ifportc->setText(1, QString(iflist[i].type_name));
            ifport->addChild(ifportc);

            ifportc = new QTreeWidgetItem;
            ifportc->setText(0, QString("Port Interface Polarity"));
            ifportc->setText(1, QString((iflist[i].polarity == 0 ? "PROVIDED" : "REQUIRED")));
            ifport->addChild(ifportc);
        }
    }

    // Get the port properties.
    QTreeWidgetItem* portProp = new QTreeWidgetItem;
    portProp->setText(0, _("Properties"));
    port->addChild(portProp);

    vector<string> portNames = pproperties.propertyNames();
    for (vector<string>::iterator iter = portNames.begin(); iter != portNames.end(); iter++) {
        QTreeWidgetItem* portPropChild = new QTreeWidgetItem;
        portPropChild->setText(0, QString((*iter).c_str()));
        portPropChild->setText(1, QString((pproperties[*iter]).c_str()));
        portProp->addChild(portPropChild);
    }
}


void RTSPropertiesViewImpl::showPort(ComponentProfile_var cprofile, QTreeWidgetItem* item)
{
    // Get the port informations.
    PortProfileList portlist = cprofile->port_profiles;
    CORBA::ULong plen(portlist.length());
    for (CORBA::ULong i = 0; i < plen; i++) {
        PortProfile portprofile = portlist[i];
        showPortConcrete(&portprofile, item);
    }
}


void RTSPropertiesViewImpl::showConnection(PortService_var port, string id, QTreeWidgetItem* item)//ComponentProfile_var cprofile, string port, string id, QTreeWidgetItem* item)
{
    item->setText(0, _("Connector Profile"));
    item->setIcon(0, QIcon(":/RTSystemEditor/icons/IconConnector.png"));

    PortProfile_var portprofile = port->get_port_profile();
    ConnectorProfileList connections = portprofile->connector_profiles;
    for(CORBA::ULong i = 0; i < connections.length(); i++){
        ConnectorProfile connector = connections[i];
        if(id == string(connector.connector_id)){
            PortServiceList& connectedPorts = connector.ports;

            QTreeWidgetItem* conPropChild = new QTreeWidgetItem;
            conPropChild->setText(0, _("Connector ID"));
            conPropChild->setText(1, QString(connector.connector_id));
            item->addChild(conPropChild);

            //conPropChild = new RTSTreeItem(RTSTreeItem::RTS_ITEM_NAME_TYPE);
            conPropChild = new QTreeWidgetItem;
            conPropChild->setText(0, _("Name"));
            conPropChild->setText(1, QString(connector.name));
            // keep the values of connection.
            //this->connItem->name = string(connector.name);
            item->addChild(conPropChild);

            coil::Properties cproperties = NVUtil::toProperties(connector.properties);
            vector<string> names = cproperties.propertyNames();
            for (vector<string>::iterator iter = names.begin(); iter != names.end(); iter++) {
                    // view the attribute of rtc-connection.
                string name = (*iter).c_str();
                //QTreeWidgetItem* connPropChild = judgementType(name);
                conPropChild = new QTreeWidgetItem;
                conPropChild->setText(0, QString(name.c_str()));
                conPropChild->setText(1, QString((cproperties[*iter]).c_str()));
                item->addChild(conPropChild);
/*
                    // keep the values of connection.
                    if (iequals(name, "dataport.dataflow_type")) {
                        this->connItem->dataflow = string((cproperties[*iter]).c_str()); 
                    } else 
                    if (iequals(name, "dataport.interface_type")) {
                        this->connItem->interface = string((cproperties[*iter]).c_str()); 
                    } else 
                    if (iequals(name, "dataport.subscription_type")) {
                        this->connItem->subscription = string((cproperties[*iter]).c_str()); 
                    } */
            }

            // Show the port informations.
            CORBA::ULong pslen(connectedPorts.length());
            for (CORBA::ULong k = 0; k < pslen; k++) {
                PortService_var pservice = connectedPorts[k];
                PortProfile* pprofile = pservice->get_port_profile();

                showPortConcrete(pprofile, item);
            }
        }
    }

}

#if 0
    QTreeWidgetItem* RTSPropertiesViewImpl::judgementType(string name)
    {
        if (iequals(name, "Name")) {
            return new RTSTreeItem(RTSTreeItem::RTS_ITEM_NAME_TYPE); 
        } else 
        if (iequals(name, "dataport.dataflow_type")) {
            return new RTSTreeItem(RTSTreeItem::RTS_ITEM_DATAFLOW_TYPE); 
        } else 
        if (iequals(name, "dataport.interface_type")) {
            return new RTSTreeItem(RTSTreeItem::RTS_ITEM_INTERFACE_TYPE); 
        } else 
        if (iequals(name, "dataport.subscription_type")) {
            return new RTSTreeItem(RTSTreeItem::RTS_ITEM_SUBSCRIPTION_TYPE); 
        } else {
            return new QTreeWidgetItem; 
        }
    }


    void RTSPropertyTreeWidget::onCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous)
    {
        setItemWidget(previous, 1, NULL);

        try {
            bool presence = false; 
            if (selection) {
                presence = true;
                delete selection;
                selection = NULL;
            } 
            RTCConnection* retConn = NULL;
            if (presence) {
                retConn = refreshProperties(connItem.get());
                if (retConn) {
                    showProperties(retConn);
                } else {
                    clear();
                }    
            }

        } catch (...) { /* ignore the exception message */ }
     
    }


    void RTSPropertyTreeWidget::onItemActivated(QTreeWidgetItem* item, int column)
    {
        bool presence = false; 
        if (selection) {
            presence = true;
            delete selection;
            selection = NULL;
        }

        if (RTSTreeItem* rts = dynamic_cast<RTSTreeItem*>(item)) {
            switch (rts->type) {
                case RTSTreeItem::RTS_ITEM_NAME_TYPE: {
                    if (!presence) { // avoid the repaint
                        selection = new RTSPlainTextItem(item, 1, &(connItem->name));

                        QFontMetrics metrics(IE(selection)->font());
                        int height = metrics.lineSpacing();
                        IE(selection)->setFixedHeight(1 * (height * 1.8));
                        IE(selection)->setLineWrapMode(QPlainTextEdit::NoWrap);            

                        IE(selection)->appendPlainText(item->text(1));
                        setItemWidget(item, 1, selection);
                    }
                    break;
                }

                case RTSTreeItem::RTS_ITEM_DATAFLOW_TYPE: {
                    int currentTarget = decideIndex(RTC_PROPERTY_DATAFLOW, 2, qtos(item->text(1)));
                    selection = new RTSComboBoxItem(item, 1, &(connItem->dataflow));
                    for (int i = 0; i < 2; i++) {
                        IC(selection)->addItem(QString(RTC_PROPERTY_DATAFLOW[i].c_str()));
                    }
                    IC(selection)->setCurrentIndex(currentTarget);
                    setItemWidget(item, 1, selection);
                    break;
                }

                case RTSTreeItem::RTS_ITEM_INTERFACE_TYPE: {
                    int currentTarget = decideIndex(RTC_PROPERTY_INTERFACE, 1, qtos(item->text(1)));
                    selection = new RTSComboBoxItem(item, 1, &(connItem->interface));
                    for (int i = 0; i < 1; i++) {
                        IC(selection)->addItem(QString(RTC_PROPERTY_INTERFACE[i].c_str()));
                    }
                    IC(selection)->setCurrentIndex(currentTarget);
                    setItemWidget(item, 1, selection);
                    break;
                }

                case RTSTreeItem::RTS_ITEM_SUBSCRIPTION_TYPE: {
                    int currentTarget = decideIndex(RTC_PROPERTY_SUBSCRIPTION, 3, qtos(item->text(1)));
                    selection = new RTSComboBoxItem(item, 1, &(connItem->subscription));
                    for (int i = 0; i < 3; i++) {
                        IC(selection)->addItem(QString(RTC_PROPERTY_SUBSCRIPTION[i].c_str()));
                    }
                    IC(selection)->setCurrentIndex(currentTarget);
                    setItemWidget(item, 1, selection);
                    break;
                }

                default: {
                    break;
                }
            }
        }

        RTCConnection* retConn = NULL;
        if (presence) {
            retConn = refreshProperties(connItem.get());
            if (retConn) {
                showProperties(retConn);
            } else {
                clear();
            }    
        }

    }


    RTCConnection* RTSPropertyTreeWidget::refreshProperties(RTSConnectionItem* item)
    {
        RTCConnection* retConn = updateConnection(connItem->id, connItem->name, 
                                                  connItem->dataflow, connItem->interface, connItem->subscription);
        if (!retConn) {
            MessageView::instance()->putln((format(_("connection failure"))).str());
            return NULL;
        }
        return retConn;

    }

   
    void RTSPropertyTreeWidget::showProperties(RTCConnection* conn)
    {
        // repaint the properties-view.        
        vector<View*> views = ViewManager::allViews();
        for (vector<View*>::iterator iterv = views.begin(); iterv != views.end(); iterv++) {
            const string& viewName = (*iterv)->name();
            if (iequals(viewName, N_("RTC Properties"))) {
                RTSPropertiesView* proView = static_cast<RTSPropertiesView*>(*iterv);
                proView->showProperties(conn->sourceRtcName, conn->sourcePortName, 
                                        conn->targetRtcName, conn->targetPortName,
                                        conn->id);
                break;
            }       
        }
    }
 

    int RTSPropertyTreeWidget::decideIndex(const string* values, int count, string target)
    {
        for (int i = 0; i < count; i++) {
            if (iequals(values[i], target)) {
                return i;
            }
        } 
        return 0;
    }


    RTCConnection* RTSPropertyTreeWidget::updateConnection(string id, string name, string dataflow, string interface, string subscription)
    {
        vector<View*> views = ViewManager::allViews();
        for (vector<View*>::iterator iterv = views.begin(); iterv != views.end(); iterv++) {
            const string& viewName = (*iterv)->name();
            if (iequals(viewName, N_("RTC Diagram"))) {
                RTSDiagramView* diaView = static_cast<RTSDiagramView*>(*iterv);
                return diaView->updateConnection(id, name, dataflow, interface, subscription);
            }       
        }
        throwException((format(_("RTC Diagram view not found"))).str());       
        return NULL;
    }


    void RTSPropertyTreeWidget::onItemChanged(QTreeWidgetItem* item, int column)
    {
    }


    void RTSPropertyTreeWidget::onItemEntered(QTreeWidgetItem* item, int column)
    {
    }


    void RTSPropertyTreeWidget::onItemPressed(QTreeWidgetItem* item, int column)
    {
    }


    void RTSComboBoxItem::changeItem(int index)
    {
        if (0 <= index) {
            item->setText(this->column, this->itemText(index));
            *value = qtos(this->itemText(index));
        }
    }


    void RTSPlainTextItem::changeItem()
    {
        item->setText(this->column, this->toPlainText());
        *value = qtos(this->toPlainText());
    }


    void RTSPropertiesView::onActivated()
    {
    }


    void RTSPropertiesView::onDeactivated()
    {
    }
 

}; // end of namespace
#endif

