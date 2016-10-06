/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
#include "RTSPropertiesView.h"
#include "RTSCommonUtil.h"
#include "RTSNameServerView.h"
#include "RTSDiagramView.h"
#include <cnoid/ViewManager>
#include <cnoid/TreeWidget>
#include <cnoid/ConnectionSet>
#include <QVBoxLayout>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <rtm/idl/RTC.hh>
#include <rtm/NVUtil.h>
#include <coil/Properties.h>
#include "gettext.h"

using namespace RTC;
using namespace cnoid;
using namespace std;
using namespace std::placeholders;

namespace {
    static const string RTC_CONTEXT_KIND [3] = {_("PERIODIC"), _("EVENT_DRIVEN"), _("OTHER")};
    static const string RTC_CONTEXT_STATE[4] = {_("CREATED"), _("INACTIVE"), _("ACTIVE"), _("ERROR")};

    static const string RTC_PROPERTY_DATAFLOW    [2] = {_("push"), _("pull")};
    static const string RTC_PROPERTY_INTERFACE   [1] = {_("corba_cdr")};
    static const string RTC_PROPERTY_SUBSCRIPTION[3] = {_("flush"), _("new"), _("periodic")};
};

namespace cnoid {

class RTSPropertyTreeWidget : public TreeWidget
{

};

class RTSPropertiesViewImpl
{
public:
    RTSPropertiesViewImpl(RTSPropertiesView* self);
    ~RTSPropertiesViewImpl();

    RTSPropertiesView* self;
    Connection selectionChangedConnection;
    Connection locationChangedConnection;

    void onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items);
    void onLocationChanged(std::string host, int port);
    void showProperties();
    void showPort(ComponentProfile_var cprofile, QTreeWidgetItem* item);
    void showPortConcrete(PortProfile* portprofile, QTreeWidgetItem* item);
    void showProfile(ComponentProfile_var cprofile, QTreeWidgetItem* item);
    void showExecutionContext(RTC::RTObject_ptr rtc, QTreeWidgetItem* item);
    void showExecutionContext(RTC::RTObject_ptr rtc, ExecutionContextList_var exeContList, QTreeWidgetItem* item);
    void showConnectionProperties(PortService_var port, string id);
    void showConnection(PortService_var port, string id, QTreeWidgetItem* item);

    RTSPropertyTreeWidget  treeWidget;
    NamingContextHelper ncHelper;

private:
    NamingContextHelper::ObjectInfo currentItem;
};

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

    // It colored a different color for each row.
    treeWidget.setAlternatingRowColors(true);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(&treeWidget);
    self->setLayout(vbox);

    RTSNameServerView* nsView = RTSNameServerView::instance();
    if(nsView){
        if(!selectionChangedConnection.connected()){
            selectionChangedConnection = nsView->sigSelectionChanged().connect(
                std::bind(&RTSPropertiesViewImpl::onItemSelectionChanged, this, _1));
        }
        if(!locationChangedConnection.connected()){
            locationChangedConnection = nsView->sigLocationChanged().connect(
                std::bind(&RTSPropertiesViewImpl::onLocationChanged, this, _1, _2));
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
    if(items.size()!=1)
        return;

    const NamingContextHelper::ObjectInfo& item = items.front();
    if(item.id != currentItem.id){
        currentItem.id = item.id;
        currentItem.isAlive = item.isAlive;
        currentItem.kind = item.kind;
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

    if(currentItem.id!="" && currentItem.isAlive){
        RTC::RTObject_ptr rtc = ncHelper.findObject<RTC::RTObject>(currentItem.id, "rtc");
        if(!ncHelper.isObjectAlive(rtc))
                return;
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
    currentItem.id = "";

    treeWidget.clear();
    QTreeWidgetItem *item = new QTreeWidgetItem;
    showConnection(port, id, item);
    treeWidget.insertTopLevelItem(0, item);
    treeWidget.expandAll();

}


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

    if(CORBA::is_nil(port) || port->_non_existent())
        return;
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



