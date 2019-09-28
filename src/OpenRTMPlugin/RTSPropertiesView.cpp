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
#include <cnoid/AppConfig>
#include <cnoid/Buttons>
#include <QVBoxLayout>
#include <boost/algorithm/string.hpp>
#include <rtm/idl/RTC.hh>
#include <rtm/NVUtil.h>
#include <coil/Properties.h>

#include <QLabel>
#include <QMessageBox>

#include "LoggerUtil.h"
#include "gettext.h"

using namespace RTC;
using namespace cnoid;
using namespace std;

namespace {
static const string RTC_CONTEXT_KIND[3] = { _("PERIODIC"), _("EVENT_DRIVEN"), _("OTHER") };
static const string RTC_CONTEXT_STATE[4] = { _("CREATED"), _("INACTIVE"), _("ACTIVE"), _("ERROR") };

static const string RTC_PROPERTY_DATAFLOW[2] = { _("push"), _("pull") };
static const string RTC_PROPERTY_INTERFACE[1] = { _("corba_cdr") };
static const string RTC_PROPERTY_SUBSCRIPTION[3] = { _("flush"), _("new"), _("periodic") };
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

    void onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items);
    void showProperties();
    void showPort(ComponentProfile_var cprofile, QTreeWidgetItem* item);
    void showPortConcrete(PortProfile* portprofile, QTreeWidgetItem* item);
    void showProfile(ComponentProfile_var cprofile, QTreeWidgetItem* item);
    void showExecutionContext(RTC::RTObject_ptr rtc, QTreeWidgetItem* item);
    void showExecutionContext(RTC::RTObject_ptr rtc, ExecutionContextList_var exeContList, QTreeWidgetItem* item);
    void showConnectionProperties(PortService_var port, string id);
    void showConnection(PortService_var port, string id, QTreeWidgetItem* item);

    RTSPropertyTreeWidget  treeWidget;

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
    if (nsView) {
        if (!selectionChangedConnection.connected()) {
            selectionChangedConnection = nsView->sigSelectionChanged().connect(
                [&](const list<NamingContextHelper::ObjectInfo>& items){
                    onItemSelectionChanged(items);
                });
        }
    }
}


RTSPropertiesViewImpl::~RTSPropertiesViewImpl()
{
    selectionChangedConnection.disconnect();
}


void RTSPropertiesViewImpl::onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items)
{
    DDEBUG_V("RTSPropertiesViewImpl::onItemSelectionChanged : %d", items.size());
    if (items.size() != 1)
        return;

    const NamingContextHelper::ObjectInfo& item = items.front();
    if (item.id_ == currentItem.id_
        && item.hostAddress_ == currentItem.hostAddress_
        && item.portNo_ == currentItem.portNo_) {
        return;
    }
    currentItem.id_ = item.id_;
    currentItem.isAlive_ = item.isAlive_;
    currentItem.kind_ = item.kind_;
    currentItem.fullPath_ = item.fullPath_;
    currentItem.hostAddress_ = item.hostAddress_;
    currentItem.portNo_ = item.portNo_;
    showProperties();
}


void RTSPropertiesViewImpl::showProperties()
{
    DDEBUG("RTSPropertiesViewImpl::showProperties");
    treeWidget.clear();

    if (currentItem.id_ != "" && currentItem.isAlive_) {
        NamingContextHelper* ncHelper = NameServerManager::instance()->getNCHelper();
        ncHelper->setLocation(currentItem.hostAddress_, currentItem.portNo_);
        RTC::RTObject_var rtc = ncHelper->findObject<RTC::RTObject>(currentItem.fullPath_);

        ComponentProfile_var cprofile;
        try {
            cprofile = rtc->get_component_profile();
        } catch (CORBA::SystemException& ex) {
            ncHelper->putExceptionMessage(ex);
            return;
        }

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
    currentItem.id_ = "";

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
    if (exeContList->length()) {
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
        ownedPropChild->setText(1, QString::number(e));
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
        ownedPropChild->setText(1, QString::number(context->get_rate()));
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

    if (CORBA::is_nil(port) || port->_non_existent())
        return;
    PortProfile_var portprofile = port->get_port_profile();
    ConnectorProfileList connections = portprofile->connector_profiles;
    for (CORBA::ULong i = 0; i < connections.length(); i++) {
        ConnectorProfile connector = connections[i];
        if (id == string(connector.connector_id)) {
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
//////////
SettingDialog::SettingDialog()
{
    chkLog = new CheckBox(_("Log Output"));
    chkLog->sigToggled().connect([&](bool on){ logChanged(on); });

    QLabel* lblLevel = new QLabel(_("Log Level:"));
    cmbLogLevel = new ComboBox();
    cmbLogLevel->addItem("SILENT");
    cmbLogLevel->addItem("FATAL");
    cmbLogLevel->addItem("ERROR");
    cmbLogLevel->addItem("WARN");
    cmbLogLevel->addItem("INFO");
    cmbLogLevel->addItem("DEBUG");
    cmbLogLevel->addItem("TRACE");
    cmbLogLevel->addItem("VERBOSE");
    cmbLogLevel->addItem("PARANOID");

    QLabel* lblSetting = new QLabel(_("Setting:"));
    leSetting = new LineEdit;

    QLabel* lblName = new QLabel(_("VendorName:"));
    leName = new LineEdit;
    QLabel* lblVersion = new QLabel(_("Version:"));
    leVersion = new LineEdit;

#if defined(OPENRTM_VERSION12)
    QLabel* lblHeartBeat = new QLabel(_("Heartbeat Period:"));
    leHeartBeat = new LineEdit;
    QLabel* lblUnitHb = new QLabel("ms");
#endif

    QFrame* frmDetail = new QFrame;
    QGridLayout* gridSubLayout = new QGridLayout(frmDetail);
    gridSubLayout->addWidget(chkLog, 0, 0, 1, 1);
    gridSubLayout->addWidget(lblLevel, 0, 1, 1, 1);
    gridSubLayout->addWidget(cmbLogLevel, 0, 2, 1, 1);

    gridSubLayout->addWidget(lblSetting, 1, 0, 1, 1);
    gridSubLayout->addWidget(leSetting, 1, 1, 1, 2);

    gridSubLayout->addWidget(lblName, 2, 0, 1, 1);
    gridSubLayout->addWidget(leName, 2, 1, 1, 2);
    gridSubLayout->addWidget(lblVersion, 3, 0, 1, 1);
    gridSubLayout->addWidget(leVersion, 3, 1, 1, 2);

#if defined(OPENRTM_VERSION12)
    gridSubLayout->addWidget(lblHeartBeat, 5, 0, 1, 1);
    gridSubLayout->addWidget(leHeartBeat, 5, 1, 1, 2);
    gridSubLayout->addWidget(lblUnitHb, 5, 3, 1, 1);
#endif

    QFrame* frmButton = new QFrame;

    auto okButton = new PushButton(_("&OK"));
    okButton->setDefault(true);
    okButton->sigClicked().connect([&](){ oKClicked(); });

    auto cancelButton = new PushButton(_("&Cancel"));
    cancelButton->sigClicked().connect([&](){ rejected(); });

    QHBoxLayout* buttonBotLayout = new QHBoxLayout(frmButton);
    buttonBotLayout->addWidget(cancelButton);
    buttonBotLayout->addStretch();
    buttonBotLayout->addWidget(okButton);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(frmDetail);
    mainLayout->addWidget(frmButton);
    setLayout(mainLayout);

    setWindowTitle(_("OpenRTM Preferences"));

    MappingPtr appVars = AppConfig::archive()->openMapping("OpenRTM");
    leSetting->setText(QString::fromStdString(appVars->get("defaultSetting", DEFAULT_CONF_FILENAME)));
    leName->setText(QString::fromStdString(appVars->get("defaultVendor", "AIST")));
    leVersion->setText(QString::fromStdString(appVars->get("defaultVersion", "1.0.0")));

#if defined(OPENRTM_VERSION12)
    leHeartBeat->setText(QString::number(appVars->get("heartBeatPeriod", 500)));
#endif

    chkLog->setChecked(appVars->get("outputLog", false));

    QString level = QString::fromStdString(appVars->get("logLevel", "INFO"));
    cmbLogLevel->setCurrentText(level);
    cmbLogLevel->setEnabled(chkLog->isChecked());
}


void SettingDialog::logChanged(bool state)
{
    cmbLogLevel->setEnabled(chkLog->isChecked());
}


void SettingDialog::oKClicked()
{
    DDEBUG("SettingDialog::oKClicked");
    MappingPtr appVars = AppConfig::archive()->openMapping("OpenRTM");

    QString orgSetting = QString::fromStdString(appVars->get("defaultSetting", "./choreonoid.rtc.conf"));
    QString newSetting = leSetting->text();
    bool orgLog = appVars->get("outputLog", false);
    bool newLog = chkLog->isChecked();
    QString orgLevel = QString::fromStdString(appVars->get("logLevel", "INFO"));
    QString newLevel = cmbLogLevel->currentText();
    bool isRestart = (orgSetting != newSetting) || (orgLog != newLog) || (orgLevel != newLevel);

    appVars->write("defaultSetting", leSetting->text().toStdString(), DOUBLE_QUOTED);
    appVars->write("defaultVendor", leName->text().toStdString(), DOUBLE_QUOTED);
    appVars->write("defaultVersion", leVersion->text().toStdString(), DOUBLE_QUOTED);

#if defined(OPENRTM_VERSION12)
    appVars->write("heartBeatPeriod", leHeartBeat->text().toInt());
#endif

    appVars->write("outputLog", chkLog->isChecked());
    appVars->write("logLevel", cmbLogLevel->currentText().toStdString());

    if (isRestart) {
        QMessageBox::warning(this, _("OpenRTM Preferences"), _("The specified setting becomes valid after RESTART."));
    }
    close();
}


void SettingDialog::rejected()
{
    close();
}




