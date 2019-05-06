#include "PortConnectionDialog.h"
#include "RTSystem.h"
#include "RTSTypeUtil.h"
#include "LoggerUtil.h"
#include <cnoid/Buttons>
#include <QLabel>
#include <QMessageBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHeaderView>
#include "gettext.h"

using namespace std;
using namespace cnoid;

PortConnectionDialogBase::PortConnectionDialogBase()
    : isAccepted(false)
{
    nameLineEdit = new QLineEdit();

    detailCheck = new CheckBox((_("Detail")));
    detailCheck->sigToggled().connect(
        [&](bool on) { enableDetails(on); });

    frmDetail = new QFrame;
    QGridLayout* gridSubLayout = new QGridLayout(frmDetail);

    lstDetail = new QTableWidget(0, 2);
    lstDetail->setSelectionBehavior(QAbstractItemView::SelectRows);
    lstDetail->setSelectionMode(QAbstractItemView::SingleSelection);
    lstDetail->verticalHeader()->setVisible(false);
    lstDetail->setColumnWidth(0, 300);
    lstDetail->setColumnWidth(1, 300);
    lstDetail->setRowCount(0);
    lstDetail->setHorizontalHeaderLabels(QStringList() << "Name" << "Value");
    gridSubLayout->addWidget(lstDetail, 0, 0, 5, 1);

    PushButton* addButton = new PushButton(_("Add"));
    addButton->sigClicked().connect([&]() { onAddButtonClicked(); });
    gridSubLayout->addWidget(addButton, 0, 1, 1, 1);

    PushButton* deleteButton = new PushButton(_("Delete"));
    deleteButton->sigClicked().connect([&]() { onDeleteButtonClicked(); });
    gridSubLayout->addWidget(deleteButton, 1, 1, 1, 1);

    buttonFrame = new QFrame;
    QHBoxLayout* buttonBotLayout = new QHBoxLayout(buttonFrame);

    PushButton* cancelButton = new PushButton(_("&Cancel"));
    cancelButton->sigClicked().connect([&]() { reject(); });
    buttonBotLayout->addWidget(cancelButton);

    PushButton* okButton = new PushButton(_("&OK"));
    okButton->setDefault(true);
    okButton->sigClicked().connect([&]() { onOkButtonClicked(); });
    buttonBotLayout->addWidget(okButton);
}


void PortConnectionDialogBase::onAddButtonClicked()
{
    DDEBUG("PortConnectionDialogBase::onAddButtonClicked");

    int row = lstDetail->rowCount();
    lstDetail->insertRow(row);

    QTableWidgetItem* nameItem = new QTableWidgetItem;
    nameItem->setText("NewName");
    lstDetail->setItem(row, 0, nameItem);

    QTableWidgetItem* valueItem = new QTableWidgetItem;
    valueItem->setText("NewValue");
    lstDetail->setItem(row, 1, valueItem);
}


void PortConnectionDialogBase::onDeleteButtonClicked()
{
    DDEBUG("PortConnectionDialogBase::onDeleteButtonClicked");

    int currRow = lstDetail->currentRow();
    lstDetail->removeRow(currRow);
    lstDetail->setFocus();
}


void PortConnectionDialogBase::enableDetails(bool on)
{
    frmEditSub->setVisible(on);
    frmEditSub->setEnabled(on);
    adjustSize();
}


void PortConnectionDialogBase::addProperty(string name, string value)
{
    NamedValuePtr param(new NamedValue(name, value));
    propList.push_back(param);
}


DataPortConnectionDialog::DataPortConnectionDialog()
{
    setWindowTitle("Data Connector Profile");

    QLabel* label01 = new QLabel(_("name : "));
    label01->setAlignment(Qt::AlignRight);

    QLabel* label02 = new QLabel(_("Data Type : "));
    label02->setAlignment(Qt::AlignRight);
    dataTypeCombo = new ComboBox;

    QLabel* label03 = new QLabel(_("Interface Type : "));
    label03->setAlignment(Qt::AlignRight);
    interfaceCombo = new ComboBox;

    QLabel* label04 = new QLabel(_("Dataflow Type : "));
    label04->setAlignment(Qt::AlignRight);
    dataflowCombo = new ComboBox;

    QLabel* label05 = new QLabel(_("Subscription Type : "));
    label05->setAlignment(Qt::AlignRight);
    subscriptionCombo = new ComboBox;
    subscriptionCombo->sigCurrentIndexChanged().connect(
        [&](int index) { subscriptionComboSelectionChanged(index); });

    QLabel* label06 = new QLabel(_("Push Rate(Hz) : "));
    label06->setAlignment(Qt::AlignRight);
    pushrateEdit = new QLineEdit;
    pushrateEdit->setEnabled(false);

    QLabel* label07 = new QLabel(_("Push Policy : "));
    label07->setAlignment(Qt::AlignRight);
    pushpolicyCombo = new ComboBox;
    pushpolicyCombo->addItem("all");
    pushpolicyCombo->addItem("fifo");
    pushpolicyCombo->addItem("skip");
    pushpolicyCombo->addItem("new");
    pushpolicyCombo->sigCurrentIndexChanged().connect(
        [&](int index) { pushpolicyComboSelectionChanged(index); });
    pushpolicyCombo->setEnabled(false);

    QLabel* label08 = new QLabel(_("Skip Count : "));
    label08->setAlignment(Qt::AlignRight);
    skipcountEdit = new QLineEdit;
    skipcountEdit->setEnabled(false);

    QFrame* frmEditMain = new QFrame;
    QGridLayout* editMainLayout = new QGridLayout(frmEditMain);
    editMainLayout->addWidget(label01, 0, 0, 1, 1);
    editMainLayout->addWidget(nameLineEdit, 0, 1, 1, 1);
    editMainLayout->addWidget(label02, 1, 0, 1, 1);
    editMainLayout->addWidget(dataTypeCombo, 1, 1, 1, 1);
    editMainLayout->addWidget(label03, 2, 0, 1, 1);
    editMainLayout->addWidget(interfaceCombo, 2, 1, 1, 1);
    editMainLayout->addWidget(label04, 3, 0, 1, 1);
    editMainLayout->addWidget(dataflowCombo, 3, 1, 1, 1);
    editMainLayout->addWidget(label05, 4, 0, 1, 1);
    editMainLayout->addWidget(subscriptionCombo, 4, 1, 1, 1);
    editMainLayout->addWidget(label06, 5, 0, 1, 1);
    editMainLayout->addWidget(pushrateEdit, 5, 1, 1, 1);
    editMainLayout->addWidget(label07, 6, 0, 1, 1);
    editMainLayout->addWidget(pushpolicyCombo, 6, 1, 1, 1);
    editMainLayout->addWidget(label08, 7, 0, 1, 1);
    editMainLayout->addWidget(skipcountEdit, 7, 1, 1, 1);
    editMainLayout->addWidget(detailCheck, 8, 0, 1, 1);
    /////
    QLabel* labelSO01 = new QLabel(_("Buffer length : "));
    labelSO01->setAlignment(Qt::AlignRight);
    outLengthEdit = new QLineEdit();
    outLengthEdit->setText("8");

    QLabel* labelSO02 = new QLabel(_("Buffer full policy : "));
    labelSO02->setAlignment(Qt::AlignRight);
    outPolicyCombo = new ComboBox;
    outPolicyCombo->addItem("overwrite");
    outPolicyCombo->addItem("do_nothing");
    outPolicyCombo->addItem("block");

    QLabel* labelSO03 = new QLabel(_("Buffer write timeout : "));
    labelSO03->setAlignment(Qt::AlignRight);
    outWriteTimeoutEdit = new QLineEdit();
    outWriteTimeoutEdit->setText("1.0");

    QLabel* labelSO04 = new QLabel(_("Buffer empty policy : "));
    labelSO04->setAlignment(Qt::AlignRight);
    outEmptyCombo = new ComboBox;
    outEmptyCombo->addItem("readback");
    outEmptyCombo->addItem("do_nothing");
    outEmptyCombo->addItem("block");

    QLabel* labelSO05 = new QLabel(_("Buffer read timeout : "));
    labelSO05->setAlignment(Qt::AlignRight);
    outReadTimeoutEdit = new QLineEdit();
    outReadTimeoutEdit->setText("1.0");

    QGroupBox *outGroup = new QGroupBox(_("Buffer (Outport)"));
    QGridLayout* outSubLayout = new QGridLayout(outGroup);
    outSubLayout->addWidget(labelSO01, 0, 0, 1, 1);
    outSubLayout->addWidget(outLengthEdit, 0, 1, 1, 1);
    outSubLayout->addWidget(labelSO02, 1, 0, 1, 1);
    outSubLayout->addWidget(outPolicyCombo, 1, 1, 1, 1);
    outSubLayout->addWidget(labelSO03, 2, 0, 1, 1);
    outSubLayout->addWidget(outWriteTimeoutEdit, 2, 1, 1, 1);
    outSubLayout->addWidget(labelSO04, 3, 0, 1, 1);
    outSubLayout->addWidget(outEmptyCombo, 3, 1, 1, 1);
    outSubLayout->addWidget(labelSO05, 4, 0, 1, 1);
    outSubLayout->addWidget(outReadTimeoutEdit, 4, 1, 1, 1);
    //
    QLabel* labelSI01 = new QLabel(_("Buffer length : "));
    labelSI01->setAlignment(Qt::AlignRight);
    inLengthEdit = new QLineEdit();
    inLengthEdit->setText("8");

    QLabel* labelSI02 = new QLabel(_("Buffer full policy : "));
    labelSI02->setAlignment(Qt::AlignRight);
    inPolicyCombo = new ComboBox;
    inPolicyCombo->addItem("overwrite");
    inPolicyCombo->addItem("do_nothing");
    inPolicyCombo->addItem("block");

    QLabel* labelSI03 = new QLabel(_("Buffer write timeout : "));
    labelSI03->setAlignment(Qt::AlignRight);
    inWriteTimeoutEdit = new QLineEdit();
    inWriteTimeoutEdit->setText("1.0");

    QLabel* labelSI04 = new QLabel(_("Buffer empty policy : "));
    labelSI04->setAlignment(Qt::AlignRight);
    inEmptyCombo = new ComboBox;
    inEmptyCombo->addItem("readback");
    inEmptyCombo->addItem("do_nothing");
    inEmptyCombo->addItem("block");

    QLabel* labelSI05 = new QLabel(_("Buffer read timeout : "));
    labelSI05->setAlignment(Qt::AlignRight);
    inReadTimeoutEdit = new QLineEdit();
    inReadTimeoutEdit->setText("1.0");

    QGroupBox *inGroup = new QGroupBox(_("Buffer (Inport)"));
    QGridLayout* inSubLayout = new QGridLayout(inGroup);
    inSubLayout->addWidget(labelSI01, 0, 0, 1, 1);
    inSubLayout->addWidget(inLengthEdit, 0, 1, 1, 1);
    inSubLayout->addWidget(labelSI02, 1, 0, 1, 1);
    inSubLayout->addWidget(inPolicyCombo, 1, 1, 1, 1);
    inSubLayout->addWidget(labelSI03, 2, 0, 1, 1);
    inSubLayout->addWidget(inWriteTimeoutEdit, 2, 1, 1, 1);
    inSubLayout->addWidget(labelSI04, 3, 0, 1, 1);
    inSubLayout->addWidget(inEmptyCombo, 3, 1, 1, 1);
    inSubLayout->addWidget(labelSI05, 4, 0, 1, 1);
    inSubLayout->addWidget(inReadTimeoutEdit, 4, 1, 1, 1);

    frmEditSub = new QFrame;
    QGridLayout* editSubLayout = new QGridLayout(frmEditSub);
    editSubLayout->addWidget(outGroup, 0, 0, 1, 1);
    editSubLayout->addWidget(inGroup, 0, 1, 1, 1);
    editSubLayout->addWidget(frmDetail, 1, 0, 1, 2);
    frmEditSub->setVisible(false);
    frmEditSub->setEnabled(false);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(frmEditMain);
    mainLayout->addWidget(frmEditSub);
    mainLayout->addWidget(buttonFrame);
    setLayout(mainLayout);

    setMinimumWidth(800);
}


void DataPortConnectionDialog::setDisp(RTSPort* source, RTSPort* target)
{
    DDEBUG("CreateConnectionDialog::setDisp");
    nameLineEdit->setText(QString((source->name + "_" + target->name).c_str()));
    //
    vector<string> types = RTSTypeUtil::getAllowDataTypes(source, target);
    //bool isAllowAny = RTCCommonUtil::isAllowAnyDataType(source, target);
    dataTypeCombo->clear();
    for (int index = 0; index < types.size(); index++) {
        dataTypeCombo->addItem(QString::fromStdString(types[index]));
    }
    dataTypeCombo->setCurrentIndex(0);
    //
    types = RTSTypeUtil::getAllowInterfaceTypes(source, target);
    interfaceCombo->clear();
    for (int index = 0; index < types.size(); index++) {
        interfaceCombo->addItem(QString::fromStdString(types[index]));
    }
    interfaceCombo->setCurrentIndex(0);
    //
    types = RTSTypeUtil::getAllowDataflowTypes(source, target);
    dataflowCombo->clear();
    for (int index = 0; index < types.size(); index++) {
        dataflowCombo->addItem(QString::fromStdString(types[index]));
    }
    dataflowCombo->setCurrentIndex(0);
    //
    types = RTSTypeUtil::getAllowSubscriptionTypes(source, target);
    subscriptionCombo->clear();
    for (int index = 0; index < types.size(); index++) {
        subscriptionCombo->addItem(QString::fromStdString(types[index]));
    }
    subscriptionCombo->setCurrentIndex(0);
}


void DataPortConnectionDialog::subscriptionComboSelectionChanged(int target)
{
    QString selected = subscriptionCombo->currentText().toLower();
    DDEBUG_V("CreateConnectionDialog::subscriptionComboSelectionChanged %s", selected.toStdString().c_str());
    if (selected == "flush") {
        pushrateEdit->clear();
        pushrateEdit->setEnabled(false);
        pushpolicyCombo->setEnabled(false);
        skipcountEdit->clear();
        skipcountEdit->setEnabled(false);

    } else if (selected == "new") {
        pushrateEdit->clear();
        pushrateEdit->setEnabled(false);
        pushpolicyCombo->setEnabled(true);
        skipcountEdit->clear();
        skipcountEdit->setEnabled(false);

    } else if (selected == "periodic") {
        pushrateEdit->setEnabled(true);
        pushrateEdit->setText("1000.0");
        pushpolicyCombo->setEnabled(true);
        skipcountEdit->clear();
        skipcountEdit->setEnabled(false);
    }
}


void DataPortConnectionDialog::pushpolicyComboSelectionChanged(int target)
{
    QString selected = pushpolicyCombo->currentText().toLower();
    if (selected == "skip") {
        skipcountEdit->setText("0");
        skipcountEdit->setEnabled(true);
    } else {
        skipcountEdit->clear();
        skipcountEdit->setEnabled(false);
    }
}


void DataPortConnectionDialog::onOkButtonClicked()
{
    DDEBUG("DataPortConnectionDialog::onOkButtonClicked");

    QList<QString> keyList;
    propList.clear();

    addProperty("dataport.data_type", dataTypeCombo->currentText().toStdString());
    addProperty("dataport.interface_type", interfaceCombo->currentText().toStdString());
    addProperty("dataport.dataflow_type", dataflowCombo->currentText().toStdString());
    addProperty("dataport.subscription_type", subscriptionCombo->currentText().toStdString());
    keyList.append(QString::fromStdString("dataport.data_type"));
    keyList.append(QString::fromStdString("dataport.interface_type"));
    keyList.append(QString::fromStdString("dataport.dataflow_type"));
    keyList.append(QString::fromStdString("dataport.subscription_type"));
    if (pushrateEdit->isEnabled()) {
        addProperty("dataport.push_rate", pushrateEdit->text().toStdString());
        keyList.append(QString::fromStdString("dataport.push_rate"));
    }
    if (pushpolicyCombo->isEnabled()) {
        addProperty("dataport.publisher.push_policy", pushpolicyCombo->currentText().toStdString());
        keyList.append(QString::fromStdString("dataport.publisher.push_policy"));
    }
    if (skipcountEdit->isEnabled()) {
        addProperty("dataport.publisher.skip_count", skipcountEdit->text().toStdString());
        keyList.append(QString::fromStdString("dataport.publisher.skip_count"));
    }
    //
    if (detailCheck->isChecked()) {
        DDEBUG("CreateConnectionDialog::okClicked Detail Checked");
        addProperty("dataport.outport.buffer.length", outLengthEdit->text().toStdString());
        addProperty("dataport.outport.buffer.write.full_policy", outPolicyCombo->currentText().toStdString());
        addProperty("dataport.outport.buffer.write.timeout", outWriteTimeoutEdit->text().toStdString());
        addProperty("dataport.outport.buffer.read.empty_policy", outEmptyCombo->currentText().toStdString());
        addProperty("dataport.outport.buffer.read.timeout", outReadTimeoutEdit->text().toStdString());
        keyList.append(QString::fromStdString("dataport.outport.buffer.length"));
        keyList.append(QString::fromStdString("dataport.outport.buffer.write.full_policy"));
        keyList.append(QString::fromStdString("dataport.outport.buffer.write.timeout"));
        keyList.append(QString::fromStdString("dataport.outport.buffer.read.empty_policy"));
        keyList.append(QString::fromStdString("dataport.outport.buffer.read.timeout"));
        //
        addProperty("dataport.inport.buffer.length", inLengthEdit->text().toStdString());
        addProperty("dataport.inport.buffer.write.full_policy", inPolicyCombo->currentText().toStdString());
        addProperty("dataport.inport.buffer.write.timeout", inWriteTimeoutEdit->text().toStdString());
        addProperty("dataport.inport.buffer.read.empty_policy", inEmptyCombo->currentText().toStdString());
        addProperty("dataport.inport.buffer.read.timeout", inReadTimeoutEdit->text().toStdString());
        keyList.append(QString::fromStdString("dataport.inport.buffer.length"));
        keyList.append(QString::fromStdString("dataport.inport.buffer.write.full_policy"));
        keyList.append(QString::fromStdString("dataport.inport.buffer.write.timeout"));
        keyList.append(QString::fromStdString("dataport.inport.buffer.read.empty_policy"));
        keyList.append(QString::fromStdString("dataport.inport.buffer.read.timeout"));
        //
        if (0 < lstDetail->rowCount()) {
            for (int index = 0; index < lstDetail->rowCount(); index++) {
                QString strName = lstDetail->item(index, 0)->text();
                QString strValue = lstDetail->item(index, 1)->text();
                DDEBUG_V("Item: %s, %s", strName.toStdString().c_str(), strValue.toStdString().c_str());
                if (keyList.contains(strName)) {
                    QMessageBox::warning(this, _("Port Connect"), _("Duplicate property name."));
                    return;
                } else {
                    keyList.append(strName);
                }
                addProperty(strName.toStdString(), strValue.toStdString());
            }
        }
    }

    isAccepted = true;
    close();
}


ServicePortConnectionDialog::ServicePortConnectionDialog()
{
    setWindowTitle("Service Connector Profile");

    QLabel* label01 = new QLabel(_("name : "));
    label01->setAlignment(Qt::AlignRight);

    QFrame* frmEditMain = new QFrame;
    QGridLayout* editMainLayout = new QGridLayout(frmEditMain);
    editMainLayout->addWidget(label01, 0, 0, 1, 1);
    editMainLayout->addWidget(nameLineEdit, 0, 1, 1, 1);
    editMainLayout->addWidget(detailCheck, 1, 0, 1, 1);

    QFrame* frmGridIF = new QFrame;
    QGridLayout* gridIFLayout = new QGridLayout(frmGridIF);

    lstInterface = new QTableWidget(0, 2);
    lstInterface->setSelectionBehavior(QAbstractItemView::SelectRows);
    lstInterface->setSelectionMode(QAbstractItemView::SingleSelection);
    lstInterface->verticalHeader()->setVisible(false);
    lstInterface->setColumnWidth(0, 400);
    lstInterface->setColumnWidth(1, 400);
    lstInterface->setRowCount(0);
    lstInterface->setHorizontalHeaderLabels(QStringList() << "Consumer" << "Provider");
    gridIFLayout->addWidget(lstInterface, 0, 0, 5, 1);

    PushButton* addButton_IF = new PushButton(_("Add"));
    addButton_IF->sigClicked().connect([&]() { addIF(); });
    gridIFLayout->addWidget(addButton_IF, 0, 1, 1, 1);

    PushButton* deleteButton_IF = new PushButton(_("Delete"));
    deleteButton_IF->sigClicked().connect([&]() { deleteIF(); });
    gridIFLayout->addWidget(deleteButton_IF, 1, 1, 1, 1);

    frmEditSub = new QFrame;
    QVBoxLayout* editSubLayout = new QVBoxLayout(frmEditSub);
    editSubLayout->addWidget(frmGridIF);
    editSubLayout->addWidget(frmDetail);
    frmEditSub->setVisible(false);
    frmEditSub->setEnabled(false);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(frmEditMain);
    mainLayout->addWidget(frmEditSub);
    mainLayout->addWidget(buttonFrame);
    setLayout(mainLayout);

    setMinimumWidth(1000);
}


void ServicePortConnectionDialog::setDisp(RTSPort* source, RTSPort* target)
{
    nameLineEdit->setText(QString((source->name + "_" + target->name).c_str()));

    registInterfaceMap(source);
    registInterfaceMap(target);
}


void ServicePortConnectionDialog::registInterfaceMap(RTSPort* port)
{
    string compName = "unknown";
    string portName = "unknown";
    //
    string name = port->name;
    if (!name.empty()) {
        vector<string> names = RTCCommonUtil::split(name, '.');
        if (names.size() < 2) {
            if (port->rtsComp != 0) {
                compName = port->rtsComp->name;
            }
            portName = names[0];
        } else {
            compName = names[0];
            portName = names[1];
        }
    }

    for (int index = 0; index < port->interList.size(); index++) {
        PortInterfacePtr ip = port->interList[index];
        ip->rtc_name = compName;
        ip->port_name = portName;
        string label = ip->toDispStr();
        DDEBUG_V("label:%s", label.c_str());
        if (ip->isRequiredPolarity()) {
            consumerLabelList.push_back(label);
            consumerConList.push_back(ip->toStr());
        } else {
            providerLabelList.push_back(label);
            providerConList.push_back(ip->toStr());
        }
    }
}


void ServicePortConnectionDialog::addIF()
{
    DDEBUG("ServicePortConnectionDialog::addIF");

    int row = lstInterface->rowCount();
    lstInterface->insertRow(row);

    QComboBox* consumerCombo = new QComboBox();
    lstInterface->setCellWidget(row, 0, consumerCombo);
    for (int index = 0; index < consumerLabelList.size(); index++) {
        consumerCombo->addItem(QString::fromStdString(consumerLabelList[index]));
    }
    QComboBox* providerCombo = new QComboBox();
    lstInterface->setCellWidget(row, 1, providerCombo);
    for (int index = 0; index < providerLabelList.size(); index++) {
        providerCombo->addItem(QString::fromStdString(providerLabelList[index]));
    }
}


void ServicePortConnectionDialog::deleteIF()
{
    int currRow = lstInterface->currentRow();
    lstInterface->removeRow(currRow);
    lstInterface->setFocus();
}


void ServicePortConnectionDialog::onOkButtonClicked()
{
    DDEBUG("ServicePortConnectionDialog::onOkButtonClicked");

    QList<QString> keyList;
    propList.clear();

    if (detailCheck->isChecked()) {
        if (0 < lstInterface->rowCount()) {
            for (int index = 0; index < lstInterface->rowCount(); index++) {
                QComboBox* consumerCombo = (QComboBox*)lstInterface->cellWidget(index, 0);
                QComboBox* providerCombo = (QComboBox*)lstInterface->cellWidget(index, 1);
                int conIndex = consumerCombo->currentIndex();
                string strConsumer = consumerConList[conIndex];
                int proIndex = providerCombo->currentIndex();
                string strProvider = providerConList[proIndex];
                DDEBUG_V("Item IF: %s, %s", strConsumer.c_str(), strProvider.c_str());
                if (keyList.contains(QString::fromStdString(strConsumer))) {
                    QMessageBox::warning(this, _("Port Connect"), _("Duplicate property name."));
                    return;
                } else {
                    keyList.append(QString::fromStdString(strConsumer));
                }
                addProperty(strConsumer, strProvider);
            }
        }
        if (0 < lstDetail->rowCount()) {
            for (int index = 0; index < lstDetail->rowCount(); index++) {
                QString strName = lstDetail->item(index, 0)->text();
                QString strValue = lstDetail->item(index, 1)->text();
                DDEBUG_V("Item: %s, %s", strName.toStdString().c_str(), strValue.toStdString().c_str());
                if (keyList.contains(strName)) {
                    QMessageBox::warning(this, _("Port Connect"), _("Duplicate property name."));
                    return;
                } else {
                    keyList.append(strName);
                }
                addProperty(strName.toStdString(), strValue.toStdString());
            }
        }
    }

    isAccepted = true;
    close();
}
