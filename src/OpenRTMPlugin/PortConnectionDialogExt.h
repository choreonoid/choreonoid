#ifndef CNOID_OPENRTM_PLUGIN_PORT_CONNECTION_DIALOG_EXT_H
#define CNOID_OPENRTM_PLUGIN_PORT_CONNECTION_DIALOG_EXT_H

#include "PortConnectionDialog.h"
#include "RTSystemExtItem.h"
#include <cnoid/Dialog>
#include <cnoid/ComboBox>
#include <cnoid/CheckBox>
#include <QLineEdit>
#include <QTableWidget>

namespace cnoid {

class PortConnectionDialogBaseExt : public Dialog
{
public:
    QLineEdit* nameLineEdit;
    std::vector<NamedValueExtPtr> propList;
    bool isAccepted;

protected:
    CheckBox* detailCheck;
    QFrame* frmEditSub;
    QTableWidget* lstDetail;
    QFrame* frmDetail;
    QFrame* buttonFrame;

    PortConnectionDialogBaseExt();
    void addProperty(std::string name, std::string value);
    virtual void onOkButtonClicked() = 0;

private:
    void onAddButtonClicked();
    void onDeleteButtonClicked();
    void enableDetails(bool on);
};

class DataPortConnectionDialogExt : public PortConnectionDialogBaseExt
{
public:
    DataPortConnectionDialogExt();
    void setDisp(RTSPortExt* source, RTSPortExt* target);

private:
    ComboBox* dataTypeCombo;
    ComboBox* interfaceCombo;
    ComboBox* dataflowCombo;
    ComboBox* subscriptionCombo;
    QLineEdit* pushrateEdit;
    ComboBox* pushpolicyCombo;
    QLineEdit* skipcountEdit;

    QLineEdit* outLengthEdit;
    ComboBox* outPolicyCombo;
    QLineEdit* outWriteTimeoutEdit;
    ComboBox* outEmptyCombo;
    QLineEdit* outReadTimeoutEdit;

    QLineEdit* inLengthEdit;
    ComboBox* inPolicyCombo;
    QLineEdit* inWriteTimeoutEdit;
    ComboBox* inEmptyCombo;
    QLineEdit* inReadTimeoutEdit;

    void subscriptionComboSelectionChanged(int target);
    void pushpolicyComboSelectionChanged(int target);
    void onOkButtonClicked() override;
};

class ServicePortConnectionDialogExt : public PortConnectionDialogBaseExt
{
public:
    ServicePortConnectionDialogExt();
    void setDisp(RTSPortExt* source, RTSPortExt* target);

private:
    QTableWidget* lstInterface;
    std::vector<std::string> consumerLabelList;
    std::vector<std::string> consumerConList;
    std::vector<std::string> providerLabelList;
    std::vector<std::string> providerConList;

    void registInterfaceMap(RTSPortExt* port);
    void addIF();
    void deleteIF();
    void onOkButtonClicked() override;
};

}

#endif
