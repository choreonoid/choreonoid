#ifndef CNOID_OPENRTM_PLUGIN_PORT_CONNECTION_DIALOG_EXT_H
#define CNOID_OPENRTM_PLUGIN_PORT_CONNECTION_DIALOG_EXT_H

#include "PortConnectionDialog.h"
#include "RTSystemExt2Item.h"
#include <cnoid/Dialog>
#include <cnoid/ComboBox>
#include <cnoid/CheckBox>
#include <QLineEdit>
#include <QTableWidget>

namespace cnoid {

class PortConnectionDialogBaseExt2 : public Dialog
{
public:
    QLineEdit* nameLineEdit;
    std::vector<NamedValueExt2Ptr> propList;
    bool isAccepted;

protected:
    CheckBox* detailCheck;
    QFrame* frmEditSub;
    QTableWidget* lstDetail;
    QFrame* frmDetail;
    QFrame* buttonFrame;

    PortConnectionDialogBaseExt2();
    void addProperty(std::string name, std::string value);
    virtual void onOkButtonClicked() = 0;

private:
    void onAddButtonClicked();
    void onDeleteButtonClicked();
    void enableDetails(bool on);
};

class DataPortConnectionDialogExt2 : public PortConnectionDialogBaseExt2
{
public:
    DataPortConnectionDialogExt2();
    void setDisp(RTSPortExt2* source, RTSPortExt2* target);

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

class ServicePortConnectionDialogExt2 : public PortConnectionDialogBaseExt2
{
public:
    ServicePortConnectionDialogExt2();
    void setDisp(RTSPortExt2* source, RTSPortExt2* target);

private:
    QTableWidget* lstInterface;
    std::vector<std::string> consumerLabelList;
    std::vector<std::string> consumerConList;
    std::vector<std::string> providerLabelList;
    std::vector<std::string> providerConList;

    void registInterfaceMap(RTSPortExt2* port);
    void addIF();
    void deleteIF();
    void onOkButtonClicked() override;
};

}

#endif
