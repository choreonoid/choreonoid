#ifndef CNOID_OPENRTM_PLUGIN_PORT_CONNECTION_DIALOG_H
#define CNOID_OPENRTM_PLUGIN_PORT_CONNECTION_DIALOG_H

#include "RTSCommonUtil.h"
#include "RTCWrapper.h"
#include <cnoid/Dialog>
#include <cnoid/ComboBox>
#include <cnoid/CheckBox>
#include <QLineEdit>
#include <QTableWidget>

namespace cnoid {

class PortConnectionDialogBase : public Dialog
{
public:
    QLineEdit* nameLineEdit;
    std::vector<NamedValuePtr> propList;
    bool isAccepted;

protected:
    CheckBox* detailCheck;
    QFrame* frmEditSub;
    QTableWidget* lstDetail;
    QFrame* frmDetail;
    QFrame* buttonFrame;

    PortConnectionDialogBase();
    void addProperty(std::string name, std::string value);
    virtual void onOkButtonClicked() = 0;

private:
    void onAddButtonClicked();
    void onDeleteButtonClicked();
    void enableDetails(bool on);
};

class DataPortConnectionDialog : public PortConnectionDialogBase
{
public:
    DataPortConnectionDialog();
    void setDisp(RTSPort* source, RTSPort* target);

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

class ServicePortConnectionDialog : public PortConnectionDialogBase
{
public:
    ServicePortConnectionDialog();
    void setDisp(RTSPort* source, RTSPort* target);

private:
    QTableWidget* lstInterface;
    std::vector<std::string> consumerLabelList;
    std::vector<std::string> consumerConList;
    std::vector<std::string> providerLabelList;
    std::vector<std::string> providerConList;

    void registInterfaceMap(RTSPort* port);
    void addIF();
    void deleteIF();
    void onOkButtonClicked() override;
};

}

#endif
