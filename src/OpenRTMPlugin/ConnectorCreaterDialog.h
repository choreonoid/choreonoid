/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_CONNECTOR_CREATOR_DIALOG_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_CONNECTOR_CREATOR_DIALOG_H_INCLUDED

#include <cnoid/Dialog>
#include <cnoid/ComboBox>
#include <cnoid/CheckBox>
#include <QLineEdit>
#include <QTableWidget>
#include <QSplitter>

#include "RTSItem.h"

using namespace cnoid;

namespace cnoid {
class ConnectorCreaterDialogBase : public Dialog {
	Q_OBJECT
public:
	ConnectorCreaterDialogBase();

	QLineEdit* nameLineEdit;
	bool isAccepted;
	std::vector<NamedValuePtr> propList;

private Q_SLOTS:
	void addClicked();
	void deleteClicked();
	void okClicked();

protected:
	CheckBox* detailCheck;
	QFrame* frmEditSub;
	QTableWidget* lstDetail;

	QSplitter* detailSplitter;

	QFrame* frmDetail;
	QFrame* frmButton;

	void detailCheckToggled(bool target);
	void addProperty(std::string name, std::string value);
	virtual void actionOkClicked() = 0;
};

class DataConnectorCreaterDialog : public ConnectorCreaterDialogBase {
public:
	DataConnectorCreaterDialog();
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
	void actionOkClicked();
};

class ServiceConnectorCreaterDialog : public ConnectorCreaterDialogBase {
	Q_OBJECT
public:
	ServiceConnectorCreaterDialog();
	void setDisp(RTSPort* source, RTSPort* target);

private Q_SLOTS:
	void addIFClicked();
	void deleteIFClicked();

private:
	QTableWidget* lstInterface;
	std::vector<std::string> consumerLabelList;
	std::vector<std::string> consumerConList;
	std::vector<std::string> providerLabelList;
	std::vector<std::string> providerConList;

	void actionOkClicked();
	void registInterfaceMap(RTSPort* port);
};

}

#endif 
