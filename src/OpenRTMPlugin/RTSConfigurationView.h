/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_CONFIGURATION_VIEW_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_CONFIGURATION_VIEW_H_INCLUDED

#include <QtWidgets>
#include <cnoid/View>

#include "RTSNameServerView.h"

using namespace std;
using namespace RTC;

namespace cnoid {
class RTSConfigurationView;
class ConfigurationParam;

enum ParamMode {
	MODE_NORMAL = 1,
	MODE_UPDATE,
	MODE_DELETE,
	MODE_INSERT,
	MODE_IGNORE
};

typedef std::shared_ptr<ConfigurationParam> ConfigurationParamPtr;

class ConfigurationParam {
public:
	ConfigurationParam(int id, QString name, QString value)
		: id_(id), name_(name), nameOrg_(name), value_(value), valueOrg_(value), mode_(MODE_NORMAL){
	}
	ConfigurationParam(const ConfigurationParamPtr source)
		: id_(source->id_), name_(source->name_), nameOrg_(source->nameOrg_),
		  value_(source->value_), valueOrg_(source->valueOrg_), mode_(MODE_NORMAL) {
	}

	inline ParamMode getMode() const { return this->mode_; }

	inline int getId() const { return this->id_; }
	inline void setId(int value) { this->id_ = value; }

	inline QString getName() const { return this->name_; }
	inline void setName(QString value) { this->name_ = value; }

	inline QString getValue() const { return this->value_; }
	inline void setValue(QString value) { this->value_ = value; }

	inline bool isChangedName() {
		if (name_ != nameOrg_ || mode_== ParamMode::MODE_INSERT) return true;
		return false;
	}
	inline bool isChangedValue() {
		if (value_ != valueOrg_ || mode_ == ParamMode::MODE_INSERT) return true;
		return false;
	}

	inline void setDelete() {
		if (mode_ == ParamMode::MODE_NORMAL || mode_ == ParamMode::MODE_UPDATE) {
			mode_ = ParamMode::MODE_DELETE;

		} else if (mode_ == ParamMode::MODE_INSERT) {
			mode_ = ParamMode::MODE_IGNORE;
		}
	}
	inline void setNew() {
		if (this->mode_ == ParamMode::MODE_NORMAL) {
			this->mode_ = ParamMode::MODE_INSERT;
		}
	}

private:
	ParamMode mode_;
	int id_;
	QString name_;
	QString value_;
	QString nameOrg_;
	QString valueOrg_;
};

class ConfigurationSetParam {
public:
	ConfigurationSetParam(int id, QString name)
		: id_(id), name_(name), nameOrg_(name), active_(false), mode_(MODE_NORMAL) {
	}

	inline int getId() const { return this->id_; }
	inline void setId(int value) { this->id_ = value; }

	inline QString getName() const { return this->name_; }
	inline void setName(QString value) { this->name_ = value; }

	inline QString getNameOrg() const { return this->nameOrg_; }

	inline void setDelete() {
		if (mode_ == ParamMode::MODE_NORMAL || mode_ == ParamMode::MODE_UPDATE) {
			mode_ = ParamMode::MODE_DELETE;

		} else if (mode_ == ParamMode::MODE_INSERT) {
			mode_ = ParamMode::MODE_IGNORE;
		}
	}
	inline void setNew() {
		if (this->mode_ == ParamMode::MODE_NORMAL) {
			this->mode_ = ParamMode::MODE_INSERT;
		}
	}

	inline bool getActive() const { return this->active_; }
	inline void setActive(bool value) {
		this->active_ = value;
		this->activeOrg_ = value;
	}

	inline void setRadio(QRadioButton* value) { this->radioActive_ = value; }

	inline int getRowCount() const { return this->rowCount_; }
	inline void setRowCount(int value) { this->rowCount_ = value; }

	inline int getMode() const { return this->mode_; }
	inline void setChanged() {
		if (mode_ == ParamMode::MODE_NORMAL) {
			mode_ = ParamMode::MODE_UPDATE;
		}
	}

	inline std::vector<ConfigurationParamPtr> getConfigurationList() const { return this->configurationList_; }
	inline void addConfiguration(ConfigurationParamPtr target) { this->configurationList_.push_back(target); }

	inline void updateActive() {
		if (radioActive_) {
			active_ = radioActive_->isChecked();
		}
	}
	inline bool isChangedActive() {
		if (active_ != activeOrg_) return true;
		return false;
	}
	inline bool isChangedName() {
		if (name_ != nameOrg_ || mode_ == ParamMode::MODE_INSERT) return true;
		return false;
	}

private:
	int id_;
	ParamMode mode_;
	QString name_;
	QString nameOrg_;
	bool active_;
	bool activeOrg_;

	QRadioButton* radioActive_;
	int rowCount_;

	std::vector<ConfigurationParamPtr> configurationList_;
};
typedef std::shared_ptr<ConfigurationSetParam> ConfigurationSetParamPtr;

class RTSConfigurationViewImpl : public QWidget {
	Q_OBJECT
public:
	RTSConfigurationViewImpl(RTSConfigurationView* self);
	~RTSConfigurationViewImpl();

	RTSConfigurationView* self;
	Connection selectionChangedConnection;
	Connection locationChangedConnection;

	void onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items);
	void onLocationChanged(std::string host, int port);

	void updateConfigSet();
	void updateDetail();

	void updateConfigurationSet();

private Q_SLOTS:
	void configSetSelectionChanged();
	void activeSetChanged();
	void setCopyClicked();
	void setAddClicked();
	void setDeleteClicked();
	void setDetailClicked();

	void addClicked();
	void deleteClicked();
	void detailClicked();

	void applyClicked();
	void cancelClicked();

private:
	NamingContextHelper::ObjectInfo currentItem_;
	RTCWrapperPtr currentRtc_;

	QLineEdit* txtCompName_;
	QTableWidget* lstConfigSet_;
	QCheckBox* chkSetDetail_;

	QLineEdit* txtConfSetName_;
	QTableWidget* lstDetail_;
	QCheckBox* chkDetail_;

	std::vector<ConfigurationSetParamPtr> configSetList_;
	ConfigurationSetParamPtr currentSet_;

	void getConfigurationSet();
	void showConfigurationSetView();
	void showConfigurationView();
};

class DetailDelegate : public QStyledItemDelegate {
	Q_OBJECT
public:
	DetailDelegate(RTSConfigurationViewImpl* view, QWidget *parent = 0);
	virtual void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;
private:
	RTSConfigurationViewImpl* view_;
};

class ConfigSetDelegate : public QStyledItemDelegate {
	Q_OBJECT

public:
	ConfigSetDelegate(RTSConfigurationViewImpl* view, QWidget *parent = 0);
	virtual void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;
private:
	RTSConfigurationViewImpl* view_;
};

class RTSConfigurationView : public cnoid::View {
public:
  static void initializeClass(ExtensionManager* ext);
  static RTSConfigurationView* instance();

	RTSConfigurationView();
  ~RTSConfigurationView();

	void updateConfigurationSet() {
		impl->updateConfigurationSet();
	}

private:
	RTSConfigurationViewImpl* impl;
};

}
#endif
