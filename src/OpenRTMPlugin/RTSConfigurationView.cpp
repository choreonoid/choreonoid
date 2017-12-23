/*!
* @brief  This is a definition of RTSystemEditorPlugin.
* @author
* @file
*/
#include "RTSConfigurationView.h"

#include <cnoid/ViewManager>
#include <rtm/NVUtil.h>
#include <rtm/CORBA_SeqUtil.h>
#include <coil/Properties.h>

#include "LoggerUtil.h"
#include "gettext.h"

using namespace std::placeholders;
using namespace cnoid;

namespace cnoid {

	struct ConfigurationComparator {
		int id_;
		ConfigurationComparator(int value) {
			id_ = value;
		}
		bool operator()(const ConfigurationParamPtr elem) const {
			return elem->getId() == id_;
		}
	};

	struct ConfigurationSetComparator {
		int id_;
		ConfigurationSetComparator(int value) {
			id_ = value;
		}
		bool operator()(const ConfigurationSetParamPtr elem) const {
			return elem->getId() == id_;
		}
	};
	//////////
	DetailDelegate::DetailDelegate(RTSConfigurationViewImpl* view, QWidget *parent) : QStyledItemDelegate(parent) {
		this->view_ = view;
	}

	void DetailDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const {
		DDEBUG("DetailDelegate::setModelData");
		QStyledItemDelegate::setModelData(editor, model, index);
		view_->updateDetail();
	}
	//////////
	ConfigSetDelegate::ConfigSetDelegate(RTSConfigurationViewImpl* view, QWidget *parent) : QStyledItemDelegate(parent) {
		this->view_ = view;
	}
	void ConfigSetDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const {
		DDEBUG("ConfigSetDelegate::setModelData");
		QStyledItemDelegate::setModelData(editor, model, index);
		view_->updateConfigSet();
	}
	//////////
	RTSConfigurationViewImpl::RTSConfigurationViewImpl(RTSConfigurationView* self)
		: self(self) {

		QLabel* lblCompName = new QLabel(_("Component Name : "));
		txtCompName_ = new QLineEdit;
		txtCompName_->setReadOnly(true);

		QFrame* frmCompName = new QFrame;
		QHBoxLayout* compNameLayout = new QHBoxLayout(frmCompName);
		compNameLayout->addWidget(lblCompName);
		compNameLayout->addWidget(txtCompName_);
		compNameLayout->setMargin(0);

		lstConfigSet_ = new QTableWidget(0, 2);
		lstConfigSet_->setSelectionBehavior(QAbstractItemView::SelectRows);
		lstConfigSet_->setSelectionMode(QAbstractItemView::SingleSelection);
		lstConfigSet_->verticalHeader()->setVisible(false);
		lstConfigSet_->setColumnWidth(0, 50);
		lstConfigSet_->setRowCount(0);
		lstConfigSet_->setHorizontalHeaderLabels(QStringList() << "Active" << "Config");
		lstConfigSet_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
		lstConfigSet_->setItemDelegate(new ConfigSetDelegate(this));

		chkSetDetail_ = new QCheckBox(_("Detail"));
		QPushButton* btnSetCopy = new QPushButton(_("Copy"));
		QPushButton* btnSetAdd = new QPushButton(_("Add"));
		QPushButton* btnSetDelete = new QPushButton(_("Delete"));
		QFrame* frmSetButtons = new QFrame;
		QHBoxLayout* setBtnLayout = new QHBoxLayout(frmSetButtons);
		setBtnLayout->addWidget(btnSetCopy);
		setBtnLayout->addStretch();
		setBtnLayout->addWidget(btnSetAdd);
		setBtnLayout->addWidget(btnSetDelete);
		setBtnLayout->addWidget(chkSetDetail_);
		setBtnLayout->setMargin(1);

		QFrame* frmConfSet = new QFrame;
		QVBoxLayout* confSetLayout = new QVBoxLayout(frmConfSet);
		confSetLayout->addWidget(frmCompName);
		confSetLayout->addWidget(lstConfigSet_);
		confSetLayout->addWidget(frmSetButtons);
		confSetLayout->setMargin(3);
		/////
		QLabel* lblConfSet = new QLabel(_("Configuration Set : "));
		txtConfSetName_ = new QLineEdit;
		txtConfSetName_->setReadOnly(true);

		QFrame* frmConfig = new QFrame;
		QHBoxLayout* configLayout = new QHBoxLayout(frmConfig);
		configLayout->addWidget(lblConfSet);
		configLayout->addWidget(txtConfSetName_);
		configLayout->setMargin(0);

		lstDetail_ = new QTableWidget(0, 2);
		lstDetail_->setSelectionBehavior(QAbstractItemView::SelectRows);
		lstDetail_->setSelectionMode(QAbstractItemView::SingleSelection);
		lstDetail_->verticalHeader()->setVisible(false);
		lstDetail_->setRowCount(0);
		lstDetail_->setHorizontalHeaderLabels(QStringList() << "Name" << "Value");
		lstDetail_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
		lstDetail_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
		lstDetail_->setItemDelegate(new DetailDelegate(this));

		chkDetail_ = new QCheckBox(_("Detail"));
		QPushButton* btnAdd = new QPushButton(_("Add"));
		btnAdd->setEnabled(false);
		QPushButton* btnDelete = new QPushButton(_("Delete"));
		btnDelete->setEnabled(false);
		QFrame* frmButtons = new QFrame;
		QHBoxLayout* btnLayout = new QHBoxLayout(frmButtons);
		btnLayout->addStretch();
		btnLayout->addWidget(btnAdd);
		btnLayout->addWidget(btnDelete);
		btnLayout->addWidget(chkDetail_);
		btnLayout->setMargin(1);

		QFrame* frmDetail = new QFrame;
		QVBoxLayout* detailLayout = new QVBoxLayout(frmDetail);
		detailLayout->addWidget(frmConfig);
		detailLayout->addWidget(lstDetail_);
		detailLayout->addWidget(frmButtons);
		detailLayout->setMargin(3);
		/////
		QSplitter* splitter = new QSplitter(Qt::Orientation::Horizontal);
		splitter->addWidget(frmConfSet);
		splitter->addWidget(frmDetail);

		QPushButton* btnApply = new QPushButton(_("Apply"));
		QPushButton* btnCancel = new QPushButton(_("Cancel"));
		QFrame* frmMainButtons = new QFrame;
		QVBoxLayout* mainBtnLayout = new QVBoxLayout(frmMainButtons);
		mainBtnLayout->addWidget(btnApply);
		mainBtnLayout->addWidget(btnCancel);
		mainBtnLayout->addStretch();

		QHBoxLayout* mainLayout = new QHBoxLayout;
		mainLayout->addWidget(splitter);
		mainLayout->addWidget(frmMainButtons);
		mainLayout->setMargin(0);

		setLayout(mainLayout);
		/////
		RTSNameServerView* nsView = RTSNameServerView::instance();
		if (nsView) {
			if (!selectionChangedConnection.connected()) {
				selectionChangedConnection = nsView->sigSelectionChanged().connect(
					std::bind(&RTSConfigurationViewImpl::onItemSelectionChanged, this, _1));
			}
			if (!locationChangedConnection.connected()) {
				locationChangedConnection = nsView->sigLocationChanged().connect(
					std::bind(&RTSConfigurationViewImpl::onLocationChanged, this, _1, _2));
				ncHelper_.setLocation(nsView->getHost(), nsView->getPort());
			}
		}

		connect(lstConfigSet_, SIGNAL(itemSelectionChanged()), this, SLOT(configSetSelectionChanged()));
		connect(btnSetCopy, SIGNAL(clicked()), this, SLOT(setCopyClicked()));
		connect(btnSetAdd, SIGNAL(clicked()), this, SLOT(setAddClicked()));
		connect(btnSetDelete, SIGNAL(clicked()), this, SLOT(setDeleteClicked()));
		connect(chkSetDetail_, SIGNAL(clicked()), this, SLOT(setDetailClicked()));

		connect(chkDetail_, SIGNAL(clicked()), this, SLOT(detailClicked()));
		connect(btnAdd, SIGNAL(clicked()), this, SLOT(addClicked()));
		connect(btnDelete, SIGNAL(clicked()), this, SLOT(deleteClicked()));
		connect(btnApply, SIGNAL(clicked()), this, SLOT(applyClicked()));
		connect(btnCancel, SIGNAL(clicked()), this, SLOT(cancelClicked()));
	}

	RTSConfigurationViewImpl::~RTSConfigurationViewImpl() {
		selectionChangedConnection.disconnect();
		locationChangedConnection.disconnect();
	}

	void RTSConfigurationViewImpl::onLocationChanged(string host, int port) {
		ncHelper_.setLocation(host, port);
	}

	void RTSConfigurationViewImpl::onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items) {
		DDEBUG("RTSConfigurationViewImpl::onItemSelectionChanged");

		lstConfigSet_->clear();
		lstConfigSet_->setRowCount(0);
		lstConfigSet_->setHorizontalHeaderLabels(QStringList() << "Active" << "Config");
		lstDetail_->clear();
		lstDetail_->setRowCount(0);
		lstDetail_->setHorizontalHeaderLabels(QStringList() << "Name" << "Value");

		if (items.size() != 1) return;

		const NamingContextHelper::ObjectInfo& item = items.front();
		currentItem_.id = item.id;
		currentItem_.isAlive = item.isAlive;
		currentItem_.kind = item.kind;
		currentItem_.fullPath = item.fullPath;
		updateConfigurationSet();
	}

	void RTSConfigurationViewImpl::configSetSelectionChanged() {
		DDEBUG("RTSConfigurationViewImpl::configSetSelectionChanged");

		int selectedId = -1;
		QTableWidgetItem* item = lstConfigSet_->currentItem();
		if (item) {
			selectedId = item->data(Qt::UserRole).toInt();
		}
		if (selectedId == -1) return;

		vector<ConfigurationSetParamPtr>::iterator targetConf = find_if(configSetList_.begin(), configSetList_.end(), ConfigurationSetComparator(selectedId));
		if (targetConf == configSetList_.end()) return;

		currentSet_ = *targetConf;
		txtConfSetName_->setText(currentSet_->getName());

		showConfigurationView();
	}

	void RTSConfigurationViewImpl::addClicked() {
		if (!currentSet_) return;

		int maxId = 0;
		for (int index = 0; index < currentSet_->getConfigurationList().size(); index++) {
			ConfigurationParamPtr param = currentSet_->getConfigurationList()[index];
			if (maxId < param->getId()) {
				maxId = param->getId();
			}
		}
		ConfigurationParamPtr param = std::make_shared<ConfigurationParam>(maxId, "New Config", "New Value");
		param->setNew();
		currentSet_->addConfiguration(param);

		showConfigurationView();
	}

	void RTSConfigurationViewImpl::deleteClicked() {
		if (!currentSet_) return;

		int selectedId = -1;
		QTableWidgetItem* item = lstDetail_->currentItem();
		if (item) {
			selectedId = item->data(Qt::UserRole).toInt();
		}
		if (selectedId == -1) return;

		vector<ConfigurationParamPtr>::iterator targetConf = find_if(currentSet_->getConfigurationList().begin(), currentSet_->getConfigurationList().end(), ConfigurationComparator(selectedId));
		if (targetConf == currentSet_->getConfigurationList().end()) return;

		(*targetConf)->setDelete();

		showConfigurationView();
	}

	void RTSConfigurationViewImpl::detailClicked() {
		showConfigurationView();
	}

	void RTSConfigurationViewImpl::showConfigurationView() {
		lstDetail_->clear();
		lstDetail_->setRowCount(0);
		lstDetail_->setHorizontalHeaderLabels(QStringList() << "Name" << "Value");

		vector<ConfigurationParamPtr> configList = currentSet_->getConfigurationList();

		for (int index = 0; index < configList.size(); index++) {
			ConfigurationParamPtr param = configList[index];
			if (param->getMode() == MODE_DELETE || param->getMode() == MODE_IGNORE) continue;
			if (chkDetail_->isChecked() == false) {
				if (param->getName().startsWith(QString::fromStdString("__"))) continue;
			}

			int row = lstDetail_->rowCount();
			lstDetail_->insertRow(row);

			QTableWidgetItem* itemName = new QTableWidgetItem;
			lstDetail_->setItem(row, 0, itemName);
			itemName->setText(param->getName());
			itemName->setData(Qt::UserRole, param->getId());
			if (param->isChangedName()) {
				lstDetail_->item(row, 0)->setBackgroundColor("#FFC0C0");
			} else {
				lstDetail_->item(row, 0)->setBackgroundColor(Qt::white);
			}

			QTableWidgetItem* itemValue = new QTableWidgetItem;
			lstDetail_->setItem(row, 1, itemValue);
			itemValue->setText(param->getValue());
			itemValue->setData(Qt::UserRole, param->getId());
			if (param->isChangedValue()) {
				lstDetail_->item(row, 1)->setBackgroundColor("#FFC0C0");
			} else {
				lstDetail_->item(row, 1)->setBackgroundColor(Qt::white);
			}
		}
	}

	void RTSConfigurationViewImpl::updateConfigurationSet() {
		DDEBUG("RTSConfigurationViewImpl::updateConfigurationSet");

		if (currentItem_.id != "" && currentItem_.isAlive) {
			getConfigurationSet();
			showConfigurationSetView();
		}
	}

	void RTSConfigurationViewImpl::setCopyClicked() {
		if (!currentSet_) return;

		int maxId = 0;
		for (int index = 0; index < configSetList_.size(); index++) {
			ConfigurationSetParamPtr param = configSetList_[index];
			if (maxId < param->getId()) {
				maxId = param->getId();
			}
		}
		ConfigurationSetParamPtr configSet = std::make_shared<ConfigurationSetParam>(maxId + 1, currentSet_->getName()+"_copy");
		configSet->setNew();
		configSetList_.push_back(configSet);

		std::vector<ConfigurationParamPtr> configList = currentSet_->getConfigurationList();
		for (int index = 0; index < configList.size(); index++) {
			ConfigurationParamPtr param = configList[index];
			ConfigurationParamPtr newParam = std::make_shared<ConfigurationParam>(param);
			newParam->setNew();
			configSet->addConfiguration(newParam);
		}
		showConfigurationSetView();
	}

	void RTSConfigurationViewImpl::setAddClicked() {
		int maxId = 0;
		for (int index = 0; index < configSetList_.size(); index++) {
			ConfigurationSetParamPtr param = configSetList_[index];
			if (maxId < param->getId()) {
				maxId = param->getId();
			}
		}
		QString name = QString::fromStdString("configSet_") + QString::number(maxId);
		ConfigurationSetParamPtr configSet = std::make_shared<ConfigurationSetParam>(maxId + 1, name);
		configSet->setNew();
		configSetList_.push_back(configSet);

		if (currentSet_) {
			std::vector<ConfigurationParamPtr> configList = currentSet_->getConfigurationList();
			for (int index = 0; index < configList.size(); index++) {
				ConfigurationParamPtr param = configList[index];
				ConfigurationParamPtr newParam = std::make_shared<ConfigurationParam>(param->getId(), param->getName(), "value");
				newParam->setNew();
				configSet->addConfiguration(newParam);
			}
		}
		showConfigurationSetView();
	}

	void RTSConfigurationViewImpl::setDeleteClicked() {
		if (!currentSet_) return;

		currentSet_->setDelete();
		showConfigurationSetView();
	}

	void RTSConfigurationViewImpl::setDetailClicked() {
		showConfigurationSetView();
	}

	void RTSConfigurationViewImpl::showConfigurationSetView() {
		lstConfigSet_->clear();
		lstConfigSet_->setRowCount(0);
		lstConfigSet_->setHorizontalHeaderLabels(QStringList() << "Active" << "Config");

		QButtonGroup* radioGrp = new QButtonGroup;

		for (int index = 0; index < configSetList_.size(); index++) {
			ConfigurationSetParamPtr param = configSetList_[index];
			if (param->getMode() == MODE_DELETE || param->getMode() == MODE_IGNORE) continue;
			if (chkSetDetail_->isChecked() == false) {
				if (param->getName().startsWith(QString::fromStdString("__"))) continue;
			}

			int row = lstConfigSet_->rowCount();
			lstConfigSet_->insertRow(row);

			QRadioButton* radioActive = new QRadioButton;
			connect(radioActive, SIGNAL(clicked()), this, SLOT(activeSetChanged()));
			param->setRadio(radioActive);
			param->setRowCount(row);
			radioGrp->addButton(radioActive);
			QFrame* frmRadio = new QFrame;
			QHBoxLayout* radioLayout = new QHBoxLayout(frmRadio);
			radioLayout->addWidget(radioActive);
			radioLayout->setAlignment(Qt::AlignCenter);

			lstConfigSet_->setCellWidget(row, 0, frmRadio);
			if (param->getActive()) {
				radioActive->setChecked(true);
			} else {
				radioActive->setChecked(false);
			}

			QTableWidgetItem* itemDummy = new QTableWidgetItem;
			lstConfigSet_->setItem(row, 0, itemDummy);
			itemDummy->setData(Qt::UserRole, param->getId());

			QTableWidgetItem* itemName = new QTableWidgetItem;
			lstConfigSet_->setItem(row, 1, itemName);
			itemName->setText(param->getName());
			itemName->setData(Qt::UserRole, param->getId());
			if (param->isChangedName()) {
				lstConfigSet_->item(row, 1)->setBackgroundColor("#FFC0C0");
			} else {
				lstConfigSet_->item(row, 1)->setBackgroundColor(Qt::white);
			}
		}

		if (0 < configSetList_.size()) {
			currentSet_ = configSetList_[0];
			txtConfSetName_->setText(currentSet_->getName());
			showConfigurationView();
		}
	}

	void RTSConfigurationViewImpl::getConfigurationSet() {
		DDEBUG("RTSConfigurationViewImpl::getConfigurationSet");
		configSetList_.clear();

		RTC::RTObject_ptr rtc;
		if (currentItem_.fullPath.size() == 0) {
			rtc = ncHelper_.findObject<RTC::RTObject>(currentItem_.id, "rtc");
		} else {
			CORBA::Object::_ptr_type obj = ncHelper_.findObject(currentItem_.fullPath);
			if (CORBA::is_nil(obj)) {
				rtc = RTC::RTObject::_nil();
			} else {
				rtc = RTC::RTObject::_narrow(obj);
				CORBA::release(obj);
			}
		}
		if (!ncHelper_.isObjectAlive(rtc)) return;

		ComponentProfile_var cprofile = rtc->get_component_profile();
		txtCompName_->setText(QString(string(cprofile->instance_name).c_str()));

		currentConf_ = rtc->get_configuration();
		QString activeName;
		try {
			SDOPackage::ConfigurationSet* activeConf = currentConf_->get_active_configuration_set();
			if (activeConf) {
				activeName = QString(string(activeConf->id).c_str());
			}
		} catch (...) {
		}

		SDOPackage::ConfigurationSetList_var confSet = currentConf_->get_configuration_sets();
		int setNum = confSet->length();
		for (int index = 0; index < setNum; index++) {
			SDOPackage::ConfigurationSet conf = confSet[index];
			QString name = QString(string(conf.id).c_str());
			ConfigurationSetParamPtr configSet = std::make_shared<ConfigurationSetParam>(index + 1, name);
			if (name == activeName) configSet->setActive(true);
			configSetList_.push_back(configSet);
			//
			coil::Properties confs = NVUtil::toProperties(conf.configuration_data);
			vector<string> confNames = confs.propertyNames();
			int id = 1;
			for (vector<string>::iterator iter = confNames.begin(); iter != confNames.end(); iter++) {
				QString name = QString((*iter).c_str());
				QString value = QString((confs[*iter]).c_str());
				ConfigurationParamPtr param = std::make_shared<ConfigurationParam>(id, name, value);
				configSet->addConfiguration(param);
				id++;
			}
		}
	}

	void RTSConfigurationViewImpl::applyClicked() {
		DDEBUG("RTSConfigurationViewImpl::applyClicked");
		QString activeName;
		int selectedSet = lstConfigSet_->currentRow();
		int selected = lstDetail_->currentRow();

		for (int index = 0; index < configSetList_.size(); index++) {
			ConfigurationSetParamPtr target = configSetList_[index];
			if (target->getActive()) {
				activeName = target->getName();
			}

			if (target->getMode() == ParamMode::MODE_INSERT) {
				DDEBUG("ConfigurationSet MODE_INSERT");
				SDOPackage::ConfigurationSet newSet;
				newSet.id = CORBA::string_dup(target->getName().toStdString().c_str());
				NVList configList;
				std::vector<ConfigurationParamPtr> configSetList = target->getConfigurationList();
				for (int idxDetail = 0; idxDetail < configSetList.size(); idxDetail++) {
					ConfigurationParamPtr param = configSetList[idxDetail];
					if (param->getMode() == MODE_DELETE || param->getMode() == MODE_IGNORE) continue;
					DDEBUG_V("id:%d", param->getId());
					CORBA_SeqUtil::push_back(configList,
						NVUtil::newNV(param->getName().toStdString().c_str(), param->getValue().toStdString().c_str()));
				}
				newSet.configuration_data = configList;
				currentConf_->add_configuration_set(newSet);

			} else if (target->getMode() == ParamMode::MODE_UPDATE) {
				DDEBUG("ConfigurationSet MODE_UPDATE");
				SDOPackage::ConfigurationSetList_var confSet = currentConf_->get_configuration_sets();
				for (int idxConf = 0; idxConf < confSet->length(); idxConf++) {
					SDOPackage::ConfigurationSet conf = confSet[idxConf];
					QString name = QString(string(conf.id).c_str());
					if (name == target->getNameOrg()) {
						conf.id = CORBA::string_dup(target->getName().toStdString().c_str());
						NVList configList;
						std::vector<ConfigurationParamPtr> configSetList = target->getConfigurationList();
						for (int idxDetail = 0; idxDetail < configSetList.size(); idxDetail++) {
							ConfigurationParamPtr param = configSetList[idxDetail];
							if (param->getMode() == MODE_DELETE || param->getMode() == MODE_IGNORE) continue;
							DDEBUG_V("id:%d", param->getId());
							CORBA_SeqUtil::push_back(configList,
								NVUtil::newNV(param->getName().toStdString().c_str(), param->getValue().toStdString().c_str()));
						}
						conf.configuration_data = configList;
						currentConf_->set_configuration_set_values(conf);

					}
				}

			} else if (target->getMode() == ParamMode::MODE_DELETE) {
				DDEBUG("ConfigurationSet MODE_DELETE");
				currentConf_->remove_configuration_set(CORBA::string_dup(target->getName().toStdString().c_str()));
			}
		}
		currentConf_->activate_configuration_set(activeName.toStdString().c_str());
		//
		getConfigurationSet();
		showConfigurationSetView();

		if (selectedSet < lstConfigSet_->rowCount()) {
			lstConfigSet_->setCurrentCell(selectedSet, 0);
		}
		if (selected < lstDetail_->rowCount()) {
			lstDetail_->setCurrentCell(selected, 0);
		}
	}

	void RTSConfigurationViewImpl::cancelClicked() {
		int selectedSet = lstConfigSet_->currentRow();
		int selected = lstDetail_->currentRow();

		getConfigurationSet();
		showConfigurationSetView();

		lstConfigSet_->setCurrentCell(selectedSet, 0);
		lstDetail_->setCurrentCell(selected, 0);
	}

	void RTSConfigurationViewImpl::updateConfigSet() {
		DDEBUG("RTSConfigurationViewImpl::updateConfigSet");

		int selectedId = -1;
		QTableWidgetItem* item = lstConfigSet_->currentItem();
		if (item) {
			selectedId = item->data(Qt::UserRole).toInt();
		}
		if (selectedId == -1) return;

		vector<ConfigurationSetParamPtr>::iterator targetConf = find_if(configSetList_.begin(), configSetList_.end(), ConfigurationSetComparator(selectedId));
		if (targetConf == configSetList_.end()) return;

		int row = lstConfigSet_->currentRow();
		(*targetConf)->setName(lstConfigSet_->item(row, 1)->text());
		(*targetConf)->setChanged();

		if ((*targetConf)->isChangedName()) {
			lstConfigSet_->item(row, 1)->setBackgroundColor("#FFC0C0");
		} else {
			lstConfigSet_->item(row, 1)->setBackgroundColor(Qt::white);
		}
	}

	void RTSConfigurationViewImpl::activeSetChanged() {
		DDEBUG("RTSConfigurationViewImpl::activeSetChanged");
		for (int index = 0; index < configSetList_.size(); index++) {
			configSetList_[index]->updateActive();
			int row = configSetList_[index]->getRowCount();
			DDEBUG_V("index:%d, row:%d", index, row);
			if (configSetList_[index]->isChangedActive()) {
				lstConfigSet_->item(row, 0)->setBackgroundColor("#FFC0C0");
			} else {
				lstConfigSet_->item(row, 0)->setBackgroundColor(Qt::white);
			}
		}
	}

	void RTSConfigurationViewImpl::updateDetail() {
		DDEBUG("RTSConfigurationViewImpl::updateDetail");

		int selectedId = -1;
		QTableWidgetItem* item = lstDetail_->currentItem();
		if (item) {
			selectedId = item->data(Qt::UserRole).toInt();
		}
		if (selectedId == -1) return;

		vector<ConfigurationParamPtr> configList = currentSet_->getConfigurationList();
		vector<ConfigurationParamPtr>::iterator targetConf = find_if(configList.begin(), configList.end(), ConfigurationComparator(selectedId));
		if (targetConf == configList.end()) return;
		//
		int row = lstDetail_->currentRow();
		(*targetConf)->setName(lstDetail_->item(row, 0)->text());
		(*targetConf)->setValue(lstDetail_->item(row, 1)->text());
		currentSet_->setChanged();

		if ((*targetConf)->isChangedName()) {
			lstDetail_->item(row, 0)->setBackgroundColor("#FFC0C0");
		} else {
			lstDetail_->item(row, 0)->setBackgroundColor(Qt::white);
		}
		if ((*targetConf)->isChangedValue()) {
			lstDetail_->item(row, 1)->setBackgroundColor("#FFC0C0");
		} else {
			lstDetail_->item(row, 1)->setBackgroundColor(Qt::white);
		}
	}
	//////////
	void RTSConfigurationView::initializeClass(ExtensionManager* ext) {
		ext->viewManager().registerClass<RTSConfigurationView>(
			"RTSConfigurationView", N_("Configuration"), ViewManager::SINGLE_OPTIONAL);
	}

	RTSConfigurationView* RTSConfigurationView::instance() {
		return ViewManager::findView<RTSConfigurationView>();
	}

	RTSConfigurationView::RTSConfigurationView() {
		impl = new RTSConfigurationViewImpl(this);
		QVBoxLayout* vbox = new QVBoxLayout();
		vbox->addWidget(impl);
		setLayout(vbox);
	}

	RTSConfigurationView::~RTSConfigurationView() {
		delete impl;
	}

}