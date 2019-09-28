
#include "RTSConfigurationView.h"
#include "RTSNameServerView.h"
#include "LoggerUtil.h"
#include <cnoid/ViewManager>
#include <cnoid/Buttons>
#include <QRadioButton>
#include <QBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QHeaderView>
#include <QPushButton>
#include <QSplitter>
#include <QButtonGroup>
#include "gettext.h"

using namespace cnoid;

namespace {

struct ConfigurationComparator
{
    int id_;
    ConfigurationComparator(int value)
    {
        id_ = value;
    }
    bool operator()(const ConfigurationParamPtr elem) const
    {
        return elem->getId() == id_;
    }
};

struct ConfigurationSetComparator
{
    int id_;
    ConfigurationSetComparator(int value)
    {
        id_ = value;
    }
    bool operator()(const ConfigurationSetParamPtr elem) const
    {
        return elem->getId() == id_;
    }
};

}


ConfigurationSetParam::ConfigurationSetParam(int id, QString name)
    : id_(id), name_(name), nameOrg_(name), active_(false), mode_(MODE_NORMAL)
{

}


void ConfigurationSetParam::setDelete()
{
    if (mode_ == ParamMode::MODE_NORMAL || mode_ == ParamMode::MODE_UPDATE) {
        mode_ = ParamMode::MODE_DELETE;

    } else if (mode_ == ParamMode::MODE_INSERT) {
        mode_ = ParamMode::MODE_IGNORE;
    }
}


void ConfigurationSetParam::setNew()
{
    if (mode_ == ParamMode::MODE_NORMAL) {
        mode_ = ParamMode::MODE_INSERT;
    }
}


void ConfigurationSetParam::setActive(bool value)
{
    active_ = value;
    activeOrg_ = value;
}


void ConfigurationSetParam::setChanged()
{
    if (mode_ == ParamMode::MODE_NORMAL) {
        mode_ = ParamMode::MODE_UPDATE;
    }
}


void ConfigurationSetParam::updateActive()
{
    if (radioActive_) {
        active_ = radioActive_->isChecked();
    }
}


DetailDelegate::DetailDelegate(RTSConfigurationViewImpl* view, QWidget *parent) : QStyledItemDelegate(parent)
{
    this->view_ = view;
}


void DetailDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    DDEBUG("DetailDelegate::setModelData");
    QStyledItemDelegate::setModelData(editor, model, index);
    view_->updateDetail();
}


ConfigSetDelegate::ConfigSetDelegate(RTSConfigurationViewImpl* view, QWidget *parent)
    : QStyledItemDelegate(parent)
{
    this->view_ = view;
}


void ConfigSetDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    DDEBUG("ConfigSetDelegate::setModelData");
    QStyledItemDelegate::setModelData(editor, model, index);
    view_->updateConfigSet();
}


void RTSConfigurationView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<RTSConfigurationView>(
        "RTSConfigurationView", N_("Configuration"), ViewManager::SINGLE_OPTIONAL);
}


RTSConfigurationView* RTSConfigurationView::instance()
{
    return ViewManager::findView<RTSConfigurationView>();
}


RTSConfigurationView::RTSConfigurationView()
{
    impl = new RTSConfigurationViewImpl(this);
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(impl);
    setLayout(vbox);
}


RTSConfigurationViewImpl::RTSConfigurationViewImpl(RTSConfigurationView* self)
    : self(self), currentRtc_(0)
{
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

    chkSetDetail_ = new CheckBox(_("Detail"));
    chkSetDetail_->sigToggled().connect([&](bool on){ setDetailClicked(); });

    PushButton* btnSetCopy = new PushButton(_("Copy"));
    btnSetCopy->sigClicked().connect([&](){ setCopyClicked(); });

    PushButton* btnSetAdd = new PushButton(_("Add"));
    btnSetAdd->sigClicked().connect([&](){ setAddClicked(); });

    PushButton* btnSetDelete = new PushButton(_("Delete"));
    btnSetDelete->sigClicked().connect([&](){ setDeleteClicked(); });

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

    chkDetail_ = new CheckBox(_("Detail"));
    chkDetail_->sigToggled().connect([&](bool on){ detailClicked(); });

    PushButton* btnAdd = new PushButton(_("Add"));
    btnAdd->setEnabled(false);
    btnAdd->sigClicked().connect([&](){ addClicked(); });

    PushButton* btnDelete = new PushButton(_("Delete"));
    btnDelete->setEnabled(false);
    btnAdd->sigClicked().connect([&](){ deleteClicked(); });

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

    QSplitter* splitter = new QSplitter(Qt::Orientation::Horizontal);
    splitter->addWidget(frmConfSet);
    splitter->addWidget(frmDetail);

    PushButton* btnApply = new PushButton(_("Apply"));
    btnApply->sigClicked().connect([&](){ applyClicked(); });

    PushButton* btnCancel = new PushButton(_("Cancel"));
    btnCancel->sigClicked().connect([&](){ cancelClicked(); });

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

    RTSNameServerView* nsView = RTSNameServerView::instance();
    if (nsView) {
        if (!selectionChangedConnection.connected()) {
            selectionChangedConnection = nsView->sigSelectionChanged().connect(
                [&](const std::list<NamingContextHelper::ObjectInfo>& items){
                    onItemSelectionChanged(items); });
        }
    }

    connect(lstConfigSet_, SIGNAL(itemSelectionChanged()), this, SLOT(configSetSelectionChanged()));
}


RTSConfigurationView::~RTSConfigurationView()
{
    delete impl;
}


RTSConfigurationViewImpl::~RTSConfigurationViewImpl()
{
    selectionChangedConnection.disconnect();
}


void RTSConfigurationView::updateConfigurationSet()
{
    impl->updateConfigurationSet();
}


void RTSConfigurationViewImpl::onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items)
{
    DDEBUG("RTSConfigurationViewImpl::onItemSelectionChanged");

    lstConfigSet_->clear();
    lstConfigSet_->setRowCount(0);
    lstConfigSet_->setHorizontalHeaderLabels(QStringList() << "Active" << "Config");
    lstDetail_->clear();
    lstDetail_->setRowCount(0);
    lstDetail_->setHorizontalHeaderLabels(QStringList() << "Name" << "Value");

    if (items.size() != 1) {
        return;
    }

    const NamingContextHelper::ObjectInfo& item = items.front();
    currentItem_.id_ = item.id_;
    currentItem_.isAlive_ = item.isAlive_;
    currentItem_.kind_ = item.kind_;
    currentItem_.fullPath_ = item.fullPath_;
    currentItem_.hostAddress_ = item.hostAddress_;
    currentItem_.portNo_ = item.portNo_;
    updateConfigurationSet();
}


void RTSConfigurationViewImpl::configSetSelectionChanged()
{
    DDEBUG("RTSConfigurationViewImpl::configSetSelectionChanged");

    int selectedId = -1;
    QTableWidgetItem* item = lstConfigSet_->currentItem();
    if (item) {
        selectedId = item->data(Qt::UserRole).toInt();
    }
    if (selectedId == -1) {
        return;
    }

    vector<ConfigurationSetParamPtr>::iterator targetConf =
        find_if(configSetList_.begin(), configSetList_.end(), ConfigurationSetComparator(selectedId));

    if (targetConf == configSetList_.end()) {
        return;
    }

    currentSet_ = *targetConf;
    txtConfSetName_->setText(currentSet_->getName());

    showConfigurationView();
}


void RTSConfigurationViewImpl::addClicked()
{
    if (!currentSet_) {
        return;
    }

    int maxId = 0;
    for (int index = 0; index < currentSet_->getConfigurationList().size(); ++index) {
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


void RTSConfigurationViewImpl::deleteClicked()
{
    if (!currentSet_) return;

    int selectedId = -1;
    QTableWidgetItem* item = lstDetail_->currentItem();
    if (item) {
        selectedId = item->data(Qt::UserRole).toInt();
    }
    if (selectedId == -1) {
        return;
    }

    vector<ConfigurationParamPtr>::iterator targetConf =
        find_if(currentSet_->getConfigurationList().begin(), currentSet_->getConfigurationList().end(), ConfigurationComparator(selectedId));
    if (targetConf == currentSet_->getConfigurationList().end()) {
        return;
    }

    (*targetConf)->setDelete();

    showConfigurationView();
}


void RTSConfigurationViewImpl::detailClicked()
{
    showConfigurationView();
}


void RTSConfigurationViewImpl::showConfigurationView()
{
    lstDetail_->clear();
    lstDetail_->setRowCount(0);
    lstDetail_->setHorizontalHeaderLabels(QStringList() << "Name" << "Value");

    vector<ConfigurationParamPtr> configList = currentSet_->getConfigurationList();

    for (int index = 0; index < configList.size(); ++index) {
        ConfigurationParamPtr param = configList[index];
        if (param->getMode() == MODE_DELETE || param->getMode() == MODE_IGNORE) {
            continue;
        }
        if (!chkDetail_->isChecked()) {
            if (param->getName().startsWith(QString::fromStdString("__"))) {
                continue;
            }
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


void RTSConfigurationViewImpl::updateConfigurationSet()
{
    DDEBUG("RTSConfigurationViewImpl::updateConfigurationSet");

    if (currentItem_.id_ != "" && currentItem_.isAlive_) {
        getConfigurationSet();
        showConfigurationSetView();
    }
}


void RTSConfigurationViewImpl::setCopyClicked()
{
    if (!currentSet_) return;

    int maxId = 0;
    for (int index = 0; index < configSetList_.size(); ++index) {
        ConfigurationSetParamPtr param = configSetList_[index];
        if (maxId < param->getId()) {
            maxId = param->getId();
        }
    }
    ConfigurationSetParamPtr configSet = std::make_shared<ConfigurationSetParam>(maxId + 1, currentSet_->getName() + "_copy");
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


void RTSConfigurationViewImpl::setAddClicked()
{
    int maxId = 0;
    for (int index = 0; index < configSetList_.size(); ++index) {
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
        for (int index = 0; index < configList.size(); ++index) {
            ConfigurationParamPtr param = configList[index];
            ConfigurationParamPtr newParam = std::make_shared<ConfigurationParam>(param->getId(), param->getName(), "value");
            newParam->setNew();
            configSet->addConfiguration(newParam);
        }
    }
    showConfigurationSetView();
}


void RTSConfigurationViewImpl::setDeleteClicked()
{
    if (!currentSet_) {
        return;
    }

    currentSet_->setDelete();
    showConfigurationSetView();
}


void RTSConfigurationViewImpl::setDetailClicked()
{
    showConfigurationSetView();
}


void RTSConfigurationViewImpl::showConfigurationSetView()
{
    lstConfigSet_->clear();
    lstConfigSet_->setRowCount(0);
    lstConfigSet_->setHorizontalHeaderLabels(QStringList() << "Active" << "Config");

    QButtonGroup* radioGrp = new QButtonGroup;

    for (int index = 0; index < configSetList_.size(); ++index) {
        ConfigurationSetParamPtr param = configSetList_[index];
        if (param->getMode() == MODE_DELETE || param->getMode() == MODE_IGNORE) {
            continue;
        }
        if (!chkSetDetail_->isChecked()) {
            if (param->getName().startsWith(QString::fromStdString("__"))) {
                continue;
            }
        }

        int row = lstConfigSet_->rowCount();
        lstConfigSet_->insertRow(row);

        QRadioButton* radioActive = new QRadioButton;
        connect(radioActive, SIGNAL(clicked()), this, SLOT(activeSetChanged()));
        param->setRadio(radioActive);
        radioGrp->addButton(radioActive);
        QFrame* frmRadio = new QFrame;
        QHBoxLayout* radioLayout = new QHBoxLayout(frmRadio);
        radioLayout->addWidget(radioActive);
        radioLayout->setAlignment(Qt::AlignCenter);

        lstConfigSet_->setCellWidget(row, 0, frmRadio);
        radioActive->setChecked(param->getActive());

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


void RTSConfigurationViewImpl::getConfigurationSet()
{
    DDEBUG("RTSConfigurationViewImpl::getConfigurationSet");
    configSetList_.clear();

    currentRtc_ = std::make_shared<RTCWrapper>();
    if (currentRtc_->getConfiguration(currentItem_, configSetList_)) {
        txtCompName_->setText(QString(string(currentRtc_->getInstanceName()).c_str()));
    }
}


void RTSConfigurationViewImpl::applyClicked()
{
    DDEBUG("RTSConfigurationViewImpl::applyClicked");
    int selectedSet = lstConfigSet_->currentRow();
    int selected = lstDetail_->currentRow();

    currentRtc_->updateConfiguration(configSetList_);

    getConfigurationSet();
    showConfigurationSetView();

    if (selectedSet < lstConfigSet_->rowCount()) {
        lstConfigSet_->setCurrentCell(selectedSet, 0);
    }
    if (selected < lstDetail_->rowCount()) {
        lstDetail_->setCurrentCell(selected, 0);
    }
}


void RTSConfigurationViewImpl::cancelClicked()
{
    int selectedSet = lstConfigSet_->currentRow();
    int selected = lstDetail_->currentRow();

    getConfigurationSet();
    showConfigurationSetView();

    lstConfigSet_->setCurrentCell(selectedSet, 0);
    lstDetail_->setCurrentCell(selected, 0);
}


void RTSConfigurationViewImpl::updateConfigSet()
{
    DDEBUG("RTSConfigurationViewImpl::updateConfigSet");

    int selectedId = -1;
    QTableWidgetItem* item = lstConfigSet_->currentItem();
    if (item) {
        selectedId = item->data(Qt::UserRole).toInt();
    }
    if (selectedId == -1) {
        return;
    }

    vector<ConfigurationSetParamPtr>::iterator targetConf =
        find_if(configSetList_.begin(), configSetList_.end(), ConfigurationSetComparator(selectedId));

    if (targetConf == configSetList_.end()) {
        return;
    }

    int row = lstConfigSet_->currentRow();
    (*targetConf)->setName(lstConfigSet_->item(row, 1)->text());
    (*targetConf)->setChanged();

    if ((*targetConf)->isChangedName()) {
        lstConfigSet_->item(row, 1)->setBackgroundColor("#FFC0C0");
    } else {
        lstConfigSet_->item(row, 1)->setBackgroundColor(Qt::white);
    }
}


void RTSConfigurationViewImpl::activeSetChanged()
{
    DDEBUG("RTSConfigurationViewImpl::activeSetChanged");
    for (int index = 0; index < configSetList_.size(); ++index) {
        ConfigurationSetParamPtr targetSet = configSetList_[index];
        targetSet->updateActive();
        for (int idxRow = 0; idxRow < lstConfigSet_->rowCount(); ++idxRow) {
            int rowId = lstConfigSet_->item(idxRow, 0)->data(Qt::UserRole).toInt();
            if( rowId==targetSet->getId()) {
                DDEBUG_V("index:%d, row:%d", index, idxRow);
                if (targetSet->isChangedActive()) {
                    lstConfigSet_->item(idxRow, 0)->setBackgroundColor("#FFC0C0");
                } else {
                    lstConfigSet_->item(idxRow, 0)->setBackgroundColor(Qt::white);
                }
                break;
            }
        }
    }
    DDEBUG("RTSConfigurationViewImpl::activeSetChanged End");
}


void RTSConfigurationViewImpl::updateDetail()
{
    DDEBUG("RTSConfigurationViewImpl::updateDetail");

    int selectedId = -1;
    QTableWidgetItem* item = lstDetail_->currentItem();
    if (item) {
        selectedId = item->data(Qt::UserRole).toInt();
    }
    if (selectedId == -1) {
        return;
    }

    vector<ConfigurationParamPtr> configList = currentSet_->getConfigurationList();
    vector<ConfigurationParamPtr>::iterator targetConf =
        find_if(configList.begin(), configList.end(), ConfigurationComparator(selectedId));

    if (targetConf == configList.end()) {
        return;
    }

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
