
#include "RTSNameServerView.h"
#include "RTSCommonUtil.h"
#include "OpenRTMUtil.h"
#include "LoggerUtil.h"
#include <cnoid/ViewManager>
#include <cnoid/Dialog>
#include <cnoid/Buttons>
#include <cnoid/SpinBox>
#include <cnoid/LineEdit>
#include <cnoid/CheckBox>
#include <cnoid/ComboBox>
#include <cnoid/MessageView>
#include <rtm/CORBA_IORUtil.h>
#include <QLabel>
#include <QGridLayout>
#include <QMimeData>
#include <QDrag>
#include <QMouseEvent>
#include <QMessageBox>
#include <QTextEdit>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool ENABLE_LIST_UPDATE_FOR_HIDDEN_VIEW_IN_LOADING_PROJECT = false;

class InfomationDialog : public Dialog
{
public:
    InfomationDialog(std::string ior);
};

}

namespace cnoid {

class ConnectDialog : public Dialog
{
public:
    ConnectDialog();

    bool isOK_;
    string hostAddress_;
    int portNum_;
    bool isManager_;

private:
    LineEdit* hostAddressBox_;
    SpinBox* portNumberSpin_;
    CheckBox* chkRTM_;

    void chkCanged();
    void okClicked();
    void cancelClicked();
};

class AddContextDialog : public Dialog
{
public:
    AddContextDialog(RTSVItem* target);

private:
    RTSVItem * target_;

    LineEdit* nameEdit_;
    ComboBox* kindCombo_;

    void okClicked();
};

class AddObjectDialog : public Dialog
{
public:
    AddObjectDialog(RTSVItem* target);

private:
    RTSVItem * target_;

    LineEdit* nameEdit_;
    LineEdit* kindEdit_;
    QTextEdit* iorText_;

    void okClicked();
};

class RTSNameServerViewImpl
{
public:
    bool isObjectListUpdateRequested;

    RTSNameServerViewImpl(RTSNameServerView* self);
    ~RTSNameServerViewImpl();

    Signal<void(const std::list<NamingContextHelper::ObjectInfo>&)> sigSelectionChanged;

    std::list<NamingContextHelper::ObjectInfo> selectedItemList;

    void updateObjectList(bool force = false);
    void setSelection(std::string RTCName, std::string RTCfullPath, NamingContextHelper::ObjectInfo nsInfo);  

private:
    RTSNameServerView * self_;
    RTSNameTreeWidget treeWidget;

    bool isUpdating_;
    PushButton* updateButton_;
    bool cancelUpdating_;

    void updateObjectListLocal();
    void updateObjectList(
        NamingContextHelper::ObjectInfoList& objects, QTreeWidgetItem* parent,
        vector<NamingContextHelper::ObjectPath> pathList);
    void onSelectionChanged();

    void showServerInfo();    void connectNameServer();
    void cleatZombee();
    void checkZombee(RTSVItem* parent);
};

}


InfomationDialog::InfomationDialog(string ior)
{
    QLabel* label01 = new QLabel(_("IOR : "));
    QTextEdit* txtIOR = new QTextEdit;
    txtIOR->setReadOnly(true);
    txtIOR->setText(QString::fromStdString(ior));

    QLabel* label02 = new QLabel(_("Detail : "));
    QTextEdit* txtDetail = new QTextEdit;
    txtDetail->setReadOnly(true);
    txtDetail->setText(QString::fromStdString(CORBA_IORUtil::formatIORinfo(ior.c_str())));

    QFrame* frmDisp = new QFrame;
    QGridLayout* dispLayout = new QGridLayout(frmDisp);
    dispLayout->setContentsMargins(2, 2, 2, 2);
    dispLayout->addWidget(label01, 0, 0);
    dispLayout->addWidget(txtIOR, 0, 1);
    dispLayout->addWidget(label02, 1, 0);
    dispLayout->addWidget(txtDetail, 1, 1);

    QFrame* frmButton = new QFrame;
    QPushButton* okButton = new QPushButton(_("&OK"));
    QHBoxLayout* buttonBotLayout = new QHBoxLayout(frmButton);
    buttonBotLayout->setContentsMargins(2, 2, 2, 2);
    buttonBotLayout->addStretch();
    buttonBotLayout->addWidget(okButton);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->setContentsMargins(2, 2, 2, 2);
    mainLayout->addWidget(frmDisp);
    mainLayout->addWidget(frmButton);
    setLayout(mainLayout);

    connect(okButton, SIGNAL(clicked()), this, SLOT(reject()));

    setMinimumWidth(800);
}


void RTSNameServerView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<RTSNameServerView>(
        "RTSNameServerView", N_("RTC List"), ViewManager::SINGLE_DEFAULT);
}


RTSNameServerView* RTSNameServerView::instance()
{
    return ViewManager::findView<RTSNameServerView>();
}


RTSNameServerView::RTSNameServerView()
{
    impl = new RTSNameServerViewImpl(this);
}


RTSNameServerViewImpl::RTSNameServerViewImpl(RTSNameServerView* self)
    : isUpdating_(false), cancelUpdating_(false)
{
    DDEBUG("RTSNameServerViewImpl::RTSNameServerViewImpl");
    this->self_ = self;

    self->setDefaultLayoutArea(View::LEFT_BOTTOM);
    NamingContextHelper* ncHelper = NameServerManager::instance()->getNCHelper();
    NameServerInfo nsInfo = RTCCommonUtil::getManagerAddress();
    NameServerManager::instance()->addNameServer(nsInfo);

    QVBoxLayout* vbox = new QVBoxLayout();

    QHBoxLayout* hbox = new QHBoxLayout();
    auto connectButton = new PushButton("Add");
    connectButton->setIcon(QIcon(":/Corba/icons/Connect.png"));
    connectButton->setToolTip(_("Add Name Server"));
    connectButton->sigClicked().connect(
        std::bind(
            static_cast<void(RTSNameServerViewImpl::*)(void)>(&RTSNameServerViewImpl::connectNameServer), this));
    hbox->addWidget(connectButton);
    hbox->addStretch();

    updateButton_ = new PushButton(_(" Update "));
    updateButton_->setIcon(QIcon(":/Corba/icons/Refresh.png"));
    updateButton_->setToolTip(_(" Update "));
    updateButton_->sigClicked().connect(
        std::bind(
            static_cast<void(RTSNameServerViewImpl::*)(void)>(&RTSNameServerViewImpl::updateObjectListLocal), this));
    hbox->addWidget(updateButton_);

    auto clearZombeeButton = new PushButton();
    clearZombeeButton->setIcon(QIcon(":/Corba/icons/KillZombie.png"));
    clearZombeeButton->setToolTip(_("Kill All Zombies"));
    clearZombeeButton->sigClicked().connect(
        std::bind(
            static_cast<void(RTSNameServerViewImpl::*)(void)>(&RTSNameServerViewImpl::cleatZombee), this));
    hbox->addWidget(clearZombeeButton);

    isObjectListUpdateRequested = false;

    vbox->addLayout(hbox, 0);

    treeWidget.setHeaderLabel(_("Object Name"));
    treeWidget.setDragEnabled(true);
    treeWidget.setDropIndicatorShown(true);

    //treeWidget.sigItemClicked().connect(std::bind(&RTSNameServerViewImpl::selectedItem, this));
    treeWidget.sigItemSelectionChanged().connect(std::bind(&RTSNameServerViewImpl::onSelectionChanged, this));
    treeWidget.setSelectionMode(QAbstractItemView::ExtendedSelection);
    treeWidget.header()->close();

    vbox->addWidget(&treeWidget, 1);
    self->setLayout(vbox);
    //
    showServerInfo();}


RTSNameServerView::~RTSNameServerView()
{
    delete impl;
}


RTSNameServerViewImpl::~RTSNameServerViewImpl()
{

}


SignalProxy<void(const std::list<NamingContextHelper::ObjectInfo>&)> RTSNameServerView::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


std::list<NamingContextHelper::ObjectInfo> RTSNameServerView::getSelection()
{
    return impl->selectedItemList;
}


void RTSNameServerView::updateView()
{
    impl->updateObjectList(true);
}


void RTSNameServerView::onActivated()
{
    if (impl->isObjectListUpdateRequested) {
        impl->updateObjectList(true);
    }
}


void RTSNameServerViewImpl::updateObjectListLocal()
{
    DDEBUG("RTSNameServerViewImpl::updateObjectListLocal");
    if(isUpdating_) {
        cancelUpdating_ = true;
        updateButton_->setText(_(" Update "));
        updateButton_->setToolTip(_(" Update "));
        isUpdating_ = false;

    } else {
        isUpdating_ = true;
        cancelUpdating_ = false;
        updateButton_->setText(_("Cancel Updating"));
        updateButton_->setToolTip(_("Cancel Updating"));

        updateObjectList(true);

        updateButton_->setText(_(" Update "));
        updateButton_->setToolTip(_(" Update "));
        isUpdating_ = false;
    }
}

void RTSNameServerViewImpl::updateObjectList(bool force)
{
    DDEBUG("RTSNameServerViewImpl::updateObjectList");

    if (isObjectListUpdateRequested) {
        force = true;
        isObjectListUpdateRequested = false;
    }

    if (!force) {
        treeWidget.expandAll();
        return;
    }

    try {
        treeWidget.clear();
        vector<NameServerInfo> serverList = NameServerManager::instance()->getServerList();
        vector<NameServerInfo> addedList;
        for (auto it = serverList.begin(); it != serverList.end(); it++) {
            vector<NameServerInfo>::iterator serverItr = find_if(addedList.begin(), addedList.end(), ServerFullComparator(*it));
            if (serverItr != addedList.end()) continue;
            addedList.push_back(*it);

            // Update to connect information
            NameServerManager::instance()->getNCHelper()->setLocation((*it).hostAddress, (*it).portNo);

            RTSVItem* topElem = new RTSVItem();
            QString hostName;
            if((*it).isRtmDefaultNameServer) {
                hostName = "Default(" + QString::fromStdString((*it).hostAddress) + ":" + QString::number((*it).portNo) + ")";
            } else {
                hostName = QString::fromStdString((*it).hostAddress) + ":" + QString::number((*it).portNo);
            }
            topElem->setText(0, hostName);
            topElem->setIcon(0, QIcon(":/Corba/icons/RT.png"));
            topElem->kind_ = KIND_SERVER;
            topElem->nsInfo_ = (*it);
            NamingContextHelper::ObjectInfo info;
            info.hostAddress_ = (*it).hostAddress;
            info.portNo_ = (*it).portNo;
            info.isRegisteredInRtmDefaultNameServer_ = (*it).isRtmDefaultNameServer;
            topElem->info_ = info;
            treeWidget.addTopLevelItem(topElem);

            MessageView::instance()->flush();

            // Clear information update to all views.
            //clearDiagram();

            if (NameServerManager::instance()->getNCHelper()->updateConnection()) {
                NamingContextHelper::ObjectInfoList objects = NameServerManager::instance()->getNCHelper()->getObjectList();
                vector<NamingContextHelper::ObjectPath> pathList;
                updateObjectList(objects, topElem, pathList);
                treeWidget.expandAll();
            } else {
                showWarningDialog(NameServerManager::instance()->getNCHelper()->errorMessage());
            }
            topElem->setIOR(string(NameServerManager::instance()->getNCHelper()->getRootIOR()));
        }
    } catch (...) {
        // ignore the exception for non crash.
        DDEBUG_V("RTSNameServerViewImpl::updateObjectList Error:%s", NameServerManager::instance()->getNCHelper()->errorMessage().c_str());
    }
}


void RTSNameServerViewImpl::updateObjectList
(NamingContextHelper::ObjectInfoList& objects, QTreeWidgetItem* parent, vector<NamingContextHelper::ObjectPath> pathList)
{
    DDEBUG("RTSNameServerViewImpl::updateObjectList Path");
    for (size_t i = 0; i < objects.size(); ++i) {
        MessageView::instance()->flush();
        if (cancelUpdating_) return;

        NamingContextHelper::ObjectInfo& info = objects[i];
        DDEBUG_V("%s=%s, %s", info.id_.c_str(), info.kind_.c_str(), info.ior_.c_str());
        info.isRegisteredInRtmDefaultNameServer_ = ((RTSVItem*)parent)->info_.isRegisteredInRtmDefaultNameServer_;

        NamingContextHelper::ObjectPath path(info.id_, info.kind_);
        pathList.push_back(path);
        RTC::RTObject_ptr rtc = NameServerManager::instance()->getNCHelper()->findObject<RTC::RTObject>(pathList);
        pathList.pop_back();
        if (!CORBA::is_nil(rtc)) {
            RTSVItem* item = new RTSVItem(info, rtc);
            item->kind_ = KIND_RTC;
            parent->addChild(item);
            continue;
        }

        if (!info.isContext_ && !RTCCommonUtil::compareIgnoreCase(info.kind_, "mgr")) {
            RTSVItem* item = new RTSVItem(info, 0);
            item->setText(0, QString::fromStdString(info.id_) + "|");
            item->setIcon(0, QIcon(":/Corba/icons/Question.png"));
            item->kind_ = KIND_OTHER;
            parent->addChild(item);
            continue;
        }

        RTSVItem* item = new RTSVItem(info, 0);
        parent->addChild(item);
        item->setText(0, QString::fromStdString(info.id_) + "|" + QString::fromStdString(info.kind_));

        NamingContextHelper::ObjectPath pathSub(info.id_, info.kind_);
        if (RTCCommonUtil::compareIgnoreCase(info.kind_, "host_cxt")) {
            item->setIcon(0, QIcon(":/Corba/icons/Server.png"));
            item->kind_ = KIND_HOST;

        } else if (RTCCommonUtil::compareIgnoreCase(info.kind_, "cate_cxt")) {
            item->setIcon(0, QIcon(":/Corba/icons/CategoryNamingContext.png"));
            item->kind_ = KIND_CATEGORY;

        } else if (RTCCommonUtil::compareIgnoreCase(info.kind_, "mgr_cxt")) {
            item->setIcon(0, QIcon(":/Corba/icons/ManagerNamingContext.png"));
            item->kind_ = KIND_MANAGER;

        } else if (RTCCommonUtil::compareIgnoreCase(info.kind_, "mod_cxt")) {
            item->setIcon(0, QIcon(":/Corba/icons/ModuleNamingContext.png"));
            item->kind_ = KIND_MODULE;

        } else if (RTCCommonUtil::compareIgnoreCase(info.kind_, "server_cxt")) {
            item->setIcon(0, QIcon(":/Corba/icons/RT.png"));
            item->kind_ = KIND_SERVER;

        } else if (RTCCommonUtil::compareIgnoreCase(info.kind_, "mgr")) {
            item->setIcon(0, QIcon(":/Corba/icons/RTCManager.png"));
            item->kind_ = KIND_RTC_MANAGER;

        } else {
            item->setIcon(0, QIcon(":/Corba/icons/Folder.png"));
            item->kind_ = KIND_FOLDER;
        }
        pathList.push_back(pathSub);

        if (item->kind_ != KIND_RTC_MANAGER) {
            NamingContextHelper* ncHelper = NameServerManager::instance()->getNCHelper();
            if (ncHelper->updateConnection()) {
                NamingContextHelper::ObjectInfoList objects = ncHelper->getObjectList(pathList);
                updateObjectList(objects, item, pathList);
            }
        }
        pathList.pop_back();
    }
    DDEBUG("RTSNameServerViewImpl::updateObjectList Path End");
}

void RTSNameServerViewImpl::onSelectionChanged()
{
    DDEBUG("RTSNameServerViewImpl::onSelectionChanged");
    selectedItemList.clear();

    QList<QTreeWidgetItem*> selected = treeWidget.selectedItems();
    for (int i = 0; i < selected.size(); ++i) {
        RTSVItem* item = dynamic_cast<RTSVItem*>(selected[i]);
        if (item) {
            selectedItemList.push_back(item->info_);
        }
    }

    sigSelectionChanged(selectedItemList);
}


void RTSNameServerView::setSelection(std::string RTCName, std::string RTCfullPath, NamingContextHelper::ObjectInfo nsInfo)
{
    impl->setSelection(RTCName, RTCfullPath, nsInfo);
}


void RTSNameServerViewImpl::setSelection(std::string RTCName, std::string RTCfullPath, NamingContextHelper::ObjectInfo nsInfo)
{
    if (RTCName.empty()) {
        treeWidget.clearSelection();
        return;
    }

    updateObjectList();

    QList<QTreeWidgetItem*> items = treeWidget.findItems(QString(RTCName.c_str()), Qt::MatchFixedString | Qt::MatchRecursive);

    DDEBUG_V("match Num:%d", items.size());

    if (items.size() == 1) {
        RTSVItem* target = (RTSVItem*)items[0];
        if (nsInfo.hostAddress_ == target->info_.hostAddress_ 
            && nsInfo.portNo_ == target->info_.portNo_
            && nsInfo.isRegisteredInRtmDefaultNameServer_ == target->info_.isRegisteredInRtmDefaultNameServer_ ) {
            treeWidget.setCurrentItem(items[0]);
        }

    } else {
        for (int index = 0; index < items.size(); ++index) {
            RTSVItem* target = (RTSVItem*)items[index];
            DDEBUG_V("source:%s, target:%s", RTCfullPath.c_str(), target->info_.getFullPath().c_str());
            if (nsInfo.hostAddress_ == target->info_.hostAddress_ 
                && nsInfo.portNo_ == target->info_.portNo_
                && nsInfo.isRegisteredInRtmDefaultNameServer_ == target->info_.isRegisteredInRtmDefaultNameServer_ ) {
                if (RTCfullPath == target->info_.getFullPath()) {
                    treeWidget.setCurrentItem(items[index]);
                    break;
                }
            }
        }
    }
}

void RTSNameServerViewImpl::showServerInfo() {
    treeWidget.clear();
    vector<NameServerInfo> serverList = NameServerManager::instance()->getServerList();
    for (auto it = serverList.begin(); it != serverList.end(); it++) {
        RTSVItem* topElem = new RTSVItem();
        QString hostName;
        if((*it).isRtmDefaultNameServer) {
            hostName = "Default(" + QString::fromStdString((*it).hostAddress) + ":" + QString::number((*it).portNo) + ")";
        } else {
            hostName = QString::fromStdString((*it).hostAddress) + ":" + QString::number((*it).portNo);
        }
        topElem->setText(0, hostName);
        topElem->setIcon(0, QIcon(":/Corba/icons/RT.png"));
        topElem->setDisabled(true);
        topElem->kind_ = KIND_SERVER;
        treeWidget.addTopLevelItem(topElem);
    }
}

void RTSNameServerViewImpl::connectNameServer()
{
    DDEBUG("RTSNameServerViewImpl::connectNameServer");

    ConnectDialog dialog;
    dialog.exec();

    if (!dialog.isOK_) return;

    if (dialog.isManager_) {
        NameServerManager::instance()->addRtmDefaultNameServer();
    } else {
        NameServerInfo nsInfo(dialog.hostAddress_, dialog.portNum_, false);
        NameServerManager::instance()->addNameServer(nsInfo);
    }
    updateObjectList(true);
}

void RTSNameServerViewImpl::cleatZombee()
{
    DDEBUG("RTSNameServerViewImpl::cleatZombee");
    int topNum = treeWidget.topLevelItemCount();
    for (int index = 0; index < topNum; index++) {
        RTSVItem* topItem = (RTSVItem*)treeWidget.topLevelItem(index);
        DDEBUG_V("topItem Name : %s", topItem->text(0).toStdString().c_str());
        checkZombee(topItem);
    }
}

void RTSNameServerViewImpl::checkZombee(RTSVItem* parent)
{
    int childCount = parent->childCount();
    vector<RTSVItem*> removeList;
    for (int idxChild = 0; idxChild < childCount; idxChild++) {
        RTSVItem* childItem = (RTSVItem*)parent->child(idxChild);
        DDEBUG_V("childItem Name : %s", childItem->text(0).toStdString().c_str());
        if (!isObjectAlive(childItem->info_.ior_)) {
            DDEBUG("RTSNameServerViewImpl::cleatZombee INACTIVE");
            childItem->removing_ = true;
            removeList.push_back(childItem);
            try {
                NameServerManager::instance()->getNCHelper()->unbind(childItem->info_.fullPath_);
            } catch(CosNaming::NamingContext::NotFound ex) {
                DDEBUG("RTSNameServerViewImpl::cleatZombee NOT EXIST");
            }
            continue;
        }
        DDEBUG("RTSNameServerViewImpl::cleatZombee ACTIVE");
        checkZombee(childItem);
    }
    //
    for (auto it = removeList.begin(); it != removeList.end(); ++it) {
        parent->removeChild(*it);
    }

}

bool RTSNameServerView::storeState(Archive& archive)
{
    ListingPtr severNodes = new Listing();

    vector<NameServerInfo> serverList = NameServerManager::instance()->getServerList();
    for (auto it = serverList.begin(); it != serverList.end(); it++) {
        MappingPtr eachNode = new Mapping();
        eachNode->write("isDefaultNameServer", (*it).isRtmDefaultNameServer);
        eachNode->write("host", (*it).hostAddress, DOUBLE_QUOTED);
        eachNode->write("port", (*it).portNo);
        severNodes->append(eachNode);
    }
    if (!severNodes->empty()) {
        archive.insert("NameServers", severNodes);
    }
    return true;
}


bool RTSNameServerView::restoreState(const Archive& archive)
{
    DDEBUG("RTSNameServerView::restoreState");
    NameServerManager::instance()->clearNameServer();

    const Listing& nodes = *archive.findListing("NameServers");
    if (nodes.isValid() && !nodes.empty()) {
        for (int index = 0; index < nodes.size(); ++index) {
            const Mapping& node = *nodes[index].toMapping();

            bool isDefaultNameServer = false;
            auto defaultNameServerValueNode = node.find("isDefaultNameServer");
            if (!defaultNameServerValueNode->isValid()) {
                defaultNameServerValueNode = node.find("isOpenRTM"); // for the backward compatibility
            }
            if(defaultNameServerValueNode->isValid()){
                isDefaultNameServer = defaultNameServerValueNode->toBool();
            }
            if (isDefaultNameServer) {
                NameServerManager::instance()->addRtmDefaultNameServer();
            } else {
                string hostAdr = "localhost";
                ValueNode* hostNode = node.find("host");
                if (hostNode->isValid()) {
                    hostAdr = hostNode->toString();
                }
                int portNo = 2809;
                ValueNode* portNode = node.find("port");
                if (portNode->isValid()) {
                    portNo = portNode->toInt();
                }
                NameServerInfo nsInfo(hostAdr, portNo, false);
                NameServerManager::instance()->addNameServer(nsInfo);
            }
        }
    }

    if (NameServerManager::instance()->getServerList().size() == 0) {
        NamingContextHelper* ncHelper = NameServerManager::instance()->getNCHelper();
        NameServerInfo nsInfo(ncHelper->host(), ncHelper->port(), false);
        NameServerManager::instance()->addNameServer(nsInfo);
    }

    archive.addPostProcess(
        [&]() {
        if (isActive()) {
            impl->updateObjectList(true);
        } else {
            if(ENABLE_LIST_UPDATE_FOR_HIDDEN_VIEW_IN_LOADING_PROJECT){
                impl->isObjectListUpdateRequested = true;
            }
        }
    });

    return true;
}


void RTSNameTreeWidget::mouseMoveEvent(QMouseEvent *event)
{
    QByteArray itemData;
    QMimeData *mimeData = new QMimeData;
    mimeData->setData("application/RTSNameServerItem", itemData);
    QDrag *drag = new QDrag(this);
    drag->setPixmap(QPixmap(":/Corba/icons/NSRTC.png"));
    drag->setMimeData(mimeData);
    drag->start(Qt::MoveAction);
}


void RTSNameTreeWidget::mousePressEvent(QMouseEvent* event)
{
    DDEBUG("RTSNameTreeWidget::mousePressEvent");
    if (event->button() == Qt::RightButton) {
        if (currentItem()) {
            menuManager.setNewPopupMenu(this);
            menuManager.addItem(_("Show IOR"))
                ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::showIOR, this));

            RTSVItem* targetItem = (RTSVItem*)this->currentItem();
            if (targetItem->kind_ == KIND_SERVER) {
                menuManager.addItem(_("Delete from View"))
                    ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::deleteFromView, this));
            } else {
                menuManager.addItem(_("Delete from Name Service"))
                    ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::deleteFromNameService, this));
            }

            if (targetItem->kind_ == KIND_RTC) {
                if (isObjectAlive(targetItem->rtc_)) {
                    menuManager.addSeparator();
                    menuManager.addItem("Activate")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::activateComponent, this));
                    menuManager.addItem("Deactivate")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::deactivateComponent, this));
                    menuManager.addItem("Reset")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::resetComponent, this));
                    if (!isManagedRTC(((RTSVItem*)this->currentItem())->rtc_)) {
                        menuManager.addItem("Exit")
                            ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::finalizeComponent, this));
                    }
                    menuManager.addSeparator();
                    menuManager.addItem("Start")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::startExecutionContext, this));
                    menuManager.addItem("Stop")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::stopExecutionContext, this));
                }
            } else if (targetItem->kind_ == KIND_RTC_MANAGER) {

            } else {
                if (targetItem->kind_ != KIND_OTHER) {
                    menuManager.addItem(_("Add Context"))
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::addContext, this));
                    menuManager.addItem(_("Add Object"))
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::addObject, this));
                }
            }

            menuManager.popupMenu()->popup(event->globalPos());
        }
    }
    TreeWidget::mousePressEvent(event);
}


void RTSNameTreeWidget::showIOR()
{
    InfomationDialog dialog(((RTSVItem*)this->currentItem())->getIOR());
    dialog.exec();
}


void RTSNameTreeWidget::deleteFromView()
{
    DDEBUG("RTSNameTreeWidget::deleteFromView");

    RTSVItem* item = (RTSVItem*)this->currentItem();
    QString target = item->text(0);
    DDEBUG_V("target:%s", target.toStdString().c_str());
    NameServerManager::instance()->removeNameServer(item->nsInfo_);
    delete item;
}


void RTSNameTreeWidget::deleteFromNameService()
{
    int ret = QMessageBox::question(this, _("Confirm"), _("Are you sure you want to delete?"));
    if (ret == QMessageBox::No) {
        return;
    }

    if (!NameServerManager::instance()->getNCHelper()->isAlive()) {
        return;
    }

    RTSVItem* item = (RTSVItem*)this->currentItem();
    NameServerManager::instance()->getNCHelper()->unbind(item->info_.fullPath_);
    RTSNameServerView::instance()->updateView();
}


void RTSNameTreeWidget::addContext()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    AddContextDialog dialog(item);
    dialog.exec();
    RTSNameServerView::instance()->updateView();
}


void RTSNameTreeWidget::addObject()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    AddObjectDialog dialog(item);
    dialog.exec();
    RTSNameServerView::instance()->updateView();
}


void RTSNameTreeWidget::activateComponent()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if (!item->activateComponent()) {
        QMessageBox::information(this, _("Activate"), _("Activation of target component FAILED."));
    }
}


void RTSNameTreeWidget::deactivateComponent()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if (!item->deactivateComponent()) {
        QMessageBox::information(this, _("Deactivate"), _("Deactivation of target component FAILED."));
    }
}


void RTSNameTreeWidget::resetComponent()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if (!item->resetComponent()) {
        QMessageBox::information(this, _("Reset"), _("FAILED to reset target component."));
    }
}


void RTSNameTreeWidget::finalizeComponent()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if (!item->finalizeComponent()) {
        QMessageBox::information(this, _("Exit"), _("FAILED to exit target component."));
    }
    item->setHidden(true);
}


void RTSNameTreeWidget::startExecutionContext()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if (!item->startExecutionContext()) {
        QMessageBox::information(this, _("Start"), _("FAILED to start ExecutionContext."));
    }
}


void RTSNameTreeWidget::stopExecutionContext()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if (!item->stopExecutionContext()) {
        QMessageBox::information(this, _("Start"), _("FAILED to stop ExecutionContext."));
    }
}


RTSVItem::RTSVItem() : removing_(false)
{

}


RTSVItem::RTSVItem(const NamingContextHelper::ObjectInfo& info, RTC::RTObject_ptr rtc) : removing_(false)
{
    info_ = info;
    QString name = info_.id_.c_str();
    setText(0, name);
    setIcon(0, info_.isAlive_ ? QIcon(":/Corba/icons/NSRTC.png") : QIcon(":/Corba/icons/NSZombi.png"));
    setIOR(info.ior_);

    if (rtc) {
        setRTObject(rtc);
    }
}

//////////
AddContextDialog::AddContextDialog(RTSVItem* target)
{
    this->target_ = target;

    QLabel* label01 = new QLabel(_("Name : "));
    nameEdit_ = new LineEdit;

    QLabel* label02 = new QLabel(_("Kind : "));
    kindCombo_ = new ComboBox;
    kindCombo_->addItem("");
    kindCombo_->addItem("host_cxt");
    kindCombo_->addItem("mgr_cxt");
    kindCombo_->addItem("cate_cxt");
    kindCombo_->addItem("mod_cxt");
    kindCombo_->setEditable(true);

    QFrame* frmDisp = new QFrame;
    QGridLayout* dispLayout = new QGridLayout;
    dispLayout->setContentsMargins(3, 3, 3, 3);
    frmDisp->setLayout(dispLayout);
    dispLayout->addWidget(label01, 0, 0, 1, 1, Qt::AlignRight);
    dispLayout->addWidget(nameEdit_, 0, 1, 1, 1);
    dispLayout->addWidget(label02, 1, 0, 1, 1, Qt::AlignRight);
    dispLayout->addWidget(kindCombo_, 1, 1, 1, 1);

    QFrame* frmButton = new QFrame;
    PushButton* okButton = new PushButton(_("&OK"));
    okButton->setAutoDefault(true);
    okButton->sigClicked().connect(
        std::bind(
            static_cast<void(AddContextDialog::*)(void)>(&AddContextDialog::okClicked), this));

    PushButton* cancelButton = new PushButton(_("&Cancel"));
    cancelButton->setAutoDefault(false);
    cancelButton->sigClicked().connect(
        std::bind(
            static_cast<void(AddContextDialog::*)(void)>(&AddContextDialog::reject), this));

    QHBoxLayout* buttonBotLayout = new QHBoxLayout(frmButton);
    buttonBotLayout->setContentsMargins(2, 2, 2, 2);
    buttonBotLayout->addWidget(cancelButton);
    buttonBotLayout->addStretch();
    buttonBotLayout->addWidget(okButton);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->setContentsMargins(2, 2, 2, 2);
    mainLayout->addWidget(frmDisp);
    mainLayout->addWidget(frmButton);
    setLayout(mainLayout);

    setMinimumWidth(300);
}


void AddContextDialog::okClicked()
{
    if (!NameServerManager::instance()->getNCHelper()->isAlive()) {
        close();
        return;
    }

    std::vector<NamingContextHelper::ObjectPath> pathList = target_->info_.fullPath_;
    QString name = nameEdit_->text();
    QString kind = kindCombo_->currentText();

    NamingContextHelper::ObjectPath path(name.toStdString(), kind.toStdString());
    pathList.push_back(path);
    if (!NameServerManager::instance()->getNCHelper()->bind_new_context(pathList)) {
        QMessageBox::information(this, _("Add Context"), _("Failed to add context."));
        nameEdit_->setFocus();
        nameEdit_->selectAll();
        return;
    }
    close();
}


AddObjectDialog::AddObjectDialog(RTSVItem* target)
{
    this->target_ = target;

    QLabel* label01 = new QLabel(_("Name : "));
    nameEdit_ = new LineEdit;

    QLabel* label02 = new QLabel(_("Kind : "));
    kindEdit_ = new LineEdit;

    QLabel* label03 = new QLabel(_("IOR : "));
    iorText_ = new QTextEdit;

    QFrame* frmDisp = new QFrame;
    QGridLayout* dispLayout = new QGridLayout;
    dispLayout->setContentsMargins(3, 3, 3, 3);
    frmDisp->setLayout(dispLayout);
    dispLayout->addWidget(label01, 0, 0, 1, 1, Qt::AlignRight);
    dispLayout->addWidget(nameEdit_, 0, 1, 1, 1);
    dispLayout->addWidget(label02, 1, 0, 1, 1, Qt::AlignRight);
    dispLayout->addWidget(kindEdit_, 1, 1, 1, 1);
    dispLayout->addWidget(label03, 2, 0, 1, 1, Qt::AlignRight);
    dispLayout->addWidget(iorText_, 2, 1, 1, 1);

    QFrame* frmButton = new QFrame;
    PushButton* okButton = new PushButton(_("&OK"));
    okButton->setAutoDefault(true);
    okButton->sigClicked().connect(
        std::bind(
            static_cast<void(AddObjectDialog::*)(void)>(&AddObjectDialog::okClicked), this));

    PushButton* cancelButton = new PushButton(_("&Cancel"));
    cancelButton->setAutoDefault(false);
    cancelButton->sigClicked().connect(
        std::bind(
            static_cast<void(AddObjectDialog::*)(void)>(&AddObjectDialog::reject), this));

    QHBoxLayout* buttonBotLayout = new QHBoxLayout(frmButton);
    buttonBotLayout->setContentsMargins(2, 2, 2, 2);
    buttonBotLayout->addWidget(cancelButton);
    buttonBotLayout->addStretch();
    buttonBotLayout->addWidget(okButton);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->setContentsMargins(2, 2, 2, 2);
    mainLayout->addWidget(frmDisp);
    mainLayout->addWidget(frmButton);
    setLayout(mainLayout);

    setMinimumWidth(350);
}


void AddObjectDialog::okClicked()
{
    if (!NameServerManager::instance()->getNCHelper()->isAlive()) {
        close();
        return;
    }

    std::vector<NamingContextHelper::ObjectPath> pathList = target_->info_.fullPath_;
    QString name = nameEdit_->text();
    QString kind = kindEdit_->text();
    QString ior = iorText_->toPlainText();
    NamingContextHelper::ObjectPath path(name.toStdString(), kind.toStdString());
    pathList.push_back(path);
    string strIor = ior.toStdString();
    if (!NameServerManager::instance()->getNCHelper()->bindObject(pathList, strIor)) {
        QMessageBox::information(this, _("Add Object"), _("Failed to add object."));
        return;
    }
    close();
}

ConnectDialog::ConnectDialog() : isOK_(false), hostAddress_(""), portNum_(2809), isManager_(false)
{
    NamingContextHelper* ncHelper = NameServerManager::instance()->getNCHelper();

    QFrame* frmBase = new QFrame;
    QGridLayout* baseLayout = new QGridLayout();
    baseLayout->setContentsMargins(2, 0, 2, 0);
    frmBase->setLayout(baseLayout);

    QLabel* lblhostAddress = new QLabel("Address:");
    hostAddressBox_ = new LineEdit();
    hostAddressBox_->setText(ncHelper->host());

    QLabel* lblportNumbers = new QLabel("Port:");
    portNumberSpin_ = new SpinBox();
    portNumberSpin_->setRange(0, 65535);
    portNumberSpin_->setValue(ncHelper->port());

    chkRTM_ = new CheckBox(_("Name server set in OpenRTM"));
    chkRTM_->sigToggled().connect(
        std::bind(
            static_cast<void(ConnectDialog::*)(void)>(&ConnectDialog::chkCanged), this));

    baseLayout->addWidget(lblhostAddress, 0, 0, 1, 1);
    baseLayout->addWidget(hostAddressBox_, 0, 1, 1, 1);
    baseLayout->addWidget(lblportNumbers, 0, 2, 1, 1);
    baseLayout->addWidget(portNumberSpin_, 0, 3, 1, 1);
    baseLayout->addWidget(chkRTM_, 1, 0, 1, 4);

    QFrame* frmButtons = new QFrame;
    PushButton* btnOK = new PushButton(_("OK"));
    btnOK->sigClicked().connect(
        std::bind(
            static_cast<void(ConnectDialog::*)(void)>(&ConnectDialog::okClicked), this));
    PushButton* btnCancel = new PushButton(_("Cancel"));
    btnCancel->sigClicked().connect(
        std::bind(
            static_cast<void(ConnectDialog::*)(void)>(&ConnectDialog::cancelClicked), this));

    QHBoxLayout* buttonLayout = new QHBoxLayout(frmButtons);
    buttonLayout->setContentsMargins(2, 2, 2, 2);
    buttonLayout->addWidget(btnOK);
    buttonLayout->addStretch();
    buttonLayout->addWidget(btnCancel);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(frmBase);
    mainLayout->addWidget(frmButtons);
    setLayout(mainLayout);
}

void ConnectDialog::chkCanged()
{
    hostAddressBox_->setEnabled(!chkRTM_->isChecked());
    portNumberSpin_->setEnabled(!chkRTM_->isChecked());
}

void ConnectDialog::cancelClicked()
{
    isOK_ = false;
    close();
}

void ConnectDialog::okClicked()
{
    isOK_ = true;
    hostAddress_ = hostAddressBox_->string();
    portNum_ = portNumberSpin_->value();
    isManager_ = chkRTM_->isChecked();
    close();
}
