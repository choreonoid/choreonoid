
#include "RTSNameServerView.h"
#include "RTSCommonUtil.h"
#include "OpenRTMUtil.h"
#include "LoggerUtil.h"
#include <cnoid/ViewManager>
#include <cnoid/Buttons>
#include <cnoid/SpinBox>
#include <cnoid/LineEdit>
#include <cnoid/MessageView>
#include <rtm/CORBA_IORUtil.h>
#include <QLabel>
#include <QGridLayout>
#include <QMimeData>
#include <QDrag>
#include <QMouseEvent>
#include <QMessageBox>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class InfomationDialog : public Dialog
{
public:
    InfomationDialog(std::string ior);
};

}

namespace cnoid {

class RTSNameServerViewImpl
{
public:
    RTSNameServerViewImpl(RTSNameServerView* self);
    ~RTSNameServerViewImpl();
    void updateObjectList(bool force = false);
    void updateObjectList(
        const NamingContextHelper::ObjectInfoList& objects, QTreeWidgetItem* parent,
        vector<NamingContextHelper::ObjectPath> pathList);
    void onSelectionChanged();
    void setSelection(std::string RTCName, std::string RTCfullPath);

    RTSNameServerView * self_;
    Signal<void(const std::list<NamingContextHelper::ObjectInfo>&)> sigSelectionChanged;
    Signal<void(std::string, int)> sigLocationChanged;
    RTSNameTreeWidget treeWidget;
    LineEdit hostAddressBox;
    SpinBox portNumberSpin;
    NamingContextHelper ncHelper;
    std::list<NamingContextHelper::ObjectInfo> selectedItemList;
    bool isObjectListUpdateRequested;
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
{
    this->self_ = self;

    self->setDefaultLayoutArea(View::LEFT_BOTTOM);

    QVBoxLayout* vbox = new QVBoxLayout();

    QHBoxLayout* hbox = new QHBoxLayout();
    hostAddressBox.setText(ncHelper.host());
    hostAddressBox.sigEditingFinished().connect
        (std::bind(
            static_cast<void(RTSNameServerViewImpl::*)(bool)>(&RTSNameServerViewImpl::updateObjectList), this, false));
    hbox->addWidget(&hostAddressBox);

    portNumberSpin.setRange(0, 65535);
    portNumberSpin.setValue(ncHelper.port());
    portNumberSpin.sigEditingFinished().connect
        (std::bind(
            static_cast<void(RTSNameServerViewImpl::*)(bool)>(&RTSNameServerViewImpl::updateObjectList), this, false));
    hbox->addWidget(&portNumberSpin);

    auto updateButton = new ToolButton(_(" Update "));
    updateButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    updateButton->sigClicked().connect(
        std::bind(
            static_cast<void(RTSNameServerViewImpl::*)(bool)>(&RTSNameServerViewImpl::updateObjectList), this, true));
    hbox->addWidget(updateButton);

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
}


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


SignalProxy<void(std::string, int)> RTSNameServerView::sigLocationChanged()
{
    return impl->sigLocationChanged;
}


std::list<NamingContextHelper::ObjectInfo> RTSNameServerView::getSelection()
{
    return impl->selectedItemList;
}


const std::string RTSNameServerView::getHost()
{
    return impl->hostAddressBox.string();
}


int RTSNameServerView::getPort()
{
    return impl->portNumberSpin.value();
}


void RTSNameServerView::updateView()
{
    impl->updateObjectList(true);
}


void RTSNameServerView::onActivated()
{
    if(impl->isObjectListUpdateRequested){
        impl->updateObjectList(true);
    }
}


void RTSNameServerViewImpl::updateObjectList(bool force)
{
    if(isObjectListUpdateRequested){
        force = true;
        isObjectListUpdateRequested = false;
    }

    if(ncHelper.host() == hostAddressBox.string() && ncHelper.port() == portNumberSpin.value() && !force){
        treeWidget.expandAll();
        return;
    }

    try {
        treeWidget.clear();

        // Update to connect information
        ncHelper.setLocation(hostAddressBox.string(), portNumberSpin.value());

        // Connection information updates to all views.
        sigLocationChanged(hostAddressBox.string(), portNumberSpin.value());

        RTSVItem* topElem = new RTSVItem();
        QString hostName = QString::fromStdString(hostAddressBox.string()) + ":" + QString::number(portNumberSpin.value());
        topElem->setText(0, hostName);
        topElem->setIcon(0, QIcon(":/Corba/icons/RT.png"));
        topElem->kind_ = KIND_SERVER;
        treeWidget.addTopLevelItem(topElem);

        // Clear information update to all views.
        //clearDiagram();

        if(ncHelper.updateConnection()){
            NamingContextHelper::ObjectInfoList objects = ncHelper.getObjectList();
            vector<NamingContextHelper::ObjectPath> pathList;
            updateObjectList(objects, topElem, pathList);
            treeWidget.expandAll();
        } else {
            showWarningDialog(ncHelper.errorMessage());
        }
        topElem->setIOR(string(ncHelper.getRootIOR()));
    }
    catch (...) {
        // ignore the exception for non crash.
        DDEBUG_V("RTSNameServerViewImpl::updateObjectList Error:%s", ncHelper.errorMessage().c_str());
    }
}


void RTSNameServerViewImpl::updateObjectList
(const NamingContextHelper::ObjectInfoList& objects, QTreeWidgetItem* parent, vector<NamingContextHelper::ObjectPath> pathList)
{
    for(size_t i = 0; i < objects.size(); ++i){
        const NamingContextHelper::ObjectInfo& info = objects[i];
        DDEBUG_V("%s=%s, %s", info.id.c_str(), info.kind.c_str(), info.ior.c_str());

        NamingContextHelper::ObjectPath path(info.id, info.kind);
        pathList.push_back(path);
        RTC::RTObject_ptr rtc = ncHelper.findObject<RTC::RTObject>(pathList);
        pathList.pop_back();
        if(CORBA::is_nil(rtc) == false){
            RTSVItem* item = new RTSVItem(info, rtc);
            item->kind_ = KIND_RTC;
            parent->addChild(item);
            continue;
        }

        if(!info.isContext){
            RTSVItem* item = new RTSVItem(info, 0);
            item->setText(0, QString::fromStdString(info.id) + "|");
            item->setIcon(0, QIcon(":/Corba/icons/Question.png"));
            item->kind_ = KIND_OTHER;
            parent->addChild(item);
            continue;
        }

        RTSVItem* item = new RTSVItem(info, 0);
        parent->addChild(item);
        item->setText(0, QString::fromStdString(info.id) + "|" + QString::fromStdString(info.kind));

        NamingContextHelper::ObjectPath pathSub(info.id, info.kind);
        if(RTCCommonUtil::compareIgnoreCase(info.kind, "host_cxt")){
            item->setIcon(0, QIcon(":/Corba/icons/Server.png"));
            item->kind_ = KIND_HOST;

        } else if(RTCCommonUtil::compareIgnoreCase(info.kind, "cate_cxt")){
            item->setIcon(0, QIcon(":/Corba/icons/CategoryNamingContext.png"));
            item->kind_ = KIND_CATEGORY;

        } else if(RTCCommonUtil::compareIgnoreCase(info.kind, "mgr_cxt")){
            item->setIcon(0, QIcon(":/Corba/icons/ManagerNamingContext.png"));
            item->kind_ = KIND_MANAGER;

        } else if(RTCCommonUtil::compareIgnoreCase(info.kind, "mod_cxt")){
            item->setIcon(0, QIcon(":/Corba/icons/ModuleNamingContext.png"));
            item->kind_ = KIND_MODULE;

        } else if(RTCCommonUtil::compareIgnoreCase(info.kind, "server_cxt")){
            item->setIcon(0, QIcon(":/Corba/icons/RT.png"));
            item->kind_ = KIND_SERVER;

        } else {
            item->setIcon(0, QIcon(":/Corba/icons/Folder.png"));
            item->kind_ = KIND_FOLDER;
        }
        pathList.push_back(pathSub);

        if(ncHelper.updateConnection()){
            NamingContextHelper::ObjectInfoList objects = ncHelper.getObjectList(pathList);
            updateObjectList(objects, item, pathList);
        }
        pathList.pop_back();
    }
}


void RTSNameServerViewImpl::onSelectionChanged()
{
    DDEBUG("RTSNameServerViewImpl::onSelectionChanged");
    selectedItemList.clear();

    QList<QTreeWidgetItem*> selected = treeWidget.selectedItems();
    for(int i = 0; i < selected.size(); ++i){
        RTSVItem* item = dynamic_cast<RTSVItem*>(selected[i]);
        if(item){
            selectedItemList.push_back(item->info_);
        }
    }

    sigSelectionChanged(selectedItemList);
}


void RTSNameServerView::setSelection(std::string RTCName, std::string RTCfullPath)
{
    impl->setSelection(RTCName, RTCfullPath);
}


void RTSNameServerViewImpl::setSelection(std::string RTCName, std::string RTCfullPath)
{
    if(RTCName.empty()){
        treeWidget.clearSelection();
        return;
    }

    updateObjectList();

    QList<QTreeWidgetItem*> items = treeWidget.findItems(QString(RTCName.c_str()), Qt::MatchFixedString | Qt::MatchRecursive);
    if(items.empty()){
        return;
    }

    if(items.size() == 1){
        treeWidget.setCurrentItem(items[0]);

    } else {
        for(int index = 0; index < items.size(); ++index){
            RTSVItem* target = (RTSVItem*)items[index];
            DDEBUG_V("source:%s, target:%s", RTCfullPath.c_str(), target->info_.getFullPath().c_str());
            if(RTCfullPath == target->info_.getFullPath()){
                treeWidget.setCurrentItem(items[index]);
                break;
            }
        }
    }
}


NamingContextHelper RTSNameServerView::getNCHelper()
{
    return impl->ncHelper;
}


bool RTSNameServerView::storeState(Archive& archive)
{
    archive.write("host", impl->ncHelper.host());
    archive.write("port", impl->ncHelper.port());
    return true;
}


bool RTSNameServerView::restoreState(const Archive& archive)
{
    string host;
    if (archive.read("host", host)) {
        impl->hostAddressBox.setText(host.c_str());
    }
    int port;
    if (archive.read("port", port)) {
        impl->portNumberSpin.setValue(port);
    }

    archive.addPostProcess(
        [&](){
            if(isActive()){
                impl->updateObjectList(true);
            } else {
                // impl->isObjectListUpdateRequested = true;
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
    if(event->button() == Qt::RightButton){
        if(currentItem()){
            menuManager.setNewPopupMenu(this);
            menuManager.addItem(_("Show IOR"))
                ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::showIOR, this));

            RTSVItem* targetItem = (RTSVItem*)this->currentItem();
            if(targetItem->kind_ == KIND_SERVER){
                menuManager.addItem(_("Delete from View"))
                    ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::deleteFromView, this));
            } else {
                menuManager.addItem(_("Delete from Name Service"))
                    ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::deleteFromNameService, this));
            }

            if(targetItem->kind_ == KIND_RTC){
                if(RTSNameServerView::instance()->getNCHelper().isObjectAlive(targetItem->rtc_)){
                    menuManager.addSeparator();
                    menuManager.addItem("Activate")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::activateComponent, this));
                    menuManager.addItem("Deactivate")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::deactivateComponent, this));
                    menuManager.addItem("Reset")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::resetComponent, this));
                    if(isManagedRTC(((RTSVItem*)this->currentItem())->rtc_) == false){
                        menuManager.addItem("Exit")
                            ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::finalizeComponent, this));
                    }
                    menuManager.addSeparator();
                    menuManager.addItem("Start")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::startExecutionContext, this));
                    menuManager.addItem("Stop")
                        ->sigTriggered().connect(std::bind(&RTSNameTreeWidget::stopExecutionContext, this));
                }
            } else {
                if(targetItem->kind_ != KIND_OTHER){
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
    RTSVItem* item = (RTSVItem*)this->currentItem();
    delete item;
}


void RTSNameTreeWidget::deleteFromNameService()
{
    int ret = QMessageBox::question(this, _("Confirm"), _("Are you sure you want to delete?"));
    if(ret == QMessageBox::No){
        return;
    }

    if(RTSNameServerView::instance()->getNCHelper().isAlive() == false){
        return;
    }

    RTSVItem* item = (RTSVItem*)this->currentItem();
    RTSNameServerView::instance()->getNCHelper().unbind(item->info_.fullPath);
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
    if(item->activateComponent() == false){
        QMessageBox::information(this, _("Activate"), _("Activation of target component FAILED."));
    }
}


void RTSNameTreeWidget::deactivateComponent()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if(item->deactivateComponent() == false){
        QMessageBox::information(this, _("Deactivate"), _("Deactivation of target component FAILED."));
    }
}


void RTSNameTreeWidget::resetComponent()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if(item->resetComponent() == false){
        QMessageBox::information(this, _("Reset"), _("FAILED to reset target component."));
    }
}


void RTSNameTreeWidget::finalizeComponent()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if(item->finalizeComponent() == false){
        QMessageBox::information(this, _("Exit"), _("FAILED to exit target component."));
    }
    item->setHidden(true);
}


void RTSNameTreeWidget::startExecutionContext()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if(item->startExecutionContext() == false){
        QMessageBox::information(this, _("Start"), _("FAILED to start ExecutionContext."));
    }
}


void RTSNameTreeWidget::stopExecutionContext()
{
    RTSVItem* item = (RTSVItem*)this->currentItem();
    if(item->stopExecutionContext() == false){
        QMessageBox::information(this, _("Start"), _("FAILED to stop ExecutionContext."));
    }
}


RTSVItem::RTSVItem()
{

}


RTSVItem::RTSVItem(const NamingContextHelper::ObjectInfo& info, RTC::RTObject_ptr rtc)
{
    info_ = info;
    QString name = info_.id.c_str();
    setText(0, name);
    setIcon(0, info_.isAlive ? QIcon(":/Corba/icons/NSRTC.png") : QIcon(":/Corba/icons/NSZombi.png"));
    setIOR(info.ior);

    if(rtc){
        setRTObject(rtc);
    }
}


AddContextDialog::AddContextDialog(RTSVItem* target)
{
    this->target_ = target;

    QLabel* label01 = new QLabel(_("Name : "));
    nameEdit_ = new QLineEdit;

    QLabel* label02 = new QLabel(_("Kind : "));
    kindCombo_ = new QComboBox;
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
    QPushButton* okButton = new QPushButton(_("&OK"));
    okButton->setAutoDefault(true);
    QPushButton* cancelButton = new QPushButton(_("&Cancel"));
    cancelButton->setAutoDefault(false);
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

    connect(okButton, SIGNAL(clicked()), this, SLOT(okClicked()));
    connect(cancelButton, SIGNAL(clicked()), this, SLOT(reject()));

    setMinimumWidth(300);
}


void AddContextDialog::okClicked()
{
    if(RTSNameServerView::instance()->getNCHelper().isAlive() == false){
        close();
        return;
    }

    std::vector<NamingContextHelper::ObjectPath> pathList = target_->info_.fullPath;
    QString name = nameEdit_->text();
    QString kind = kindCombo_->currentText();

    NamingContextHelper::ObjectPath path(name.toStdString(), kind.toStdString());
    pathList.push_back(path);
    if(RTSNameServerView::instance()->getNCHelper().bind_new_context(pathList) == false){
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
    nameEdit_ = new QLineEdit;

    QLabel* label02 = new QLabel(_("Kind : "));
    kindEdit_ = new QLineEdit;

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
    QPushButton* okButton = new QPushButton(_("&OK"));
    okButton->setAutoDefault(true);
    QPushButton* cancelButton = new QPushButton(_("&Cancel"));
    cancelButton->setAutoDefault(false);
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

    connect(okButton, SIGNAL(clicked()), this, SLOT(okClicked()));
    connect(cancelButton, SIGNAL(clicked()), this, SLOT(reject()));

    setMinimumWidth(350);
}


void AddObjectDialog::okClicked()
{
    if(RTSNameServerView::instance()->getNCHelper().isAlive() == false){
        close();
        return;
    }

    std::vector<NamingContextHelper::ObjectPath> pathList = target_->info_.fullPath;
    QString name = nameEdit_->text();
    QString kind = kindEdit_->text();
    QString ior = iorText_->toPlainText();
    NamingContextHelper::ObjectPath path(name.toStdString(), kind.toStdString());
    pathList.push_back(path);
    string strIor = ior.toStdString();
    if(RTSNameServerView::instance()->getNCHelper().bindObject(pathList, strIor) == false){
        QMessageBox::information(this, _("Add Object"), _("Failed to add object."));
        return;
    }
    close();
}
