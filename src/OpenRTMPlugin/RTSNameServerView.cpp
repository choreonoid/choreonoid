/**
   @author Shin'ichiro Nakaoka
   @author Hisashi Ikari
*/
#include "RTSNameServerView.h"
#include <cnoid/ViewManager>

#include <cnoid/Buttons>
#include <cnoid/SpinBox>
#include <cnoid/LineEdit>
#include <cnoid/MessageView>
#include <QBoxLayout>
#include <QIcon>
#include <QMimeData>
#include <QDrag>
#include <boost/algorithm/string.hpp>
#include "gettext.h"

#undef _HOST_CXT_VERSION

namespace cnoid {

class RTSVItem : public QTreeWidgetItem
{
public :
    RTSVItem(const NamingContextHelper::ObjectInfo& info){
        info_ = info;
        QString name = info_.id.c_str();
        setText(0, name);
        setIcon(0, info_.isAlive ? QIcon(":/Corba/icons/NSRTC.png") :
                QIcon(":/Corba/icons/NSZombi.png"));
    }
    NamingContextHelper::ObjectInfo info_;
};

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


class RTSNameServerViewImpl
{
public:
    RTSNameServerViewImpl(RTSNameServerView* self);
    ~RTSNameServerViewImpl();
    void updateObjectList(bool force=false);
    void updateObjectList(const NamingContextHelper::ObjectInfoList& objects, QTreeWidgetItem* parent);
    void setConnection();
    void onSelectionChanged();
    //void selectedItem();
#ifdef _HOST_CXT_VERSION
    void extendDiagram(const NamingContextHelper::ObjectInfo& info, QTreeWidgetItem* parent);
#endif
    void clearDiagram();
    void setSelection(std::string RTCName);

    Signal<void(const std::list<NamingContextHelper::ObjectInfo>&)> sigSelectionChanged;
    Signal<void(std::string, int)> sigLocationChanged;
    RTSNameTreeWidget treeWidget;
    LineEdit hostAddressBox;
    SpinBox portNumberSpin;

    NamingContextHelper ncHelper;
    std::list<NamingContextHelper::ObjectInfo> selectedItemList;
};

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


RTSNameServerViewImpl::RTSNameServerViewImpl(RTSNameServerView* self)
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);

    QVBoxLayout* vbox = new QVBoxLayout();

    QHBoxLayout* hbox = new QHBoxLayout();
    hostAddressBox.setText("localhost");
    hostAddressBox.sigEditingFinished().connect
        (std::bind(
            static_cast<void(RTSNameServerViewImpl::*)(bool)>(&RTSNameServerViewImpl::updateObjectList), this, false));
    hbox->addWidget(&hostAddressBox);

    portNumberSpin.setRange(0, 65535);
    portNumberSpin.setValue(2809);
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
    impl->updateObjectList();
}


void RTSNameServerViewImpl::updateObjectList(bool force)
{
    if(ncHelper.host()==hostAddressBox.string() && ncHelper.port()==portNumberSpin.value() && !force)
            return;

    try {
        treeWidget.clear();

        // Update to connect information
        ncHelper.setLocation(hostAddressBox.string(), portNumberSpin.value());

        // Connection information updates to all views.
        sigLocationChanged(hostAddressBox.string(), portNumberSpin.value());

        // Clear information update to all views.
        //clearDiagram();
        
        if(ncHelper.isAlive()){
            NamingContextHelper::ObjectInfoList objects = ncHelper.getObjectList();
            updateObjectList(objects, NULL);
            treeWidget.expandAll();
        }else{
            showWarningDialog(ncHelper.errorMessage());
        }

    } catch (...) {
        // ignore the exception for non crash.
    }
}


void RTSNameServerViewImpl::updateObjectList(const NamingContextHelper::ObjectInfoList& objects, QTreeWidgetItem* parent)
{
    for(size_t i = 0; i < objects.size(); ++i){
        const NamingContextHelper::ObjectInfo& info = objects[i];
        if (boost::iequals(info.kind, "rtc") || boost::iequals(info.kind, "host_cxt")) {
            if (parent == NULL && boost::iequals(info.kind, "host_cxt")) {
            #ifdef _HOST_CXT_VERSION
                extendDiagram(info, parent);
            } else if (parent && boost::iequals(info.kind, "rtc")){
            #else
            } else if (boost::iequals(info.kind, "rtc")){
            #endif
                RTSVItem* item = new RTSVItem(info);
                if (parent == NULL) treeWidget.addTopLevelItem(item);
                else parent->addChild(item);
            }
        }
    }
}


void RTSNameServerViewImpl::onSelectionChanged()
{
    selectedItemList.clear();

    QList<QTreeWidgetItem*> selected = treeWidget.selectedItems();
    for(int i=0; i < selected.size(); ++i){
        RTSVItem* item = dynamic_cast<RTSVItem*>(selected[i]);
        if(item){
            selectedItemList.push_back(item->info_);
        }
    }

    sigSelectionChanged(selectedItemList);
}


void RTSNameServerView::setSelection(std::string RTCName)
{
    impl->setSelection(RTCName);
}


void RTSNameServerViewImpl::setSelection(std::string RTCName)
{
    if(RTCName.empty()){
        treeWidget.clearSelection();
        return;
    }

    updateObjectList();

    QList<QTreeWidgetItem*> items = treeWidget.findItems(QString(RTCName.c_str()), Qt::MatchFixedString);
    if(!items.empty())
        treeWidget.setCurrentItem(items[0]);

}


#ifdef _HOST_CXT_VERSION
void RTSNameServerViewImpl::extendDiagram(const NamingContextHelper::ObjectInfo& info, QTreeWidgetItem* parent)
{
    // view the host context.
    QTreeWidgetItem* item = new QTreeWidgetItem();
    QString name = info.id.c_str();
    string rtcName = string(qtos(name));
    item->setIcon(0, QIcon(":/Corba/icons/NSHostCxt.png"));
    item->setText(0, QString(string(rtcName).c_str()));
    if (parent == NULL) treeWidget.addTopLevelItem(item);
    else parent->addChild(item);

    // view the rtc in host context.
    NamingContextHelperPtr helper = NamingContextHelperPtr(new NamingContextHelper);
    QString addressText = hostAddressBox.text();
    string address = string(qtos(addressText));
    int port = portNumberSpin.value(); 
    helper->setLocation(address, port);
    if (helper->isAlive()) {
        CORBA::Object_var obj = helper->findObject(rtcName, "host_cxt");
        helper->bindObject(obj, rtcName, "host_cxt");
        NamingContextHelper::ObjectInfoList objects = helper->getObjectList();
        updateObjectList(objects, item);
        //RTS_RELEASE(obj, "release");
        if (!CORBA::is_nil(obj)) CORBA::release(obj);
    }

}
#endif
