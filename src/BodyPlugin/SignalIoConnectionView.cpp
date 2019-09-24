#include "SignalIoConnectionView.h"
#include "SignalIoConnectionMapItem.h"
#include "WorldItem.h"
#include "BodyItem.h"
#include <cnoid/SignalIoDevice>
#include <cnoid/SignalIoConnectionMap>
#include <cnoid/ViewManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/Buttons>
#include <QBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QTableView>
#include <QHeaderView>
#include <QAbstractTableModel>
#include <QStyledItemDelegate>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

constexpr int NumColumns = 6;
constexpr int OutDeviceColumn = 0;
constexpr int OutSignalNumberColumn = 1;
constexpr int ConnectionArrowColumn = 2;
constexpr int InDeviceColumn = 3;
constexpr int InSignalNumberColumn = 4;
constexpr int NoteColumn = 5;

QString getDeviceLabel(const string& bodyName, const string& deviceName)
{
    QString label;
    if(deviceName.empty()){
        label = bodyName.c_str();
    } else {
        label = QString("%1 - %2").arg(bodyName.c_str()).arg(deviceName.c_str());
    }
    return label;
}

struct DeviceInfo
{
    SignalIoDevice* device;
    string bodyName;
    string deviceName;
    DeviceInfo(SignalIoDevice* device, const string& bodyName, const string& deviceName)
        : device(device), bodyName(bodyName), deviceName(deviceName) { }
};

class ConnectionMapModel : public QAbstractTableModel
{
public:
    SignalIoConnectionMapPtr connectionMap;
    
    ConnectionMapModel(QObject* parent);
    void setConnectionMap(SignalIoConnectionMap* connectionMap);
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    QVariant getDeviceLabel(SignalIoConnection* connection, SignalIoConnection::IoType which) const;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void insertConnection(int row, SignalIoConnection* connection);
};

class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    SignalIoConnectionView::Impl* view;
    
    CustomizedItemDelegate(SignalIoConnectionView::Impl* view);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    QWidget* createSignalNumberSpingBox(
        QWidget* parent, SignalIoConnection* connection, SignalIoConnection::IoType which) const;    
    virtual void updateEditorGeometry(
        QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};

class SignalDeviceComboBox : public QComboBox
{
public:
    vector<DeviceInfo> deviceInfos;

    SignalDeviceComboBox(
        SignalIoConnectionView::Impl* view, SignalIoConnection* connection, SignalIoConnection::IoType which,
        QWidget* parent);

    DeviceInfo* getCurrentDeviceInfo(){ return &deviceInfos[currentIndex()]; }
};

}

namespace cnoid {

class SignalIoConnectionView::Impl : public QTableView
{
public:
    SignalIoConnectionView* self;
    TargetItemPicker<SignalIoConnectionMapItem> targetItemPicker;
    SignalIoConnectionMapItemPtr targetItem;
    QLabel targetLabel;
    ConnectionMapModel* connectionMapModel;

    Impl(SignalIoConnectionView* self);
    void setConnectionMapItem(SignalIoConnectionMapItem* item);
    void onInsertButtonClicked();
    void onRemoveButtonClicked();
    void setConnectionInfoToTableRow(int row, SignalIoConnection* connection);
};

}


ConnectionMapModel::ConnectionMapModel(QObject* parent)
    : QAbstractTableModel(parent)
{

}


void ConnectionMapModel::setConnectionMap(SignalIoConnectionMap* connectionMap)
{
    beginResetModel();
    this->connectionMap = connectionMap;
    endResetModel();
}


int ConnectionMapModel::rowCount(const QModelIndex& parent) const
{
    int n = 0;
    if(!parent.isValid() && connectionMap){
        n = connectionMap->numConnections();
    }
    return (n == 0) ? 1 : n;
}


int ConnectionMapModel::columnCount(const QModelIndex& parent) const
{
    return NumColumns;
}
        

QVariant ConnectionMapModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case 0:
                return _("Output Device");
            case 1:
                return _("Signal No");
            case 3:
                return _("Input Device");
            case 4:
                return _("Signal No");
            case 5:
                return _("Note");
            default:
                return QVariant();
            }
        } else {
            return QString::number(section);
        }
    } else if(role == Qt::TextAlignmentRole){
        if(orientation == Qt::Horizontal){
            if(section == 5){
                return Qt::AlignLeft;
            } else {
                return Qt::AlignCenter;
            }
        }
    }
    return QVariant();
}


QModelIndex ConnectionMapModel::index(int row, int column, const QModelIndex& parent) const
{
    if(!connectionMap || parent.isValid()){
        return QModelIndex();
    }
    if(row < connectionMap->numConnections()){
        auto connection = connectionMap->connection(row);
        return createIndex(row, column, connection);
    }
    return QModelIndex();
}
    

Qt::ItemFlags ConnectionMapModel::flags(const QModelIndex& index) const
{
    if(!index.isValid()){
        return QAbstractTableModel::flags(index);
    }
    if(index.column() != ConnectionArrowColumn){
        return QAbstractTableModel::flags(index) | Qt::ItemIsEditable;
    }
    return QAbstractTableModel::flags(index);
}


QVariant ConnectionMapModel::data(const QModelIndex& index, int role) const
{
    auto connection = static_cast<SignalIoConnection*>(index.internalPointer());

    if(!connection || !index.isValid()){
        return QVariant();
    }

    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(index.column()){
        case OutDeviceColumn:
            return getDeviceLabel(connection, SignalIoConnection::Out);
        case OutSignalNumberColumn:
            return connection->outSignalNumber();
        case ConnectionArrowColumn:
            return "-->";
        case InDeviceColumn:
            return getDeviceLabel(connection, SignalIoConnection::In);
        case InSignalNumberColumn:
            return connection->inSignalNumber();
        default:
            break;
        }
    } else if(role == Qt::TextAlignmentRole){
        if(index.column() == NoteColumn){
            return Qt::AlignLeft;
        } else {
            return Qt::AlignCenter;
        }
    }
            
    return QVariant();
}


QVariant ConnectionMapModel::getDeviceLabel(SignalIoConnection* connection, SignalIoConnection::IoType which) const
{
    auto device = connection->device(which);
    if(device){
        auto body = device->body();
        if(device->name().empty()){
            return body->name().c_str();
        } else {
            return format("{0} - {1}", body->name(), device->name()).c_str();
        }
    } else {
        auto& bodyName = connection->bodyName(which);
        auto& deviceName = connection->deviceName(which);
        if(deviceName.empty()){
            return bodyName.c_str();
        } else {
            return format("{0} - {1}", bodyName, deviceName).c_str();
        }
    }
}


bool ConnectionMapModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if(index.isValid() && role == Qt::EditRole){
        auto connection = static_cast<SignalIoConnection*>(index.internalPointer());
        switch(index.column()){

        case OutDeviceColumn:
        case InDeviceColumn:
            if(auto info = static_cast<DeviceInfo*>(value.value<void*>())){
                auto which = (index.column() == OutDeviceColumn) ? SignalIoConnection::Out : SignalIoConnection::In;
                if(info->device){
                    connection->setDevice(which, info->device);
                } else {
                    connection->setNames(which, info->bodyName, info->deviceName);
                }
                Q_EMIT dataChanged(index, index, {role});
            }
            break;
            
        case OutSignalNumberColumn:
            connection->setOutSignalNumber(value.toInt());
            Q_EMIT dataChanged(index, index, {role});
            return true;
            break;
            
        case InSignalNumberColumn:
            connection->setInSignalNumber(value.toInt());
            Q_EMIT dataChanged(index, index, {role});
            return true;
        default:
            break;
        }
    }
    return false;
}


void ConnectionMapModel::insertConnection(int row, SignalIoConnection* connection)
{
    if(connectionMap){
        beginInsertRows(QModelIndex(), row, 0);
        connectionMap->insert(row, connection);
        endInsertRows();
    }
}


CustomizedItemDelegate::CustomizedItemDelegate(SignalIoConnectionView::Impl* view)
    : QStyledItemDelegate(view),
      view(view)
{

}


QWidget* CustomizedItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QWidget* editor = nullptr;
    
    auto connection = static_cast<SignalIoConnection*>(index.internalPointer());
    
    switch(index.column()){
    case OutDeviceColumn:
        editor = new SignalDeviceComboBox(view, connection, SignalIoConnection::Out, parent);
        break;
    case OutSignalNumberColumn:
        editor = createSignalNumberSpingBox(parent, connection, SignalIoConnection::Out);
        break;
    case InDeviceColumn:
        editor = new SignalDeviceComboBox(view, connection, SignalIoConnection::In, parent);
        break;
    case InSignalNumberColumn:
        editor = createSignalNumberSpingBox(parent, connection, SignalIoConnection::In);
        break;
    default:
        editor = QStyledItemDelegate::createEditor(parent, option, index);
        break;
    }

    return editor;
}


QWidget* CustomizedItemDelegate::createSignalNumberSpingBox
(QWidget* parent, SignalIoConnection* connection, SignalIoConnection::IoType which) const
{
    auto spin = new QSpinBox(parent);
    spin->setAlignment(Qt::AlignCenter);
    spin->setFrame(false);
    if(auto device = connection->device(which)){
        spin->setRange(0, device->numBinarySignals() - 1);
    } else {
        spin->setRange(0, 999);
    }
    return spin;
}


void CustomizedItemDelegate::updateEditorGeometry
(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    editor->setGeometry(option.rect);

    if(auto combo = dynamic_cast<QComboBox*>(editor)){
        combo->showPopup();
    }
}


void CustomizedItemDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    QStyledItemDelegate::setEditorData(editor, index);
}


void CustomizedItemDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    switch(index.column()){
    case OutDeviceColumn:
    case InDeviceColumn:
        if(auto combo = dynamic_cast<SignalDeviceComboBox*>(editor)){
            auto value = QVariant::fromValue(static_cast<void*>(combo->getCurrentDeviceInfo()));
            model->setData(index, value);
        }
        break;
    default:
        QStyledItemDelegate::setModelData(editor, model, index);
    }
}


SignalDeviceComboBox::SignalDeviceComboBox
(SignalIoConnectionView::Impl* view, SignalIoConnection* connection, SignalIoConnection::IoType which, QWidget* parent)
    : QComboBox(parent)
{
    setFrame(false);

    string currentBodyName;
    string currentDeviceName;
    auto currentDevice = connection->device(which);
    if(currentDevice){
        currentBodyName = currentDevice->body()->name();
        currentDeviceName = currentDevice->name();
    } else {
        currentBodyName = connection->bodyName(which);
        currentDeviceName = connection->deviceName(which);
    }
    bool isCurrentDeviceAvailable = false;
    
    if(view->targetItem){
        view->targetItem->forEachIoDevice(
            [&](BodyItem* bodyItem, SignalIoDevice* device){
                auto body = device->body();
                int index = count();
                addItem(getDeviceLabel(body->name(), device->name()));
                deviceInfos.emplace_back(device, body->name(), device->name());
                
                if(!isCurrentDeviceAvailable){
                    if((currentDevice && currentDevice == device) ||
                       (body->name() == currentBodyName && device->name() == currentDeviceName)){
                        isCurrentDeviceAvailable = true;
                        setCurrentIndex(index);
                    }
                }
            });

/*        
        Item* rootItem = view->targetItem->findOwnerItem<WorldItem>();
        if(!rootItem){
            rootItem = RootItem::instance();
        }
        ItemList<BodyItem> bodyItems;
        bodyItems.extractSubTreeItems(rootItem);
        for(auto& bodyItem : bodyItems){
            for(auto& device : bodyItem->body()->devices<SignalIoDevice>()){
                auto body = device->body();
                int index = count();
                addItem(getDeviceLabel(body->name(), device->name()));
                deviceInfos.emplace_back(device, body->name(), device->name());
                
                if(!isCurrentDeviceAvailable){
                    if((currentDevice && currentDevice == device) ||
                       (body->name() == currentBodyName && device->name() == currentDeviceName)){
                        isCurrentDeviceAvailable = true;
                        setCurrentIndex(index);
                    }
                }
            }
    }
*/
    }
    if(!isCurrentDeviceAvailable && !currentBodyName.empty()){
        int index = count();
        addItem(getDeviceLabel(currentBodyName, currentDeviceName));
        setCurrentIndex(index);
        deviceInfos.emplace_back(nullptr, currentBodyName, currentDeviceName);
    }
}


void SignalIoConnectionView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<SignalIoConnectionView>(
        "SignalIoConnectionView", N_("I/O Connection"), ViewManager::SINGLE_OPTIONAL);
}


SignalIoConnectionView::SignalIoConnectionView()
{
    impl = new Impl(this);
}


SignalIoConnectionView::Impl::Impl(SignalIoConnectionView* self)
    : self(self),
      targetItemPicker(self)
{
    self->setDefaultLayoutArea(View::RIGHT);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    int hs = self->style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
    
    auto hbox = new QHBoxLayout;
    hbox->setSpacing(0);
    hbox->addSpacing(hs);
    targetLabel.setStyleSheet("font-weight: bold");
    hbox->addWidget(&targetLabel, 0, Qt::AlignVCenter);
    hbox->addSpacing(hs);
    auto insertButton = new PushButton(_("Insert"));
    insertButton->sigClicked().connect([&](){ onInsertButtonClicked(); });
    hbox->addWidget(insertButton);
    auto removeButton = new PushButton(_("Remove"));
    removeButton->sigClicked().connect([&](){ onRemoveButtonClicked(); });
    hbox->addWidget(removeButton);
    hbox->addStretch();
    vbox->addLayout(hbox);

    // Setup the table
    auto hframe = new QFrame;
    hframe->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    vbox->addWidget(hframe);
    setFrameShape(QFrame::NoFrame);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setTabKeyNavigation(true);
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    setItemDelegate(new CustomizedItemDelegate(this));
    setEditTriggers(
        /* QAbstractItemView::CurrentChanged | */
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::SelectedClicked |
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    auto hheader = horizontalHeader();
    hheader->setSectionResizeMode(QHeaderView::ResizeToContents);
    hheader->setStretchLastSection(true);
    auto vheader = verticalHeader();
    vheader->setSectionResizeMode(QHeaderView::ResizeToContents);

    connectionMapModel = new ConnectionMapModel(this);
    setModel(connectionMapModel);

    vbox->addWidget(this);
    self->setLayout(vbox);

    targetItemPicker.sigTargetItemChanged().connect(
        [&](SignalIoConnectionMapItem* item){
            setConnectionMapItem(item); });
}


SignalIoConnectionView::~SignalIoConnectionView()
{
    delete impl;
}


void SignalIoConnectionView::Impl::setConnectionMapItem(SignalIoConnectionMapItem* item)
{
    targetItem = item;

    if(item){
        targetLabel.setText(item->name().c_str());
        connectionMapModel->setConnectionMap(item->connectionMap());
    } else {
        targetLabel.setText("---");
        connectionMapModel->setConnectionMap(nullptr);
    }
}


void SignalIoConnectionView::Impl::onInsertButtonClicked()
{
    if(!targetItem){
        return;
    }

    Item* rootItem = targetItem->findOwnerItem<WorldItem>();
    ItemList<BodyItem> bodyItems;
    bodyItems.extractSubTreeItems(rootItem ? rootItem : RootItem::instance());
    SignalIoConnectionPtr intraConnection;
    SignalIoConnectionPtr interConnection;
    for(auto& bodyItem1 : bodyItems){
        if(auto ioDevice1 = bodyItem1->body()->findDevice<SignalIoDevice>()){
            for(auto& bodyItem2 : bodyItems){
                if(auto ioDevice2 = bodyItem2->body()->findDevice<SignalIoDevice>()){
                    if(!intraConnection && bodyItem1 == bodyItem2){
                        intraConnection = new SignalIoConnection(ioDevice1, 0, ioDevice2, 0);
                    } else if(!interConnection && bodyItem1 != bodyItem2){
                        interConnection = new SignalIoConnection(ioDevice1, 0, ioDevice2, 0);
                    }
                    if(intraConnection && interConnection){
                        break;
                    }
                }
            }
        }
    }

    if(interConnection){
        connectionMapModel->insertConnection(0, interConnection);
    } else if(intraConnection){
        connectionMapModel->insertConnection(0, intraConnection);
    } else {
        showWarningDialog(_("There are no I/O devices in the world"));
    }
}


void SignalIoConnectionView::Impl::onRemoveButtonClicked()
{

}

       
bool SignalIoConnectionView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "currentConnectionMapItem");
    return true;
}


bool SignalIoConnectionView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "currentConnectionMapItem");
    return true;
}
