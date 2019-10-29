#include "IoConnectionView.h"
#include "IoConnectionMapItem.h"
#include "WorldItem.h"
#include "BodyItem.h"
#include <cnoid/DigitalIoDevice>
#include <cnoid/IoConnectionMap>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
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
#include <QKeyEvent>
#include <QMouseEvent>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

constexpr int NumColumns = 6;
constexpr int OutDeviceColumn = 0;
constexpr int OutSignalIndexColumn = 1;
constexpr int ConnectionArrowColumn = 2;
constexpr int InDeviceColumn = 3;
constexpr int InSignalIndexColumn = 4;
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
    DigitalIoDevice* device;
    string bodyName;
    string deviceName;
    DeviceInfo(DigitalIoDevice* device, const string& bodyName, const string& deviceName)
        : device(device), bodyName(bodyName), deviceName(deviceName) { }
};

class ConnectionMapModel : public QAbstractTableModel
{
public:
    IoConnectionMapPtr connectionMap;
    
    ConnectionMapModel(QObject* parent);
    void setConnectionMap(IoConnectionMap* connectionMap);
    bool isValid() const { return connectionMap != nullptr; }
    int numConnections() const { return connectionMap ? connectionMap->numConnections() : 0; }
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    QVariant getDeviceLabel(DigitalIoConnection* connection, DigitalIoConnection::IoType which) const;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void addNewConnection(int row, DigitalIoConnection* connection, bool doInsert);
    void removeConnections(QModelIndexList selected);
};

class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    IoConnectionView::Impl* view;
    
    CustomizedItemDelegate(IoConnectionView::Impl* view);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    QWidget* createSignalIndexSpingBox(
        QWidget* parent, DigitalIoConnection* connection, DigitalIoConnection::IoType which) const;    
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
        IoConnectionView::Impl* view, DigitalIoConnection* connection, DigitalIoConnection::IoType which,
        QWidget* parent);

    DeviceInfo* getCurrentDeviceInfo(){ return &deviceInfos[currentIndex()]; }
};

}

namespace cnoid {

class IoConnectionView::Impl : public QTableView
{
public:
    IoConnectionView* self;
    TargetItemPicker<IoConnectionMapItem> targetItemPicker;
    IoConnectionMapItemPtr targetItem;
    QLabel targetLabel;
    ConnectionMapModel* connectionMapModel;
    PushButton addButton;
    MenuManager contextMenuManager;

    Impl(IoConnectionView* self);
    void setConnectionMapItem(IoConnectionMapItem* item);
    void addNewConnection(int index, bool doInsert);
    void addConnectionIntoCurrentIndex(bool doInsert);
    void removeSelectedConnections();
    void setConnectionInfoToTableRow(int row, DigitalIoConnection* connection);
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(int row, QPoint globalPos);
};

}


ConnectionMapModel::ConnectionMapModel(QObject* parent)
    : QAbstractTableModel(parent)
{

}


void ConnectionMapModel::setConnectionMap(IoConnectionMap* connectionMap)
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
    if(n == 0){ // to show an empty row
        n = 1;
    }
    return n;
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
            return Qt::AlignCenter;
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
    auto connection = static_cast<DigitalIoConnection*>(index.internalPointer());

    if(!connection || !index.isValid()){
        return QVariant();
    }

    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(index.column()){
        case OutDeviceColumn:
            return getDeviceLabel(connection, DigitalIoConnection::Out);
        case OutSignalIndexColumn:
            return connection->outSignalIndex();
        case ConnectionArrowColumn:
            return "-->";
        case InDeviceColumn:
            return getDeviceLabel(connection, DigitalIoConnection::In);
        case InSignalIndexColumn:
            return connection->inSignalIndex();
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


QVariant ConnectionMapModel::getDeviceLabel(DigitalIoConnection* connection, DigitalIoConnection::IoType which) const
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
        auto connection = static_cast<DigitalIoConnection*>(index.internalPointer());
        switch(index.column()){

        case OutDeviceColumn:
        case InDeviceColumn:
            if(auto info = static_cast<DeviceInfo*>(value.value<void*>())){
                auto which = (index.column() == OutDeviceColumn) ? DigitalIoConnection::Out : DigitalIoConnection::In;
                if(info->device){
                    connection->setDevice(which, info->device);
                } else {
                    connection->setNames(which, info->bodyName, info->deviceName);
                }
                Q_EMIT dataChanged(index, index, {role});
            }
            break;
            
        case OutSignalIndexColumn:
            connection->setOutSignalIndex(value.toInt());
            Q_EMIT dataChanged(index, index, {role});
            return true;
            break;
            
        case InSignalIndexColumn:
            connection->setInSignalIndex(value.toInt());
            Q_EMIT dataChanged(index, index, {role});
            return true;
        default:
            break;
        }
    }
    return false;
}


void ConnectionMapModel::addNewConnection(int row, DigitalIoConnection* connection, bool doInsert)
{
    if(connectionMap){
        if(connectionMap->numConnections() == 0){
            // Remove the empty row first
            beginRemoveRows(QModelIndex(), 0, 0);
            endRemoveRows();
        }
        int newConnectionRow = doInsert ? row : row + 1;
        beginInsertRows(QModelIndex(), newConnectionRow, newConnectionRow);
        connectionMap->insert(newConnectionRow, connection);
        endInsertRows();
    }
}


void ConnectionMapModel::removeConnections(QModelIndexList selected)
{
    if(connectionMap){
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int row = index.row() - numRemoved;
            beginRemoveRows(QModelIndex(), row, row);
            auto connection = static_cast<DigitalIoConnection*>(index.internalPointer());
            connectionMap->remove(connection);
            ++numRemoved;
            endRemoveRows();
        }
        if(connectionMap->numConnections() == 0){
            // This is necessary to show the empty row
            beginResetModel();
            endResetModel();
        }
    }
}
        

CustomizedItemDelegate::CustomizedItemDelegate(IoConnectionView::Impl* view)
    : QStyledItemDelegate(view),
      view(view)
{

}


QWidget* CustomizedItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QWidget* editor = nullptr;
    
    auto connection = static_cast<DigitalIoConnection*>(index.internalPointer());
    
    switch(index.column()){
    case OutDeviceColumn:
        editor = new SignalDeviceComboBox(view, connection, DigitalIoConnection::Out, parent);
        break;
    case OutSignalIndexColumn:
        editor = createSignalIndexSpingBox(parent, connection, DigitalIoConnection::Out);
        break;
    case InDeviceColumn:
        editor = new SignalDeviceComboBox(view, connection, DigitalIoConnection::In, parent);
        break;
    case InSignalIndexColumn:
        editor = createSignalIndexSpingBox(parent, connection, DigitalIoConnection::In);
        break;
    default:
        editor = QStyledItemDelegate::createEditor(parent, option, index);
        break;
    }

    return editor;
}


QWidget* CustomizedItemDelegate::createSignalIndexSpingBox
(QWidget* parent, DigitalIoConnection* connection, DigitalIoConnection::IoType which) const
{
    auto spin = new QSpinBox(parent);
    spin->setAlignment(Qt::AlignCenter);
    spin->setFrame(false);
    if(auto device = connection->device(which)){
        spin->setRange(0, device->numSignalLines() - 1);
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
(IoConnectionView::Impl* view, DigitalIoConnection* connection, DigitalIoConnection::IoType which, QWidget* parent)
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
            [&](BodyItem* bodyItem, DigitalIoDevice* device){
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
    }

    if(!isCurrentDeviceAvailable && !currentBodyName.empty()){
        int index = count();
        addItem(getDeviceLabel(currentBodyName, currentDeviceName));
        setCurrentIndex(index);
        deviceInfos.emplace_back(nullptr, currentBodyName, currentDeviceName);
    }
}


void IoConnectionView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<IoConnectionView>(
        "IoConnectionView", N_("I/O Connection"), ViewManager::SINGLE_OPTIONAL);
}


IoConnectionView::IoConnectionView()
{
    impl = new Impl(this);
}


IoConnectionView::Impl::Impl(IoConnectionView* self)
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
    addButton.setText(_("Add"));
    addButton.sigClicked().connect([&](){ addConnectionIntoCurrentIndex(false); });
    hbox->addWidget(&addButton);
    hbox->addStretch();
    vbox->addLayout(hbox);

    // Setup the table
    auto hframe = new QFrame;
    hframe->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    vbox->addWidget(hframe);
    setFrameShape(QFrame::NoFrame);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setTabKeyNavigation(true);
    setCornerButtonEnabled(true);
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
        [&](IoConnectionMapItem* item){
            setConnectionMapItem(item); });
}


IoConnectionView::~IoConnectionView()
{
    delete impl;
}


void IoConnectionView::Impl::setConnectionMapItem(IoConnectionMapItem* item)
{
    targetItem = item;

    if(item){
        targetLabel.setText(item->name().c_str());
        connectionMapModel->setConnectionMap(item->connectionMap());
    } else {
        targetLabel.setText("---");
        connectionMapModel->setConnectionMap(nullptr);
    }
    addButton.setEnabled(targetItem != nullptr);
}


void IoConnectionView::Impl::addNewConnection(int index, bool doInsert)
{
    auto& connectionMap = connectionMapModel->connectionMap;
    if(!connectionMap){
        return;
    }
    DigitalIoConnectionPtr connection;
    if(index < connectionMap->numConnections()){
        auto existing = connectionMap->connection(index);
        if(existing->hasDeviceInstances()){
            connection = existing->clone();
        }
    }

    if(!connection){
        Item* rootItem = targetItem->findOwnerItem<WorldItem>();
        ItemList<BodyItem> bodyItems;
        bodyItems.extractSubTreeItems(rootItem ? rootItem : RootItem::instance());
        DigitalIoConnectionPtr intraConnection;
        DigitalIoConnectionPtr interConnection;
        for(auto& bodyItem1 : bodyItems){
            if(auto ioDevice1 = bodyItem1->body()->findDevice<DigitalIoDevice>()){
                for(auto& bodyItem2 : bodyItems){
                    if(auto ioDevice2 = bodyItem2->body()->findDevice<DigitalIoDevice>()){
                        if(!intraConnection && bodyItem1 == bodyItem2){
                            intraConnection = new DigitalIoConnection(ioDevice1, 0, ioDevice2, 0);
                        } else if(!interConnection && bodyItem1 != bodyItem2){
                            interConnection = new DigitalIoConnection(ioDevice1, 0, ioDevice2, 0);
                        }
                        if(intraConnection && interConnection){
                            break;
                        }
                    }
                }
            }
        }
        connection = interConnection ? interConnection : intraConnection;
    }

    if(!connection){
        showWarningDialog(_("There are no I/O devices in the world"));
    } else {
        connectionMapModel->addNewConnection(index, connection, doInsert);
    }
}


void IoConnectionView::Impl::addConnectionIntoCurrentIndex(bool doInsert)
{
    auto current = selectionModel()->currentIndex();
    int index = current.isValid() ? current.row() : connectionMapModel->numConnections();
    addNewConnection(index, doInsert);
}


void IoConnectionView::Impl::removeSelectedConnections()
{
    connectionMapModel->removeConnections(selectionModel()->selectedRows());
}


void IoConnectionView::Impl::keyPressEvent(QKeyEvent* event)
{
    bool processed = true;

    switch(event->key()){
    case Qt::Key_Escape:
        clearSelection();
        break;
    case Qt::Key_Insert:
        addConnectionIntoCurrentIndex(true);
        break;
    case Qt::Key_Delete:
        removeSelectedConnections();
        break;
    default:
        processed = false;
        break;
    }
        
    if(!processed && (event->modifiers() & Qt::ControlModifier)){
        processed = true;
        switch(event->key()){
        case Qt::Key_A:
            selectAll();
            break;
        default:
            processed = false;
            break;
        }
    }

    if(!processed){
        QTableView::keyPressEvent(event);
    }
}

       
void IoConnectionView::Impl::mousePressEvent(QMouseEvent* event)
{
    QTableView::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        int row = rowAt(event->pos().y());
        if(row >= 0){
            showContextMenu(row, event->globalPos());
        }
    }
}


void IoConnectionView::Impl::showContextMenu(int row, QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);

    contextMenuManager.addItem(_("Add"))
        ->sigTriggered().connect([=](){ addNewConnection(row, false); });

    contextMenuManager.addItem(_("Remove"))
        ->sigTriggered().connect([=](){ removeSelectedConnections(); });
    
    contextMenuManager.popupMenu()->popup(globalPos);
}


bool IoConnectionView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "currentConnectionMapItem");
    return true;
}


bool IoConnectionView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "currentConnectionMapItem");
    return true;
}
