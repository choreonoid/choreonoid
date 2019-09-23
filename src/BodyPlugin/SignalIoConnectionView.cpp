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
#include <QTableView>
#include <QAbstractTableModel>
#include <QHeaderView>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

constexpr int NumColumns = 6;

class ConnectionMapModel : public QAbstractTableModel
{
public:
    SignalIoConnectionMapPtr connectionMap;
    
    ConnectionMapModel(QObject* parent);
    void setConnectionMap(SignalIoConnectionMap* connectionMap);
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    QVariant getDeviceLabel(SignalIoConnection* ioConnection, int which) const;
    void insertConnection(int row, SignalIoConnection* connection);
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


QVariant ConnectionMapModel::data(const QModelIndex& index, int role) const
{
    if(!index.isValid() || !connectionMap){
        return QVariant();
    }
    if(index.row() >= connectionMap->numConnections()){
        return QVariant();
    }

    auto ioConnection = connectionMap->connection(index.row());
    
    if(role == Qt::DisplayRole){
        switch(index.column()){
        case 0:
            return getDeviceLabel(ioConnection, SignalIoConnection::Out);
        case 1:
            return ioConnection->outSignalIndex();
        case 2:
            return "-->";
        case 3:
            return getDeviceLabel(ioConnection, SignalIoConnection::In);
        case 4:
            return ioConnection->inSignalIndex();
        default:
            break;
        }
    } else if(role == Qt::TextAlignmentRole){
        if(index.column() == 5){
            return Qt::AlignLeft;
        } else {
            return Qt::AlignCenter;
        }
    }
            
    return QVariant();
}


QVariant ConnectionMapModel::getDeviceLabel(SignalIoConnection* ioConnection, int which) const
{
    auto device = ioConnection->device(which);
    if(device){
        auto body = device->body();
        if(device->name().empty()){
            return body->name().c_str();
        } else {
            return format("{0} - {1}", body->name(), device->name()).c_str();
        }
    } else {
        auto& bodyName = ioConnection->bodyName(which);
        auto& deviceName = ioConnection->deviceName(which);
        if(deviceName.empty()){
            return bodyName.c_str();
        } else {
            return format("{0} - {1}", bodyName, deviceName).c_str();
        }
    }
}


void ConnectionMapModel::insertConnection(int row, SignalIoConnection* connection)
{
    if(connectionMap){
        beginInsertRows(QModelIndex(), row, 0);
        connectionMap->insert(row, connection);
        endInsertRows();
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
