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
#include <QTableWidget>
#include <QHeaderView>
#include <QDoubleSpinBox>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class SignalIoConnectionView::Impl : public QTableWidget
{
public:
    SignalIoConnectionView* self;
    TargetItemPicker<SignalIoConnectionMapItem> targetItemPicker;
    SignalIoConnectionMapItemPtr targetItem;
    QLabel targetLabel;

    Impl(SignalIoConnectionView* self);
    void setConnectionMapItem(SignalIoConnectionMapItem* item);
    void onAddButtonClicked();
    void onRemoveButtonClicked();
    void updateTables();
    void setConnectionInfoToTableRow(int row, SignalIoConnection* connection);
};

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
    auto addButton = new PushButton(_("Add"));
    addButton->sigClicked().connect([&](){ onAddButtonClicked(); });
    hbox->addWidget(addButton);
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
    setColumnCount(6);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setTabKeyNavigation(true);
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    QTableWidgetItem* headerItem[6];
    headerItem[0] = new QTableWidgetItem(_("Output Device"));
    headerItem[1] = new QTableWidgetItem(_("Signal No."));
    headerItem[2] = new QTableWidgetItem;
    headerItem[3] = new QTableWidgetItem(_("Input Device"));
    headerItem[4] = new QTableWidgetItem(_("Signal No."));
    headerItem[5] = new QTableWidgetItem;
    for(int i=0; i < 6; ++i){
        setHorizontalHeaderItem(i, headerItem[i]);
    }

    auto hheader = horizontalHeader();
    hheader->setSectionResizeMode(QHeaderView::ResizeToContents);
    hheader->setStretchLastSection(true);

    auto vheader = verticalHeader();
    vheader->hide();
    vheader->setSectionResizeMode(QHeaderView::ResizeToContents);

    vbox->addWidget(this);
    self->setLayout(vbox);

    targetItemPicker.sigTargetItemChanged().connect(
        [&](SignalIoConnectionMapItem* item){
            setConnectionMapItem(item); });

    updateTables();
}


SignalIoConnectionView::~SignalIoConnectionView()
{
    delete impl;
}


void SignalIoConnectionView::Impl::setConnectionMapItem(SignalIoConnectionMapItem* item)
{
    targetItem = item;
    targetLabel.setText(item ? item->name().c_str() : "---");
}


void SignalIoConnectionView::Impl::onAddButtonClicked()
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

    bool added = false;
    auto connectionMap = targetItem->connectionMap();
    if(interConnection){
        connectionMap->appendConnection(interConnection);
        added = true;
    } else if(intraConnection){
        connectionMap->appendConnection(intraConnection);
        added = true;
    } else {
        showWarningDialog(_("There are no I/O devices in the world"));
    }
    if(added){
        updateTables();
    }
}


void SignalIoConnectionView::Impl::onRemoveButtonClicked()
{

}


void SignalIoConnectionView::Impl::updateTables()
{
    clearContents();
    clearSpans();

    int n = 0;
    if(!targetItem){
        setRowCount(1);
    } else{
        auto connections = targetItem->connectionMap();
        n = connections->numConnections();
        setRowCount(n + 1);
        for(int i=0; i < n; ++i){ 
            setConnectionInfoToTableRow(i, connections->connection(i));
        }
    }

    setSpan(n, 0, 1, 5);
    auto endItem = new QTableWidgetItem(_("END"));
    endItem->setFlags(Qt::ItemIsEnabled);
    endItem->setTextAlignment(Qt::AlignCenter);
    setItem(n, 0, endItem);

    auto emptyItem = new QTableWidgetItem;
    emptyItem->setFlags(Qt::NoItemFlags);
    setItem(n, 5, emptyItem);

    resizeColumnsToContents();
}


void SignalIoConnectionView::Impl::setConnectionInfoToTableRow(int row, SignalIoConnection* connection)
{
    constexpr Qt::ItemFlag SelectableFlag = Qt::NoItemFlags;
    
    for(int i=0; i < 2; ++i){
        auto device = connection->device(i);
        auto body = device->body();

        string name;
        if(device->name().empty()){
            name = body->name();
        } else {
            name = format("{0} - {1}", body->name(), device->name());
        }
        auto nameItem = new QTableWidgetItem(name.c_str());
        nameItem->setFlags(SelectableFlag | Qt::ItemIsEditable | Qt::ItemIsEnabled);
        nameItem->setTextAlignment(Qt::AlignCenter);
        setItem(row, i * 3, nameItem);

        auto index = connection->signalIndex(i);
        auto indexItem = new QTableWidgetItem(QString::number(index));
        indexItem->setFlags(SelectableFlag | Qt::ItemIsEditable | Qt::ItemIsEnabled);
        indexItem->setTextAlignment(Qt::AlignCenter);
        setItem(row, i * 3 + 1, indexItem);
    }

    auto arrowItem = new QTableWidgetItem("-->");
    arrowItem->setFlags(SelectableFlag);
    arrowItem->setTextAlignment(Qt::AlignCenter);
    setItem(row, 2, arrowItem);

    auto emptyItem = new QTableWidgetItem;
    emptyItem->setFlags(Qt::NoItemFlags);
    setItem(row, 5, emptyItem);
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
