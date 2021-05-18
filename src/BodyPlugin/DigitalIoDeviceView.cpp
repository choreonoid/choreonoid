#include "DigitalIoDeviceView.h"
#include "BodySelectionManager.h"
#include <cnoid/ViewManager>
#include <cnoid/BodyItem>
#include <cnoid/DigitalIoDevice>
#include <QTableView>
#include <QHeaderView>
#include <QAbstractTableModel>
#include <QStyledItemDelegate>
#include <QBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

constexpr int NumColumns = 4;
constexpr int InLabelColumn = 0;
constexpr int InStateColumn = 1;
constexpr int OutLabelColumn = 2;
constexpr int OutStateColumn = 3;

class IoDeviceModel : public QAbstractTableModel
{
public:
    DigitalIoDevicePtr ioDevice;
    ScopedConnection ioDeviceConnection;

    IoDeviceModel(QObject* parent);
    void setIoDevice(DigitalIoDevice* ioDevice);
    void onIoDeviceStateChanged();    
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
};

class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    IoDeviceModel* model;

    CustomizedItemDelegate(DigitalIoDeviceView::Impl* view);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};

}

namespace cnoid {

class DigitalIoDeviceView::Impl : public QTableView
{
public:
    DigitalIoDeviceView* self;
    BodyItem* targetBodyItem;
    BodySelectionManager* bodySelectionManager;
    ScopedConnection bodySelectionConnection;
    IoDeviceModel* ioDeviceModel;
    ScopedConnection bodyItemConnection;
    QLabel targetLabel;

    Impl(DigitalIoDeviceView* self);
    void onActivated();
    void onCurrentBodyItemChanged(BodyItem* bodyItem);
};

}


namespace {

IoDeviceModel::IoDeviceModel(QObject* parent)
    : QAbstractTableModel(parent)
{

}


void IoDeviceModel::setIoDevice(DigitalIoDevice* ioDevice)
{
    beginResetModel();
    this->ioDevice = ioDevice;
    if(ioDevice){
        ioDeviceConnection =
            ioDevice->sigStateChanged().connect(
                [&](){ onIoDeviceStateChanged(); });
    } else {
        ioDeviceConnection.disconnect();
    }
    endResetModel();
}


void IoDeviceModel::onIoDeviceStateChanged()
{
    const int n = ioDevice->numSignalLines();
    if(n > 0){
        const int lastRow = n - 1;
        Q_EMIT dataChanged(index(0, InStateColumn), index(lastRow, InStateColumn), {Qt::DisplayRole});
        Q_EMIT dataChanged(index(0, OutStateColumn), index(lastRow, OutStateColumn), {Qt::DisplayRole});
    }
}


int IoDeviceModel::rowCount(const QModelIndex& parent) const
{
    return ioDevice ? ioDevice->numSignalLines() : 0;
}


int IoDeviceModel::columnCount(const QModelIndex& parent) const
{
    return NumColumns;
}
    

QVariant IoDeviceModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case InLabelColumn:
                return _("Input label");
            case InStateColumn:
                return _("Input");
            case OutLabelColumn:
                return _("Output label");
            case OutStateColumn:
                return _("Output");
            }
        } else {
            return QString::number(section);
        }
    } else if(role == Qt::TextAlignmentRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case InLabelColumn:
            case OutLabelColumn:
                return Qt::AlignLeft + Qt::AlignVCenter;
            default:
                return Qt::AlignCenter;
            }
        }
    }
    return QVariant();
}


QModelIndex IoDeviceModel::index(int row, int column, const QModelIndex& parent) const
{
    if(!ioDevice || parent.isValid()){
        return QModelIndex();
    }
    if(row < ioDevice->numSignalLines()){
        return createIndex(row, column);
    }
    return QModelIndex();
}
    

Qt::ItemFlags IoDeviceModel::flags(const QModelIndex& index) const
{
    if(index.column() == InStateColumn || index.column() == OutStateColumn){
        return QAbstractTableModel::flags(index) | Qt::ItemIsEditable;
    }
    return QAbstractTableModel::flags(index);
}


QVariant IoDeviceModel::data(const QModelIndex& modelIndex, int role) const
{
    int index = modelIndex.row();
    int column = modelIndex.column();
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(column){
        case InLabelColumn:
            return ioDevice->inLabel(index).c_str();
        case OutLabelColumn:
            return ioDevice->outLabel(index).c_str();
        case InStateColumn:
        case OutStateColumn:
        {
            bool on;
            if(column == InStateColumn){
                on = ioDevice->in(index);
            } else {
                on = ioDevice->out(index);
            }
            return on ? _("On") : _("Off");
        }
        default:
            break;
        }
    } else if(role == Qt::TextAlignmentRole){
        switch(column){
        case InLabelColumn:
        case OutLabelColumn:
            return (Qt::AlignLeft + Qt::AlignVCenter);
        default:
            return Qt::AlignCenter;
        }
    }
    return QVariant();
}


bool IoDeviceModel::setData(const QModelIndex& modelIndex, const QVariant& value, int role)
{
    if(modelIndex.isValid() && role == Qt::EditRole){
        int index = modelIndex.row();
        int column = modelIndex.column();
        if(column == InStateColumn){
            ioDevice->setIn(index, value.toBool());
            Q_EMIT dataChanged(modelIndex, modelIndex, {role});
        } else if(column == OutStateColumn){
            ioDevice->setOut(index, value.toBool());
            Q_EMIT dataChanged(modelIndex, modelIndex, {role});
        }
    }
    return false;
}


CustomizedItemDelegate::CustomizedItemDelegate(DigitalIoDeviceView::Impl* view)
    : QStyledItemDelegate(view),
      model(view->ioDeviceModel)
{

}


QWidget* CustomizedItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& modelIndex) const
{
    QWidget* editor = nullptr;
    int index = modelIndex.row();
    int column = modelIndex.column();
    switch(column){
    case InStateColumn:
    case OutStateColumn:
    {
        auto combo = new QComboBox(parent);
        combo->addItem(_("On"));
        combo->addItem(_("Off"));
        bool on;
        if(column == InStateColumn){
            on = model->ioDevice->in(index) ? 0 : 1;
        } else {
            on = model->ioDevice->out(index) ? 0 : 1;
        }
        combo->setCurrentIndex(on);
        editor = combo;
        break;
    }
    default:
        break;
    }
    if(!editor){
        editor = QStyledItemDelegate::createEditor(parent, option, modelIndex);
    }
    return editor;
}


void CustomizedItemDelegate::setModelData
(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    switch(index.column()){
    case InStateColumn:
    case OutStateColumn:
        if(auto combo = dynamic_cast<QComboBox*>(editor)){
            bool on = (combo->currentIndex() == 0) ? true : false;
            model->setData(index, on);
        }
        break;
    cefault:
        QStyledItemDelegate::setModelData(editor, model, index);
    }
}

}


void DigitalIoDeviceView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<DigitalIoDeviceView>(
        "DigitalIoDeviceView", N_("I/O device"), ViewManager::SINGLE_OPTIONAL);
}


DigitalIoDeviceView::DigitalIoDeviceView()
{
    impl = new Impl(this);
}


DigitalIoDeviceView::Impl::Impl(DigitalIoDeviceView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::BOTTOM);

    int hs = self->style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
    int vs = self->style()->pixelMetric(QStyle::PM_LayoutVerticalSpacing);
    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    vbox->addSpacing(vs);
    auto hbox = new QHBoxLayout;
    hbox->setSpacing(0);
    hbox->addSpacing(hs);
    targetLabel.setStyleSheet("font-weight: bold");
    hbox->addWidget(&targetLabel, 0, Qt::AlignVCenter);
    hbox->addStretch();
    vbox->addLayout(hbox);
    vbox->addSpacing(vs);

    auto hframe = new QFrame;
    hframe->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    vbox->addWidget(hframe);
    setFrameShape(QFrame::NoFrame);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setTabKeyNavigation(true);
    setCornerButtonEnabled(false);
    setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    setEditTriggers(
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::SelectedClicked |
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    ioDeviceModel = new IoDeviceModel(this);
    setModel(ioDeviceModel);
    setItemDelegate(new CustomizedItemDelegate(this));

    auto hheader = horizontalHeader();
    hheader->setSectionResizeMode(InLabelColumn, QHeaderView::Stretch);
    hheader->setSectionResizeMode(InStateColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(OutLabelColumn, QHeaderView::Stretch);
    hheader->setSectionResizeMode(OutStateColumn, QHeaderView::ResizeToContents);
    auto vheader = verticalHeader();
    vheader->setSectionResizeMode(QHeaderView::ResizeToContents);

    vbox->addWidget(this);
    self->setLayout(vbox);

    targetBodyItem = nullptr;
    bodySelectionManager = BodySelectionManager::instance();
}


DigitalIoDeviceView::~DigitalIoDeviceView()
{
    delete impl;
}


void DigitalIoDeviceView::onActivated()
{
    impl->onActivated();
}


void DigitalIoDeviceView::Impl::onActivated()
{
    bodySelectionConnection =
        bodySelectionManager->sigCurrentBodyItemChanged().connect(
            [&](BodyItem* bodyItem){ onCurrentBodyItemChanged(bodyItem); });

    onCurrentBodyItemChanged(bodySelectionManager->currentBodyItem());
}


void DigitalIoDeviceView::onDeactivated()
{
    impl->bodySelectionConnection.disconnect();
    impl->onCurrentBodyItemChanged(nullptr);
}


void DigitalIoDeviceView::Impl::onCurrentBodyItemChanged(BodyItem* bodyItem)
{
    if(bodyItem != targetBodyItem){
        DigitalIoDevice* ioDevice = nullptr;
        if(!bodyItem){
            targetLabel.setText("---");
        } else {
            ioDevice = bodyItem->body()->findDevice<DigitalIoDevice>();
            string deviceName;
            if(ioDevice){
                if(!ioDevice->name().empty()){
                    deviceName = ioDevice->name();
                } else {
                    deviceName = "Digital I/O";
                }
                targetLabel.setText(
                    format("{0} - {1}", bodyItem->displayName(), deviceName).c_str());
            } else {
                targetLabel.setText(
                    format(_("{0} (No I/O)"), bodyItem->displayName()).c_str());
            }

        }
        ioDeviceModel->setIoDevice(ioDevice);
        targetBodyItem = bodyItem;
    }
}


bool DigitalIoDeviceView::storeState(Archive& archive)
{
    return true;
}


bool DigitalIoDeviceView::restoreState(const Archive& archive)
{
    return true;
}

