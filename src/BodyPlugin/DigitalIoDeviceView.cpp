#include "DigitalIoDeviceView.h"
#include "BodySelectionManager.h"
#include <cnoid/ViewManager>
#include <cnoid/BodyItem>
#include <cnoid/DigitalIoDevice>
#include <cnoid/Archive>
#include <cnoid/Format>
#include <QTableView>
#include <QHeaderView>
#include <QAbstractTableModel>
#include <QStyledItemDelegate>
#include <QBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QEvent>
#include "gettext.h"

using namespace std;
using namespace cnoid;

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
    bool isOnOffIconMode;

    IoDeviceModel(QObject* parent);
    void setOnOffIconMode(bool on);
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
    IoDeviceModel* ioDeviceModel;

    CustomizedItemDelegate(DigitalIoDeviceView::Impl* view);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setModelData(
        QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
    virtual void paint(
        QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& modelIndex) const override;
    virtual bool editorEvent(
        QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& index) override;
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


IoDeviceModel::IoDeviceModel(QObject* parent)
    : QAbstractTableModel(parent)
{
    isOnOffIconMode = true;
}


void IoDeviceModel::setOnOffIconMode(bool on)
{
    isOnOffIconMode = on;
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
                return QString(_("Input label"));
            case InStateColumn:
                return QString(_("Input"));
            case OutLabelColumn:
                return QString(_("Output label"));
            case OutStateColumn:
                return QString(_("Output"));
            }
        } else {
            return QString::number(section);
        }
    } else if(role == Qt::TextAlignmentRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case InLabelColumn:
            case OutLabelColumn:
                return static_cast<Qt::Alignment::Int>(Qt::AlignLeft | Qt::AlignVCenter);
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
    return Qt::ItemFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
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
            bool on;
            if(column == InStateColumn){
                on = ioDevice->in(index);
            } else {
                on = ioDevice->out(index);
            }
            if(isOnOffIconMode){
                return on;
            } else {
                return on ? QString(_("On")) : QString(_("Off"));
            }
            break;
        default:
            break;
        }
    } else if(role == Qt::TextAlignmentRole){
        switch(column){
        case InLabelColumn:
        case OutLabelColumn:
            return static_cast<Qt::Alignment::Int>(Qt::AlignLeft | Qt::AlignVCenter);
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
        } else if(column == InLabelColumn){
            ioDevice->setInLabel(index, value.toString().toStdString());
            Q_EMIT dataChanged(modelIndex, modelIndex, { role });
        } else if(column == OutLabelColumn){
            ioDevice->setOutLabel(index, value.toString().toStdString());
            Q_EMIT dataChanged(modelIndex, modelIndex, { role });
        }
    }
    return false;
}


CustomizedItemDelegate::CustomizedItemDelegate(DigitalIoDeviceView::Impl* view)
    : QStyledItemDelegate(view),
      ioDeviceModel(view->ioDeviceModel)
{

}


QWidget* CustomizedItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& modelIndex) const
{
    QWidget* editor = nullptr;

    if(!ioDeviceModel->isOnOffIconMode){
        int column = modelIndex.column();
        if(column == InStateColumn || column == OutStateColumn){
            int index = modelIndex.row();
            auto combo = new QComboBox(parent);
            combo->addItem(_("On"));
            combo->addItem(_("Off"));
            int comboIndex = 0;
            if(column == InStateColumn){
                comboIndex = ioDeviceModel->ioDevice->in(index) ? 0 : 1;
            } else {
                comboIndex = ioDeviceModel->ioDevice->out(index) ? 0 : 1;
            }
            combo->setCurrentIndex(comboIndex);
            editor = combo;
        }
    }

    if(!editor){
        editor = QStyledItemDelegate::createEditor(parent, option, modelIndex);
    }
    
    return editor;
}


void CustomizedItemDelegate::setModelData
(QWidget* editor, QAbstractItemModel* model, const QModelIndex& modelIndex) const
{
    bool done = false;

    if(!ioDeviceModel->isOnOffIconMode){
        int column = modelIndex.column();
        if(column == InStateColumn || column == OutStateColumn){
            if(auto combo = dynamic_cast<QComboBox*>(editor)){
                bool on = (combo->currentIndex() == 0) ? true : false;
                model->setData(modelIndex, on);
                done = true;
            }
        }
    }
    
    if(!done){
        QStyledItemDelegate::setModelData(editor, model, modelIndex);
    }
}


void CustomizedItemDelegate::paint
(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& modelIndex) const 
{
    bool done = false;
    
    if(ioDeviceModel->isOnOffIconMode){
        int column = modelIndex.column();
        if(column == InStateColumn || column == OutStateColumn){
            bool on = modelIndex.data().toBool();
            QRect iconRect = option.rect;
            static QIcon onIcon(":Body/icon/digital-io-on.svg");
            static QIcon offIcon(":Body/icon/digital-io-off.svg");
            if(on){
                onIcon.paint(painter, iconRect);
            } else {
                offIcon.paint(painter, iconRect);
            }
            done = true;
        }
    }

    if(!done){
        QStyledItemDelegate::paint(painter, option, modelIndex);
    }
}


bool CustomizedItemDelegate::editorEvent
(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& modelIndex)
{
    bool done = false;
    
    if(ioDeviceModel->isOnOffIconMode){
        int column = modelIndex.column();
        if(column == InStateColumn || column == OutStateColumn){
            if(event->type() == QEvent::MouseButtonPress){
                bool on = modelIndex.data().toBool();
                model->setData(modelIndex, !on, Qt::EditRole);
                event->accept();
            }
            done = true;
        }
    }
    return done;
}



void DigitalIoDeviceView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<DigitalIoDeviceView>(
        N_("DigitalIoDeviceView"), N_("I/O device"));
}


DigitalIoDeviceView::DigitalIoDeviceView()
{
    impl = new Impl(this);
}


DigitalIoDeviceView::Impl::Impl(DigitalIoDeviceView* self)
    : self(self)
{
    self->setDefaultLayoutArea(BottomCenterArea);

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
                    formatC("{0} - {1}", bodyItem->displayName(), deviceName).c_str());
            } else {
                targetLabel.setText(
                    formatR(_("{0} (No I/O)"), bodyItem->displayName()).c_str());
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
    // Restore-only option for customization written in the builtin project file
    bool on;
    if(archive.read("use_on_off_icon", on)){
        impl->ioDeviceModel->setOnOffIconMode(on);
    }
    return true;
}

