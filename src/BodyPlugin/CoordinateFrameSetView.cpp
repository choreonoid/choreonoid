#include "CoordinateFrameSetView.h"
#include "CoordinateFrameSetItem.h"
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameContainer>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/Buttons>
#include <cnoid/EigenUtil>
#include <QBoxLayout>
#include <QLabel>
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

constexpr int NumColumns = 8;
constexpr int IdColumn = 0;
constexpr int XColumn = 1;
constexpr int YColumn = 2;
constexpr int ZColumn = 3;
constexpr int RXColumn = 4;
constexpr int RYColumn = 5;
constexpr int RZColumn = 6;
constexpr int NoteColumn = 7;

class FrameContainerModel : public QAbstractTableModel
{
public:
    CoordinateFrameContainerPtr frames;
    
    FrameContainerModel(QObject* parent);
    void setFrameContainer(CoordinateFrameContainer* frames);
    bool isValid() const { return frames != nullptr; }
    int numFrames() const { return frames ? frames->numFrames() : 0; }
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void addFrame(int row, CoordinateFrame* frame, bool doInsert);
    void removeFrames(QModelIndexList selected);
};

class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    CoordinateFrameSetView::Impl* view;
    
    CustomizedItemDelegate(CoordinateFrameSetView::Impl* view);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void updateEditorGeometry(
        QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};

}

namespace cnoid {

class CoordinateFrameSetView::Impl : public QTableView
{
public:
    CoordinateFrameSetView* self;
    TargetItemPicker<CoordinateFrameSetItem> targetItemPicker;
    CoordinateFrameSetItemPtr targetItem;
    QLabel targetLabel;
    FrameContainerModel* frameContainerModel;
    PushButton addButton;
    MenuManager contextMenuManager;

    Impl(CoordinateFrameSetView* self);
    void setCoordinateFrameSetItem(CoordinateFrameSetItem* item);
    void addFrameIntoCurrentIndex(bool doInsert);
    void addFrame(int index, bool doInsert);
    void removeSelectedFrames();
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(int row, QPoint globalPos);
};

}


FrameContainerModel::FrameContainerModel(QObject* parent)
    : QAbstractTableModel(parent)
{

}


void FrameContainerModel::setFrameContainer(CoordinateFrameContainer* frames)
{
    beginResetModel();
    this->frames = frames;
    endResetModel();
}


int FrameContainerModel::rowCount(const QModelIndex& parent) const
{
    int n = 0;
    if(!parent.isValid() && frames){
        n = frames->numFrames();
    }
    if(n == 0){ // to show an empty row
        n = 1;
    }
    return n;
}


int FrameContainerModel::columnCount(const QModelIndex& parent) const
{
    return NumColumns;
}
        

QVariant FrameContainerModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case 0:
                return "ID";
            case 1:
                return "X";
            case 2:
                return "Y";
            case 3:
                return "Z";
            case 4:
                return "R";
            case 5:
                return "P";
            case 6:
                return "Y";
            case 7:
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


QModelIndex FrameContainerModel::index(int row, int column, const QModelIndex& parent) const
{
    if(!frames || parent.isValid()){
        return QModelIndex();
    }
    if(row < frames->numFrames()){
        auto frame = frames->frame(row);
        return createIndex(row, column, frame);
    }
    return QModelIndex();
}
    

Qt::ItemFlags FrameContainerModel::flags(const QModelIndex& index) const
{
    if(!index.isValid()){
        return QAbstractTableModel::flags(index);
    }
    if(index.column() == IdColumn){
        return QAbstractTableModel::flags(index) | Qt::ItemIsEditable;
    }
    return QAbstractTableModel::flags(index);
}


QVariant FrameContainerModel::data(const QModelIndex& index, int role) const
{
    auto frame = static_cast<CoordinateFrame*>(index.internalPointer());

    if(!frame || !index.isValid()){
        return QVariant();
    }

    if(role == Qt::DisplayRole || role == Qt::EditRole){
        int column = index.column();
        if(column == IdColumn){
            return frame->idLabel().c_str();
        } else if(column >= XColumn && column <= ZColumn){
            return frame->T().translation()(column - XColumn);
        } else if(column >= RXColumn && column <= RZColumn){
            auto rpy = rpyFromRot(frame->T().linear());
            return rpy(column - RXColumn);
        }
    } else if(role == Qt::TextAlignmentRole){
        return Qt::AlignCenter;
    }
            
    return QVariant();
}


bool FrameContainerModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if(index.isValid() && role == Qt::EditRole){
        auto frame = static_cast<CoordinateFrame*>(index.internalPointer());
        int column = index.column();
        if(column == IdColumn){
            frames->resetId(frame, value.toString().toStdString());
            Q_EMIT dataChanged(index, index, {role});
        }
        /*
        else if(column >= XColumn && column <= ZColumn){
            frame->T().translation()(column - XColumn) = value.toDouble();
            Q_EMIT dataChanged(index, index, {role});
        } else if(column >= RXColumn && column <= RZColumn){
            auto rpy = rpyFromRot(frame->T());
            rpy[column - RXColumn] = value.toDouble();
            auto R = rotFromRpy(rpy);
            frames->T().linear() = R;
            Q_EMIT dataChanged(index, index, {role});
        }
        */
    }
    return false;
}


void FrameContainerModel::addFrame(int row, CoordinateFrame* frame, bool doInsert)
{
    if(frames){
        if(frames->numFrames() == 0){
            // Remove the empty row first
            beginRemoveRows(QModelIndex(), 0, 0);
            endRemoveRows();
        }
        int newFrameRow = doInsert ? row : row + 1;
        beginInsertRows(QModelIndex(), newFrameRow, newFrameRow);
        frames->insert(newFrameRow, frame);
        endInsertRows();
    }
}


void FrameContainerModel::removeFrames(QModelIndexList selected)
{
    if(frames){
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int row = index.row() - numRemoved;
            beginRemoveRows(QModelIndex(), row, row);
            frames->removeAt(row);
            ++numRemoved;
            endRemoveRows();
        }
        if(frames->numFrames() == 0){
            // This is necessary to show the empty row
            beginResetModel();
            endResetModel();
        }
    }
}
        

CustomizedItemDelegate::CustomizedItemDelegate(CoordinateFrameSetView::Impl* view)
    : QStyledItemDelegate(view),
      view(view)
{

}


QWidget* CustomizedItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QWidget* editor = nullptr;
    
    auto frame = static_cast<CoordinateFrame*>(index.internalPointer());
    
    switch(index.column()){
    case IdColumn:
    default:
        editor = QStyledItemDelegate::createEditor(parent, option, index);
        break;
    }

    return editor;
}


void CustomizedItemDelegate::updateEditorGeometry
(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    editor->setGeometry(option.rect);
}


void CustomizedItemDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    QStyledItemDelegate::setEditorData(editor, index);
}


void CustomizedItemDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    switch(index.column()){
    default:
        QStyledItemDelegate::setModelData(editor, model, index);
    }
}


void CoordinateFrameSetView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<CoordinateFrameSetView>(
        "CoordinateFrameSetView", N_("Coordinate Frames"), ViewManager::SINGLE_OPTIONAL);
}


CoordinateFrameSetView::CoordinateFrameSetView()
{
    impl = new Impl(this);
}


CoordinateFrameSetView::Impl::Impl(CoordinateFrameSetView* self)
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
    addButton.sigClicked().connect([&](){ addFrameIntoCurrentIndex(false); });
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

    frameContainerModel = new FrameContainerModel(this);
    setModel(frameContainerModel);

    vbox->addWidget(this);
    self->setLayout(vbox);

    targetItemPicker.sigTargetItemChanged().connect(
        [&](CoordinateFrameSetItem* item){
            setCoordinateFrameSetItem(item); });
}


CoordinateFrameSetView::~CoordinateFrameSetView()
{
    delete impl;
}


void CoordinateFrameSetView::Impl::setCoordinateFrameSetItem(CoordinateFrameSetItem* item)
{
    targetItem = item;

    if(item){
        targetLabel.setText(item->name().c_str());
        frameContainerModel->setFrameContainer(item->frames());
    } else {
        targetLabel.setText("---");
        frameContainerModel->setFrameContainer(nullptr);
    }
    addButton.setEnabled(targetItem != nullptr);
}


void CoordinateFrameSetView::Impl::addFrameIntoCurrentIndex(bool doInsert)
{
    auto current = selectionModel()->currentIndex();
    int index = current.isValid() ? current.row() : frameContainerModel->numFrames();
    addFrame(index, doInsert);
}


void CoordinateFrameSetView::Impl::addFrame(int index, bool doInsert)
{
    CoordinateFramePtr frame = new CoordinateFrame;
    frameContainerModel->addFrame(index, frame, doInsert);
}


void CoordinateFrameSetView::Impl::removeSelectedFrames()
{
    frameContainerModel->removeFrames(selectionModel()->selectedRows());
}


void CoordinateFrameSetView::Impl::keyPressEvent(QKeyEvent* event)
{
    bool processed = true;

    switch(event->key()){
    case Qt::Key_Escape:
        clearSelection();
        break;
    case Qt::Key_Insert:
        addFrameIntoCurrentIndex(true);
        break;
    case Qt::Key_Delete:
        removeSelectedFrames();
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

       
void CoordinateFrameSetView::Impl::mousePressEvent(QMouseEvent* event)
{
    QTableView::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        int row = rowAt(event->pos().y());
        if(row >= 0){
            showContextMenu(row, event->globalPos());
        }
    }
}


void CoordinateFrameSetView::Impl::showContextMenu(int row, QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);

    contextMenuManager.addItem(_("Add"))
        ->sigTriggered().connect([=](){ addFrame(row, false); });

    contextMenuManager.addItem(_("Remove"))
        ->sigTriggered().connect([=](){ removeSelectedFrames(); });
    
    contextMenuManager.popupMenu()->popup(globalPos);
}


bool CoordinateFrameSetView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "currentItem");
    return true;
}


bool CoordinateFrameSetView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "currentItem");
    return true;
}
