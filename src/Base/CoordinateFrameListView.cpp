#include "CoordinateFrameListView.h"
#include "CoordinateFrameListItem.h"
#include "PositionEditManager.h"
#include <cnoid/CoordinateFrameList>
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

constexpr int NumColumns = 3;
constexpr int IdColumn = 0;
constexpr int NoteColumn = 1;
constexpr int PositionColumn = 2;

class FrameListModel : public QAbstractTableModel
{
public:
    CoordinateFrameListPtr frames;
    CoordinateFramePtr defaultFrame;
    QFont monoFont;
    bool isDefaultFrameEnabled_;
    
    FrameListModel(QObject* parent);
    void setDefaultFrameEnabled(bool on);
    bool isDefaultFrameEnabled() const;
    void setFrameList(CoordinateFrameList* frames);
    bool isValid() const;
    int numFrames() const;
    CoordinateFrame* frameAtRow(int row);
    int rowOfFrame(CoordinateFrame* frame) const;
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void addFrame(int row, CoordinateFrame* frame, bool doInsert);
    void removeFrames(QModelIndexList selected);
    void notifyFramePositionUpdate(CoordinateFrame* frame);
};

class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    CoordinateFrameListView::Impl* view;
    
    CustomizedItemDelegate(CoordinateFrameListView::Impl* view);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void updateEditorGeometry(
        QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};

}

namespace cnoid {

class CoordinateFrameListView::Impl : public QTableView, public AbstractPositionEditTarget
{
public:
    CoordinateFrameListView* self;
    TargetItemPicker<CoordinateFrameListItem> targetItemPicker;
    CoordinateFrameListItemPtr targetItem;
    CoordinateFrameListPtr frames;
    FrameListModel* frameListModel;
    QLabel targetLabel;
    PushButton addButton;
    MenuManager contextMenuManager;

    // For the position editing by external editors
    PositionEditManager* positionEditManager;
    CoordinateFramePtr frameBeingEditedOutside;
    Signal<void(const Position& T)> sigPositionChanged_;
    Signal<void()> sigPositionEditTargetExpired_;

    Impl(CoordinateFrameListView* self);
    void setCoordinateFrameListItem(CoordinateFrameListItem* item);
    void addFrameIntoCurrentIndex(bool doInsert);
    void addFrame(int row, bool doInsert);
    void removeSelectedFrames();
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(int row, QPoint globalPos);
    virtual void selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) override;

    void startExternalPositionEditing(CoordinateFrame* frame);
    void stopExternalPositionEditing();
    virtual Referenced* getPositionObject() override;
    virtual std::string getPositionName() const override;
    virtual Position getPosition() const override;
    virtual bool isEditable() const override;
    virtual bool setPosition(const Position& T) override;
    virtual SignalProxy<void(const Position& T)> sigPositionChanged() override;
    virtual SignalProxy<void()> sigPositionEditTargetExpired() override;
};

}


FrameListModel::FrameListModel(QObject* parent)
    : QAbstractTableModel(parent),
      monoFont("Monospace")
{
    monoFont.setStyleHint(QFont::TypeWriter);
    setDefaultFrameEnabled(true);
}


void FrameListModel::setDefaultFrameEnabled(bool on)
{
    isDefaultFrameEnabled_ = on;
    defaultFrame = new CoordinateFrame(0, frames);
}


bool FrameListModel::isDefaultFrameEnabled() const
{
    return isDefaultFrameEnabled_;
}
    

void FrameListModel::setFrameList(CoordinateFrameList* frames)
{
    beginResetModel();
    this->frames = frames;
    setDefaultFrameEnabled(isDefaultFrameEnabled_);
    endResetModel();
}


bool FrameListModel::isValid() const
{
    return frames != nullptr;
}


int FrameListModel::numFrames() const
{
    if(frames){
        int n = frames->numFrames();
        return isDefaultFrameEnabled_ ? (n + 1) : n;
    }
    return 0;
}


CoordinateFrame* FrameListModel::frameAtRow(int row)
{
    if(isDefaultFrameEnabled_){
        if(row == 0){
            return defaultFrame;
        }
        return frames->frameAt(row - 1);
    } else {
        return frames->frameAt(row);
    }
}


int FrameListModel::rowOfFrame(CoordinateFrame* frame) const
{
    if(frames && frame != defaultFrame){
        int row = frames->indexOf(frame);
        if(isDefaultFrameEnabled_){
            row += 1;
        }
        return row;
    }
    return -1;
}


int FrameListModel::rowCount(const QModelIndex& parent) const
{
    int n = 0;
    if(!parent.isValid()){
        n = numFrames();
    }
    if(n == 0){ // to show an empty row
        n = 1;
    }
    return n;
}


int FrameListModel::columnCount(const QModelIndex& parent) const
{
    return NumColumns;
}
        

QVariant FrameListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case IdColumn:
                return "ID";
            case NoteColumn:
                return _("Note");
            case PositionColumn:
                return _("Position");
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


QModelIndex FrameListModel::index(int row, int column, const QModelIndex& parent) const
{
    if(!frames || parent.isValid()){
        return QModelIndex();
    }
    if(row < numFrames()){
        auto frame = const_cast<FrameListModel*>(this)->frameAtRow(row);
        return createIndex(row, column, frame);
    }
    return QModelIndex();
}
    

Qt::ItemFlags FrameListModel::flags(const QModelIndex& index) const
{
    auto flags = QAbstractTableModel::flags(index);
    if(index.isValid() && index.column() != PositionColumn){
        if(!isDefaultFrameEnabled_ || index.row() != 0){
            flags = flags | Qt::ItemIsEditable;
        }
    }
    return flags;
}


QVariant FrameListModel::data(const QModelIndex& index, int role) const
{
    auto frame = static_cast<CoordinateFrame*>(index.internalPointer());

    if(!frame || !index.isValid()){
        return QVariant();
    }

    int column = index.column();
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        if(column == IdColumn){
            return frame->id().label().c_str();

        } else if(column == NoteColumn){
            return frame->note().c_str();

        } else if(column == PositionColumn){
            auto p = frame->T().translation();
            auto rpy = degree(rpyFromRot(frame->T().linear()));
            return format("{0: 1.3f} {1: 1.3f} {2: 1.3f} {3: 6.1f} {4: 6.1f} {5: 6.1f}",
                          p.x(), p.y(), p.z(), rpy[0], rpy[1], rpy[2]).c_str();
        }
    } else if(role == Qt::TextAlignmentRole){
        if(column == NoteColumn){
            return (Qt::AlignLeft + Qt::AlignVCenter);
        } else {
            return Qt::AlignCenter;
        }
    } else if(role == Qt::FontRole){
        if(column == PositionColumn){
            return monoFont;
        }
    }
            
    return QVariant();
}


bool FrameListModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if(index.isValid() && role == Qt::EditRole){
        auto frame = static_cast<CoordinateFrame*>(index.internalPointer());
        if(frame == defaultFrame){
            return false;
        }
        int column = index.column();
        if(column == IdColumn){
            bool isInt;
            auto stringId = value.toString();
            int intId = stringId.toInt(&isInt);
            if(isInt){
                frames->resetId(frame, intId);
            } else {
                frames->resetId(frame, stringId.toStdString());
            }
            Q_EMIT dataChanged(index, index, {role});
        } else if(column == NoteColumn){
            frame->setNote(value.toString().toStdString());
            Q_EMIT dataChanged(index, index, {role});
        }
    }
    return false;
}


void FrameListModel::addFrame(int row, CoordinateFrame* frame, bool doInsert)
{
    if(frames){
        int newFrameRow = doInsert ? row : row + 1;
        if(newFrameRow != 0 || !defaultFrame){
            if(numFrames() == 0){
                // Remove the empty row first
                beginRemoveRows(QModelIndex(), 0, 0);
                endRemoveRows();
            }
            beginInsertRows(QModelIndex(), newFrameRow, newFrameRow);
            int newFrameIndex = isDefaultFrameEnabled_ ? (newFrameRow - 1) : newFrameRow;
            frames->insert(newFrameIndex, frame);
            endInsertRows();
        }
    }
}


void FrameListModel::removeFrames(QModelIndexList selected)
{
    if(frames){
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int row = index.row() - numRemoved;
            if(!isDefaultFrameEnabled_ || row != 0){
                beginRemoveRows(QModelIndex(), row, row);
                int frameIndex = isDefaultFrameEnabled_ ? row - 1 : row;
                frames->removeAt(frameIndex);
                ++numRemoved;
                endRemoveRows();
            }
        }
        if(frames->numFrames() == 0){
            // This is necessary to show the empty row
            beginResetModel();
            endResetModel();
        }
    }
}


void FrameListModel::notifyFramePositionUpdate(CoordinateFrame* frame)
{
    int row = rowOfFrame(frame);
    if(row >= 0){
        auto modelIndex = index(row, PositionColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
}
        

CustomizedItemDelegate::CustomizedItemDelegate(CoordinateFrameListView::Impl* view)
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


void CoordinateFrameListView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<CoordinateFrameListView>(
        "CoordinateFrameListView", N_("Coordinate Frames"), ViewManager::SINGLE_OPTIONAL);
}


CoordinateFrameListView::CoordinateFrameListView()
{
    impl = new Impl(this);
}


CoordinateFrameListView::Impl::Impl(CoordinateFrameListView* self)
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
        /* QAbstractItemView::SelectedClicked | */
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    frameListModel = new FrameListModel(this);
    frameListModel->setDefaultFrameEnabled(true);
    setModel(frameListModel);

    auto hheader = horizontalHeader();
    hheader->setSectionResizeMode(IdColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(NoteColumn, QHeaderView::Stretch);
    hheader->setSectionResizeMode(PositionColumn, QHeaderView::ResizeToContents);
    auto vheader = verticalHeader();
    vheader->setSectionResizeMode(QHeaderView::ResizeToContents);
    vheader->hide();

    connect(this, SIGNAL(clicked(const QModelIndex&)),
            self, SLOT(onTableItemClicked(const QModelIndex&)));
    
    vbox->addWidget(this);
    self->setLayout(vbox);

    targetItemPicker.sigTargetItemChanged().connect(
        [&](CoordinateFrameListItem* item){
            setCoordinateFrameListItem(item); });

    positionEditManager = PositionEditManager::instance();
}


CoordinateFrameListView::~CoordinateFrameListView()
{
    delete impl;
}


void CoordinateFrameListView::onActivated()
{

}


void CoordinateFrameListView::onDeactivated()
{
    impl->stopExternalPositionEditing();
}


void CoordinateFrameListView::Impl::setCoordinateFrameListItem(CoordinateFrameListItem* item)
{
    stopExternalPositionEditing();

    targetItem = item;

    if(item){
        targetLabel.setText(item->name().c_str());
        frames = item->frameList();
        frameListModel->setFrameList(frames);
    } else {
        targetLabel.setText("---");
        frames = nullptr;
        frameListModel->setFrameList(nullptr);
    }
    addButton.setEnabled(targetItem != nullptr);
}


void CoordinateFrameListView::Impl::addFrameIntoCurrentIndex(bool doInsert)
{
    auto current = selectionModel()->currentIndex();
    int row = current.isValid() ? current.row() : frameListModel->numFrames();
    addFrame(row, doInsert);
}


void CoordinateFrameListView::Impl::addFrame(int row, bool doInsert)
{
    if(frames){
        auto id = frames->createNextId();
        CoordinateFramePtr frame = new CoordinateFrame(id);
        frameListModel->addFrame(row, frame, doInsert);
        resizeColumnToContents(IdColumn);
        resizeColumnToContents(PositionColumn);
    }
}


void CoordinateFrameListView::Impl::removeSelectedFrames()
{
    frameListModel->removeFrames(selectionModel()->selectedRows());
}


void CoordinateFrameListView::Impl::keyPressEvent(QKeyEvent* event)
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

       
void CoordinateFrameListView::Impl::mousePressEvent(QMouseEvent* event)
{
    QTableView::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        int row = rowAt(event->pos().y());
        if(row >= 0){
            showContextMenu(row, event->globalPos());
        }
    }
}


void CoordinateFrameListView::Impl::showContextMenu(int row, QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);

    contextMenuManager.addItem(_("Add"))
        ->sigTriggered().connect([=](){ addFrame(row, false); });

    if(!frameListModel->isDefaultFrameEnabled() || row != 0){
        contextMenuManager.addItem(_("Remove"))
            ->sigTriggered().connect([=](){ removeSelectedFrames(); });
    }
    
    contextMenuManager.popupMenu()->popup(globalPos);
}


void CoordinateFrameListView::Impl::selectionChanged
(const QItemSelection& selected, const QItemSelection& deselected)
{
    QTableView::selectionChanged(selected, deselected);

    auto indexes = selected.indexes();
    if(indexes.empty()){
        startExternalPositionEditing(nullptr);
    } else {
        auto frame = static_cast<CoordinateFrame*>(indexes.front().internalPointer());
        startExternalPositionEditing(frame);
    }
}


void CoordinateFrameListView::onTableItemClicked(const QModelIndex& index)
{
    auto frame = static_cast<CoordinateFrame*>(index.internalPointer());
    impl->startExternalPositionEditing(frame);
}


void CoordinateFrameListView::Impl::startExternalPositionEditing(CoordinateFrame* frame)
{
    if(frame != frameBeingEditedOutside){
        stopExternalPositionEditing();

        frameBeingEditedOutside = frame;

        if(frame){
            positionEditManager->requestPositionEdit(this);
        }
    }
}


void CoordinateFrameListView::Impl::stopExternalPositionEditing()
{
    if(frameBeingEditedOutside){
        sigPositionEditTargetExpired_();
    }
    frameBeingEditedOutside = nullptr;
}


Referenced* CoordinateFrameListView::Impl::getPositionObject()
{
    return frameBeingEditedOutside;
}


std::string CoordinateFrameListView::Impl::getPositionName() const
{
    if (frameBeingEditedOutside) {
        auto& id = frameBeingEditedOutside->id();
        auto& note = frameBeingEditedOutside->note();
        if (id.isString() || note.empty()) {
            return format("{0}: {1}", targetItem->name(), id.label());
        } else {
            return format("{0}: {1} ( {2} )", targetItem->name(), id.label(), note);
        }
    }
    return string();
}


Position CoordinateFrameListView::Impl::getPosition() const
{
    if(frameBeingEditedOutside){
        return frameBeingEditedOutside->T();
    }
    return Position::Identity();
}


bool CoordinateFrameListView::Impl::isEditable() const
{
    if(frameBeingEditedOutside && frameBeingEditedOutside->id() != 0){
        return true;
    }
    return false;
}


bool CoordinateFrameListView::Impl::setPosition(const Position& T)
{
    if(frameBeingEditedOutside){
        frameBeingEditedOutside->T() = T;
        frameListModel->notifyFramePositionUpdate(frameBeingEditedOutside);
        sigPositionChanged_(T);
        return true;
    }
    return false;
}


SignalProxy<void(const Position& T)> CoordinateFrameListView::Impl::sigPositionChanged()
{
    return sigPositionChanged_;
}


SignalProxy<void()> CoordinateFrameListView::Impl::sigPositionEditTargetExpired()
{
    return sigPositionEditTargetExpired_;
}


bool CoordinateFrameListView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "currentItem");
    return true;
}


bool CoordinateFrameListView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "currentItem");
    return true;
}
