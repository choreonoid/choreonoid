#include "CoordinateFrameListView.h"
#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "TargetItemPicker.h"
#include "LocatableItem.h"
#include "Buttons.h"
#include <cnoid/CoordinateFrameList>
#include <cnoid/EigenUtil>
#include <cnoid/ConnectionSet>
#include <QBoxLayout>
#include <QLabel>
#include <QTableView>
#include <QHeaderView>
#include <QAbstractTableModel>
#include <QStyledItemDelegate>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QGuiApplication>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

constexpr int NumColumns = 5;
constexpr int IdColumn = 0;
constexpr int NoteColumn = 1;
constexpr int PositionColumn = 2;
constexpr int GlobalCheckColumn = 3;
constexpr int VisibleCheckColumn = 4;

class FrameListModel : public QAbstractTableModel
{
public:
    CoordinateFrameListView::Impl* view;
    CoordinateFrameListItemPtr frameListItem;
    CoordinateFrameListPtr frameList;
    ScopedConnectionSet frameListConnections;
    QFont monoFont;
    
    FrameListModel(CoordinateFrameListView::Impl* view);
    void setFrameListItem(CoordinateFrameListItem* frameListItem);
    bool isValid() const;
    int numFrames() const;
    CoordinateFrame* frameAt(const QModelIndex& index) const;
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void addFrame(int row, CoordinateFrame* frame, bool doInsert);
    void removeFrames(QModelIndexList selected);
    void onFrameAdded(int frameIndex);
    void onFrameRemoved(int frameIndex);
    void onFrameUpdated(int frameIndex, int flags);
    void onFrameMarkerViisibilityChanged(int frameIndex);
};

class CheckItemDelegate : public QStyledItemDelegate
{
public:
    CoordinateFrameListView::Impl* view;
    bool isValid;
    
    CheckItemDelegate(CoordinateFrameListView::Impl* view);
    virtual void paint(
        QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual bool editorEvent(
        QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& index) override;
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};

}

namespace cnoid {

class CoordinateFrameListView::Impl : public QTableView
{
public:
    CoordinateFrameListView* self;
    TargetItemPicker<CoordinateFrameListItem> targetItemPicker;
    CoordinateFrameListItemPtr targetItem;
    bool isIndependentItemizationListSupported;
    CoordinateFrameListPtr frameList;
    LocationProxyPtr locationProxy;
    FrameListModel* frameListModel;
    CheckItemDelegate* globalCheckDelegate;
    ReferencedPtr transientMarkerHolder;
    QLabel targetLabel;
    PushButton addButton;
    MenuManager contextMenuManager;
    bool isSelectionChangedAlreadyCalled;

    Impl(CoordinateFrameListView* self);
    void setCoordinateFrameListItem(CoordinateFrameListItem* item);
    void addFrameIntoCurrentIndex(bool doInsert);
    void addFrame(int row, bool doInsert);
    void removeSelectedFrames();
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(int row, QPoint globalPos);
    virtual void selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) override;
    void startLocationEditing(const QModelIndex& modelIndex);
    void stopLocationEditing();
};

}


namespace {

FrameListModel::FrameListModel(CoordinateFrameListView::Impl* view)
    : QAbstractTableModel(view),
      view(view),
      monoFont("Monospace")
{
    monoFont.setStyleHint(QFont::TypeWriter);
}


void FrameListModel::setFrameListItem(CoordinateFrameListItem* frameListItem)
{
    beginResetModel();

    this->frameListItem = frameListItem;
    if(frameListItem){
        this->frameList = frameListItem->frameList();
    } else {
        this->frameList = nullptr;
    }

    frameListConnections.disconnect();
    if(frameList){
        frameListConnections.add(
            frameList->sigFrameAdded().connect(
                [&](int index){ onFrameAdded(index); }));
        frameListConnections.add(
            frameList->sigFrameRemoved().connect(
                [&](int index, CoordinateFrame*){ onFrameRemoved(index); }));
        frameListConnections.add(
            frameList->sigFrameUpdated().connect(
                [&](int index, int flags){ onFrameUpdated(index, flags); }));
        frameListConnections.add(
            frameListItem->sigFrameMarkerVisibilityChanged().connect(
                [&](int index, bool /* on */){ onFrameMarkerViisibilityChanged(index); }));
    }
            
    endResetModel();
}


bool FrameListModel::isValid() const
{
    return frameList != nullptr;
}


int FrameListModel::numFrames() const
{
    if(frameList){
        return frameList->numFrames();
    }
    return 0;
}


CoordinateFrame* FrameListModel::frameAt(const QModelIndex& index) const
{
    if(!index.isValid()){
        return nullptr;
    }
    return frameList->frameAt(index.row());
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
                return " ID ";
            case NoteColumn:
                return _("Note");
            case PositionColumn:
                return _("Position");
            case GlobalCheckColumn:
                return "G";
            case VisibleCheckColumn:
                return "V";
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
    if(!frameList || parent.isValid()){
        return QModelIndex();
    }
    if(row < numFrames()){
        return createIndex(row, column);
    }
    return QModelIndex();
}
    

Qt::ItemFlags FrameListModel::flags(const QModelIndex& index) const
{
    auto flags = QAbstractTableModel::flags(index);
    if(index.isValid()){
        int column = index.column();
        if(column != PositionColumn){
            if(column == VisibleCheckColumn){
                flags |= Qt::ItemIsEditable;
            } else {
                if(index.row() > 0 || (frameList && !frameList->hasFirstElementAsDefaultFrame())){
                    flags |= Qt::ItemIsEditable;
                }
            }
        }
    }
    return flags;
}


QVariant FrameListModel::data(const QModelIndex& index, int role) const
{
    auto frame = frameAt(index);
    if(!frame){
        return QVariant();
    }
    int column = index.column();
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(column){
        case IdColumn:
            return frame->id().label().c_str();

        case NoteColumn:
            return frame->note().c_str();

        case PositionColumn: {
            Isometry3 T;
            if(frameListItem->getRelativeFramePosition(frame, T)){
                auto p = T.translation();
                auto rpy = degree(rpyFromRot(T.linear()));
                return format("{0: 1.3f} {1: 1.3f} {2: 1.3f} {3: 6.1f} {4: 6.1f} {5: 6.1f}",
                              p.x(), p.y(), p.z(), rpy[0], rpy[1], rpy[2]).c_str();
            } else {
                return " -.---  -.---  -.---    -.-    -.-    -.-";
            }
        }
        case GlobalCheckColumn:
            return frame->isGlobal();

        case VisibleCheckColumn:
            return frameListItem->isFrameMarkerVisible(frame);

        default:
            break;
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
    if(!index.isValid()){
        return false;
    }

    auto frameIndex = index.row();
    if(frameIndex == 0 && frameList && frameList->hasFirstElementAsDefaultFrame() &&
       index.column() != VisibleCheckColumn){
        return false;
    }
    int updateFlags = 0;
    auto frame = frameList->frameAt(frameIndex);

    if(role == Qt::EditRole){
        switch(index.column()){
        case IdColumn: {
            bool isInt;
            auto stringId = value.toString();
            int intId = stringId.toInt(&isInt);
            if(isInt){
                frameList->resetId(frame, intId);
            } else {
                frameList->resetId(frame, stringId.toStdString());
            }
            updateFlags = CoordinateFrame::IdUpdate;
            break;
        }
        case NoteColumn:
            frame->setNote(value.toString().toStdString());
            updateFlags = CoordinateFrame::NoteUpdate;
            break;

        case GlobalCheckColumn: {
            bool isGlobal = value.toBool();
            int mode = isGlobal ? CoordinateFrame::Global : CoordinateFrame::Local;
            if(frameListItem->switchFrameMode(frame, mode)){
                updateFlags = CoordinateFrame::ModeUpdate | CoordinateFrame::PositionUpdate;
            }
            break;
        }
        case VisibleCheckColumn:
            frameListItem->setFrameMarkerVisible(frame, value.toBool());
            Q_EMIT dataChanged(index, index, {role});
            break;

        default:
            break;
        }
    }
    if(updateFlags){
        Q_EMIT dataChanged(index, index, {role});
        frame->notifyUpdate(updateFlags);
    }
    return false;
}


void FrameListModel::addFrame(int row, CoordinateFrame* frame, bool doInsert)
{
    if(frameList){
        int newFrameIndex = doInsert ? row : row + 1;
        frameList->insert(newFrameIndex, frame);
    }
}


void FrameListModel::removeFrames(QModelIndexList selected)
{
    if(frameList){
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int frameIndex = index.row() - numRemoved;
            if(frameIndex > 0 || !frameList->hasFirstElementAsDefaultFrame()){
                frameList->removeAt(frameIndex);
            }
            ++numRemoved;
        }
    }
}


void FrameListModel::onFrameAdded(int frameIndex)
{
    if(numFrames() == 0){
        // Remove the empty row first
        beginRemoveRows(QModelIndex(), 0, 0);
        endRemoveRows();
    }
    beginInsertRows(QModelIndex(), frameIndex, frameIndex);
    endInsertRows();
}


void FrameListModel::onFrameRemoved(int frameIndex)
{
    beginRemoveRows(QModelIndex(), frameIndex, frameIndex);
    endRemoveRows();
    if(numFrames() == 0){
        // This is necessary to show the empty row
        beginResetModel();
        endResetModel();
    }
}


void FrameListModel::onFrameUpdated(int frameIndex, int flags)
{
    if(flags & CoordinateFrame::IdUpdate){
        auto modelIndex = index(frameIndex, IdColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
    if(flags & CoordinateFrame::ModeUpdate){
        auto modelIndex = index(frameIndex, GlobalCheckColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
        flags |= CoordinateFrame::PositionUpdate;
    }
    if(flags & CoordinateFrame::NoteUpdate){
        auto modelIndex = index(frameIndex, NoteColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
    if(flags & CoordinateFrame::PositionUpdate){
        auto modelIndex = index(frameIndex, PositionColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
}


void FrameListModel::onFrameMarkerViisibilityChanged(int frameIndex)
{
    auto modelIndex = index(frameIndex, VisibleCheckColumn, QModelIndex());
    Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
}


CheckItemDelegate::CheckItemDelegate(CoordinateFrameListView::Impl* view)
    : QStyledItemDelegate(view),
      view(view)
{
    isValid = true;
}


void CheckItemDelegate::paint
(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    if(!isValid){
        return;
    }
    if(index.row() == 0 && index.column() == GlobalCheckColumn){
        if(view->frameList && view->frameList->hasFirstElementAsDefaultFrame()){
            return;
        }
    }

    QVariant value = index.data();
    bool isChecked = value.toBool();
    QStyle* style = view->style();
    QRect checkBoxRect = style->subElementRect(QStyle::SE_CheckBoxIndicator, &option);
    int w = checkBoxRect.width();
    int h = checkBoxRect.height();
    int centerX = option.rect.left() + qMax(option.rect.width() / 2 - w / 2, 0);
    int centerY = option.rect.top() + qMax(option.rect.height() / 2 - h / 2, 0);
    QStyleOptionViewItem modifiedOption(option);
    modifiedOption.rect.moveTo(centerX, centerY);
    modifiedOption.rect.setSize(QSize(w, h));
    if(isChecked){
        modifiedOption.state |= QStyle::State_On;
    }
    style->drawPrimitive(QStyle::PE_IndicatorItemViewItemCheck, &modifiedOption, painter);
}


bool CheckItemDelegate::editorEvent
(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& index)
{
    if(event->type() == QEvent::MouseButtonRelease){
        QVariant value = model->data(index, Qt::EditRole);
        auto isChecked = value.toBool();
        model->setData(index, !isChecked);
        event->accept();
    }
    return QStyledItemDelegate::editorEvent(event, model, option, index);
}


QWidget* CheckItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    return nullptr;
}


QSize CheckItemDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QStyle* style = view->style();
    QRect checkBoxRect = style->subElementRect(QStyle::SE_CheckBoxIndicator, &option);
    QSize size = checkBoxRect.size();
    size.setWidth(size.width() * 2);
    return size;
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
    hbox->addSpacing(hs);
    targetLabel.setStyleSheet("font-weight: bold");
    hbox->addWidget(&targetLabel, 0, Qt::AlignVCenter);
    hbox->addStretch();
    addButton.setText(_("Add"));
    addButton.sigClicked().connect([&](){ addFrameIntoCurrentIndex(false); });
    hbox->addWidget(&addButton);
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
    setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    globalCheckDelegate = new CheckItemDelegate(this);
    setItemDelegateForColumn(GlobalCheckColumn, globalCheckDelegate);
    setItemDelegateForColumn(VisibleCheckColumn, new CheckItemDelegate(this));
    setEditTriggers(
        /* QAbstractItemView::CurrentChanged | */
        QAbstractItemView::DoubleClicked |
        /* QAbstractItemView::SelectedClicked | */
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    frameListModel = new FrameListModel(this);
    setModel(frameListModel);

    auto hheader = horizontalHeader();
    hheader->setMinimumSectionSize(24);
    hheader->setSectionResizeMode(IdColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(NoteColumn, QHeaderView::Stretch);
    hheader->setSectionResizeMode(PositionColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(GlobalCheckColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(VisibleCheckColumn, QHeaderView::ResizeToContents);
    verticalHeader()->hide();

    connect(this, &QTableView::pressed,
            [this](const QModelIndex& index){
                if(QGuiApplication::mouseButtons() == Qt::LeftButton){
                    if(!isSelectionChangedAlreadyCalled){
                        startLocationEditing(index);
                    }
                }
            });
    
    vbox->addWidget(this);
    self->setLayout(vbox);

    isIndependentItemizationListSupported = false;
    targetItemPicker.setTargetPredicate(
        [&](CoordinateFrameListItem* item){
            if(!isIndependentItemizationListSupported){
                return item->itemizationMode() != CoordinateFrameListItem::IndependentItemization;
            }
            return true;
        });
    
    targetItemPicker.sigTargetItemChanged().connect(
        [&](CoordinateFrameListItem* item){
            setCoordinateFrameListItem(item);
        });
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
    impl->stopLocationEditing();
}


void CoordinateFrameListView::onAttachedMenuRequest(MenuManager& menuManager)
{
    auto supportCheck = menuManager.addCheckItem(_("Support offset frames"));
    supportCheck->setChecked(impl->isIndependentItemizationListSupported);
    supportCheck->sigToggled().connect(
        [&](bool on){
            impl->isIndependentItemizationListSupported = on;
            impl->targetItemPicker.refresh();
        });
}


void CoordinateFrameListView::Impl::setCoordinateFrameListItem(CoordinateFrameListItem* item)
{
    stopLocationEditing();

    targetItem = item;
    transientMarkerHolder.reset();

    if(item){
        string caption;
        if(auto parentLocation = item->getFrameParentLocationProxy()){
            targetLabel.setText(
                format("{0} - {1}",  parentLocation->getName(), item->displayName()).c_str());
        } else {
            targetLabel.setText(item->displayName().c_str());
        }
        frameList = item->frameList();
        frameListModel->setFrameListItem(item);
        globalCheckDelegate->isValid = frameList->isForBaseFrames();
    } else {
        targetLabel.setText("---");
        frameList = nullptr;
        frameListModel->setFrameListItem(nullptr);
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
    if(frameList){
        auto id = frameList->createNextId();
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
    isSelectionChangedAlreadyCalled = false;
    
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

    if(row > 0 || (frameList && !frameList->hasFirstElementAsDefaultFrame())){
        contextMenuManager.addItem(_("Remove"))
            ->sigTriggered().connect([=](){ removeSelectedFrames(); });
    }
    
    contextMenuManager.popupMenu()->popup(globalPos);
}


void CoordinateFrameListView::Impl::selectionChanged
(const QItemSelection& selected, const QItemSelection& deselected)
{
    isSelectionChangedAlreadyCalled = true;
    
    QTableView::selectionChanged(selected, deselected);

    auto indexes = selected.indexes();
    if(indexes.empty()){
        stopLocationEditing();
        transientMarkerHolder.reset();
    } else {
        auto modelIndex = indexes.front();
        startLocationEditing(modelIndex);
        auto frame = frameListModel->frameAt(modelIndex);
        transientMarkerHolder = targetItem->transientFrameMarkerHolder(frame);
    }
}


void CoordinateFrameListView::Impl::startLocationEditing(const QModelIndex& modelIndex)
{
    stopLocationEditing();
    int frameIndex = modelIndex.row();
    if(auto frameItem = targetItem->findFrameItemAt(frameIndex)){
        locationProxy = frameItem->getLocationProxy();
        locationProxy->requestEdit();
    }
}


void CoordinateFrameListView::Impl::stopLocationEditing()
{
    if(locationProxy){
        locationProxy->expire();
        locationProxy.reset();
    }
}


bool CoordinateFrameListView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "current_item");
    return true;
}


bool CoordinateFrameListView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "current_item");
    return true;
}
