#include "MprPositionListView.h"
#include "MprPositionList.h"
#include "MprPosition.h"
#include "MprProgramItemBase.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/Buttons>
#include <cnoid/ConnectionSet>
#include <cnoid/BodyItem>
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
constexpr int JointSpaceCheckColumn = 2;

class PositionListModel : public QAbstractTableModel
{
public:
    MprPositionListView::Impl* view;
    MprProgramItemBasePtr programItem;
    MprPositionListPtr positionList;
    ScopedConnectionSet positionListConnections;
    
    PositionListModel(MprPositionListView::Impl* view);
    void setProgramItem(MprProgramItemBase* programItem);
    bool isValid() const;
    int numPositions() const;
    MprPosition* positionAt(const QModelIndex& index) const;
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void changePositionType(int positionIndex, MprPosition* position, bool isJointSpace);
    void addPosition(int row, MprPosition* position, bool doInsert);
    void removePositions(QModelIndexList selected);
    void onPositionAdded(int positionIndex);
    void onPositionRemoved(int positionIndex, MprPosition* position);
    void onPositionUpdated(int positionIndex, int flags);
};

class CheckItemDelegate : public QStyledItemDelegate
{
public:
    MprPositionListView::Impl* view;
    bool isValid;
    
    CheckItemDelegate(MprPositionListView::Impl* view);
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

class MprPositionListView::Impl : public QTableView
{
public:
    MprPositionListView* self;
    TargetItemPicker<MprProgramItemBase> targetItemPicker;
    MprProgramItemBasePtr targetItem;
    MprPositionListPtr positionList;
    PositionListModel* positionListModel;
    CheckItemDelegate* globalCheckDelegate;
    ReferencedPtr transientMarkerHolder;
    QLabel targetLabel;
    PushButton addButton;
    MenuManager contextMenuManager;
    bool isSelectionChangedAlreadyCalled;

    Impl(MprPositionListView* self);
    void setProgramItem(MprProgramItemBase* item);
    void addPositionIntoCurrentIndex(bool doInsert);
    void addPosition(int row, bool doInsert);
    void removeSelectedPositions();
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(int row, QPoint globalPos);
    virtual void selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) override;
    void applyPosition(const QModelIndex& modelIndex);
};

}


PositionListModel::PositionListModel(MprPositionListView::Impl* view)
    : QAbstractTableModel(view),
      view(view)
{

}


void PositionListModel::setProgramItem(MprProgramItemBase* programItem)
{
    beginResetModel();

    this->programItem = programItem;
    if(programItem){
        this->positionList = programItem->positionList();
    } else {
        this->positionList = nullptr;
    }

    positionListConnections.disconnect();
    if(positionList){
        positionListConnections.add(
            positionList->sigPositionAdded().connect(
                [&](int index){ onPositionAdded(index); }));
        positionListConnections.add(
            positionList->sigPositionRemoved().connect(
                [&](int index, MprPosition* position){ onPositionRemoved(index, position); }));
        positionListConnections.add(
            positionList->sigPositionUpdated().connect(
                [&](int index, int flags){ onPositionUpdated(index, flags); }));
    }
            
    endResetModel();
}


bool PositionListModel::isValid() const
{
    return positionList != nullptr;
}


int PositionListModel::numPositions() const
{
    if(positionList){
        return positionList->numPositions();
    }
    return 0;
}


MprPosition* PositionListModel::positionAt(const QModelIndex& index) const
{
    if(!index.isValid()){
        return nullptr;
    }
    return positionList->positionAt(index.row());
}
        
    
int PositionListModel::rowCount(const QModelIndex& parent) const
{
    int n = 0;
    if(!parent.isValid()){
        n = numPositions();
    }
    if(n == 0){ // to show an empty row
        n = 1;
    }
    return n;
}


int PositionListModel::columnCount(const QModelIndex& parent) const
{
    return NumColumns;
}
        

QVariant PositionListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case IdColumn:
                return " ID ";
            case NoteColumn:
                return _("Note");
            case JointSpaceCheckColumn:
                return _("J");
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


QModelIndex PositionListModel::index(int row, int column, const QModelIndex& parent) const
{
    if(!positionList || parent.isValid()){
        return QModelIndex();
    }
    if(row < numPositions()){
        return createIndex(row, column);
    }
    return QModelIndex();
}
    

Qt::ItemFlags PositionListModel::flags(const QModelIndex& index) const
{
    auto flags = QAbstractTableModel::flags(index);
    if(index.isValid()){
        flags |= Qt::ItemIsEditable;
    }
    return flags;
}


QVariant PositionListModel::data(const QModelIndex& index, int role) const
{
    auto position = positionAt(index);
    if(!position){
        return QVariant();
    }
    int column = index.column();
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(column){
        case IdColumn:
            return position->id().label().c_str();

        case NoteColumn:
            return position->note().c_str();

        case JointSpaceCheckColumn:
            return position->isFK();

        default:
            break;
        }
    } else if(role == Qt::TextAlignmentRole){
        if(column == NoteColumn){
            return (Qt::AlignLeft + Qt::AlignVCenter);
        } else {
            return Qt::AlignCenter;
        }
    }
    return QVariant();
}


bool PositionListModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if(!index.isValid()){
        return false;
    }

    auto positionIndex = index.row();
    int updateFlags = 0;
    auto position = positionList->positionAt(positionIndex);

    if(role == Qt::EditRole){
        switch(index.column()){
        case IdColumn: {
            bool isInt;
            auto stringId = value.toString();
            int intId = stringId.toInt(&isInt);
            if(isInt){
                positionList->resetId(position, intId);
            } else {
                positionList->resetId(position, stringId.toStdString());
            }
            updateFlags = MprPosition::IdUpdate;
            break;
        }
        case NoteColumn:
            position->setNote(value.toString().toStdString());
            updateFlags = MprPosition::NoteUpdate;
            break;

        case JointSpaceCheckColumn:
            changePositionType(positionIndex, position, value.toBool());
            Q_EMIT dataChanged(index, index, {role});
            break;
            
        default:
            break;
        }
    }
    if(updateFlags){
        Q_EMIT dataChanged(index, index, {role});
        position->notifyUpdate(updateFlags);
    }
    return false;
}


void PositionListModel::changePositionType(int positionIndex, MprPosition* position, bool isJointSpace)
{
    MprPosition* newPosition = nullptr;
    if(position->isIK() && isJointSpace){
        newPosition = new MprFkPosition;
    } else if(position->isFK() && !isJointSpace){
        newPosition = new MprIkPosition;
    }
    newPosition->setId(position->id());
    newPosition->setNote(position->note());
    newPosition->setCurrentPosition(programItem->kinematicsKit());

    positionListConnections.block();
    positionList->removeAt(positionIndex);
    positionList->insert(positionIndex, newPosition);
    positionListConnections.unblock();
}


void PositionListModel::addPosition(int row, MprPosition* position, bool doInsert)
{
    if(positionList){
        int newPositionIndex = doInsert ? row : row + 1;
        if(positionList->insert(newPositionIndex, position)){
            if(numPositions() == 0){
                // Remove the empty row first
                beginRemoveRows(QModelIndex(), 0, 0);
                endRemoveRows();
            }
            beginInsertRows(QModelIndex(), newPositionIndex, newPositionIndex);
            endInsertRows();
        }
    }
}


void PositionListModel::removePositions(QModelIndexList selected)
{
    if(positionList){
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int row = index.row() - numRemoved;
            beginRemoveRows(QModelIndex(), row, row);
            positionList->removeAt(row);
            ++numRemoved;
            endRemoveRows();
        }
        if(positionList->numPositions() == 0){
            // This is necessary to show the empty row
            beginResetModel();
            endResetModel();
        }
    }
}


void PositionListModel::onPositionAdded(int positionIndex)
{

}


void PositionListModel::onPositionRemoved(int positionIndex, MprPosition* position)
{

}


void PositionListModel::onPositionUpdated(int positionIndex, int flags)
{
    if(flags & MprPosition::IdUpdate){
        auto modelIndex = index(positionIndex, IdColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
    if(flags & MprPosition::NoteUpdate){
        auto modelIndex = index(positionIndex, NoteColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
}


CheckItemDelegate::CheckItemDelegate(MprPositionListView::Impl* view)
    : QStyledItemDelegate(view),
      view(view)
{

}


void CheckItemDelegate::paint
(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
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


void MprPositionListView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<MprPositionListView>(
        "MprPositionListView", N_("Program Waypoints"), ViewManager::SINGLE_OPTIONAL);
}


MprPositionListView::MprPositionListView()
{
    impl = new Impl(this);
}


MprPositionListView::Impl::Impl(MprPositionListView* self)
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
    addButton.sigClicked().connect([&](){ addPositionIntoCurrentIndex(false); });
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
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    setItemDelegateForColumn(JointSpaceCheckColumn, new CheckItemDelegate(this));
    setEditTriggers(
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    positionListModel = new PositionListModel(this);
    setModel(positionListModel);

    auto hheader = horizontalHeader();
    hheader->setMinimumSectionSize(1);
    hheader->setSectionResizeMode(IdColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(NoteColumn, QHeaderView::Stretch);
    hheader->setSectionResizeMode(JointSpaceCheckColumn, QHeaderView::ResizeToContents);
    auto vheader = verticalHeader();
    vheader->setSectionResizeMode(QHeaderView::ResizeToContents);
    vheader->hide();

    connect(this, &QTableView::pressed,
            [this](const QModelIndex& index){
                if(!isSelectionChangedAlreadyCalled){
                    applyPosition(index);
                }
            });
    
    vbox->addWidget(this);
    self->setLayout(vbox);

    targetItemPicker.sigTargetItemChanged().connect(
        [&](MprProgramItemBase* item){ setProgramItem(item); });
}


MprPositionListView::~MprPositionListView()
{
    delete impl;
}


void MprPositionListView::Impl::setProgramItem(MprProgramItemBase* item)
{
    targetItem = item;

    if(item){
        string caption;
        if(auto bodyItem = item->targetBodyItem()){
            targetLabel.setText(
                format("{0} - {1}",  bodyItem->displayName(), item->displayName()).c_str());
        } else {
            targetLabel.setText(item->displayName().c_str());
        }
        positionList = item->positionList();
        positionListModel->setProgramItem(item);
    } else {
        targetLabel.setText("---");
        positionList = nullptr;
        positionListModel->setProgramItem(nullptr);
    }
    addButton.setEnabled(targetItem != nullptr);
}


void MprPositionListView::Impl::addPositionIntoCurrentIndex(bool doInsert)
{
    auto current = selectionModel()->currentIndex();
    int row = current.isValid() ? current.row() : positionListModel->numPositions();
    addPosition(row, doInsert);
}


void MprPositionListView::Impl::addPosition(int row, bool doInsert)
{
    if(positionList){
        auto id = positionList->createNextId();
        MprPositionPtr position = new MprIkPosition(id);
        positionListModel->addPosition(row, position, doInsert);
        resizeColumnToContents(IdColumn);
    }
}


void MprPositionListView::Impl::removeSelectedPositions()
{
    positionListModel->removePositions(selectionModel()->selectedRows());
}


void MprPositionListView::Impl::keyPressEvent(QKeyEvent* event)
{
    bool processed = true;

    switch(event->key()){
    case Qt::Key_Escape:
        clearSelection();
        break;
    case Qt::Key_Insert:
        addPositionIntoCurrentIndex(true);
        break;
    case Qt::Key_Delete:
        removeSelectedPositions();
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

       
void MprPositionListView::Impl::mousePressEvent(QMouseEvent* event)
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


void MprPositionListView::Impl::showContextMenu(int row, QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);

    contextMenuManager.addItem(_("Add"))
        ->sigTriggered().connect([=](){ addPosition(row, false); });

    contextMenuManager.addItem(_("Remove"))
        ->sigTriggered().connect([=](){ removeSelectedPositions(); });
    
    contextMenuManager.popupMenu()->popup(globalPos);
}


void MprPositionListView::Impl::selectionChanged
(const QItemSelection& selected, const QItemSelection& deselected)
{
    isSelectionChangedAlreadyCalled = true;
    
    QTableView::selectionChanged(selected, deselected);

    auto indexes = selected.indexes();
    if(!indexes.empty()){
        applyPosition(indexes.front());
    }
}


void MprPositionListView::Impl::applyPosition(const QModelIndex& modelIndex)
{


}


bool MprPositionListView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "current_item");
    return true;
}


bool MprPositionListView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "current_item");
    return true;
}
