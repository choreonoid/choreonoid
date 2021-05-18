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
#include <cnoid/EigenUtil>
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

constexpr int NumColumns = 4;
constexpr int IdColumn = 0;
constexpr int NoteColumn = 1;
constexpr int JointSpaceCheckColumn = 2;
constexpr int PositionColumn = 3;

class PositionListModel : public QAbstractTableModel
{
public:
    MprPositionListView::Impl* view;
    MprProgramItemBasePtr programItem;
    MprPositionListPtr positionList;
    ScopedConnectionSet positionListConnections;
    QFont monoFont;
    
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
    QVariant getPositionData(MprPosition* position) const;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void changePositionType(int positionIndex, MprPosition* position, bool isJointSpace);
    void addPosition(int row, MprPosition* position, bool doInsert);
    void removePositions(QModelIndexList selected);
    void onPositionAdded(int positionIndex);
    void onPositionRemoved(int positionIndex);
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
    MprProgramItemBasePtr programItem;
    MprPositionListPtr positionList;
    PositionListModel* positionListModel;
    CheckItemDelegate* globalCheckDelegate;
    ReferencedPtr transientMarkerHolder;
    QLabel targetLabel;
    PushButton addButton;
    PushButton touchupButton;
    MenuManager contextMenuManager;
    BodySyncMode bodySyncMode;
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
    void setBodySyncMode(BodySyncMode mode);
    bool applyPosition(int positionIndex, bool forceDirectSync);
    void touchupCurrentPosition();
};

}


namespace {

PositionListModel::PositionListModel(MprPositionListView::Impl* view)
    : QAbstractTableModel(view),
      view(view),
      monoFont("Monospace")
{
    monoFont.setStyleHint(QFont::TypeWriter);
}


void PositionListModel::setProgramItem(MprProgramItemBase* programItem)
{
    beginResetModel();

    this->programItem = programItem;
    if(programItem){
        this->positionList = programItem->program()->positionList();
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
                [&](int index, MprPosition*){ onPositionRemoved(index); }));
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
    if(index.isValid() && index.column() != PositionColumn){
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

        case PositionColumn:
            return getPositionData(position);

        default:
            break;
        }
    } else if(role == Qt::TextAlignmentRole){
        if(column == NoteColumn || column == PositionColumn){
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


QVariant PositionListModel::getPositionData(MprPosition* position) const
{
    if(position->isIK()){
        auto ik = position->ikPosition();
        auto p = ik->position().translation();
        auto rpy = degree(ik->rpy());
        auto& baseId = ik->baseFrameId();
        auto& offsetId = ik->offsetFrameId();
        if(baseId.isInt() && offsetId.isInt()){
            return format("{0: 1.3f} {1: 1.3f} {2: 1.3f} {3: 6.1f} {4: 6.1f} {5: 6.1f} : {6:2X} {7:2d} {8:2d}",
                          p.x(), p.y(), p.z(), rpy[0], rpy[1], rpy[2], ik->configuration(),
                          baseId.toInt(), offsetId.toInt()).c_str();
        } else {
            return format("{0: 1.3f} {1: 1.3f} {2: 1.3f} {3: 6.1f} {4: 6.1f} {5: 6.1f} : {6:2X}",
                          p.x(), p.y(), p.z(), rpy[0], rpy[1], rpy[2], ik->configuration()).c_str();
        }
    } else if(position->isFK()){
        auto fk = position->fkPosition();
        string data;
        int n = fk->numJoints();
        int m = n - 1;
        for(int i=0; i < n; ++i){
            data += format("{0: 6.1f}", degree(fk->q(i)));
            if(i < m){
                data += " ";
            }
        }
        return data.c_str();
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
            break;
            
        default:
            break;
        }
    }
    if(updateFlags){
        //Q_EMIT dataChanged(index, index, {role});
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

    if(view->applyPosition(positionIndex, true)){
        newPosition->fetch(programItem->kinematicsKit());
        positionList->replace(positionIndex, newPosition);
    }
}


void PositionListModel::addPosition(int row, MprPosition* position, bool doInsert)
{
    if(positionList){
        int newPositionIndex = doInsert ? row : row + 1;
        positionList->insert(newPositionIndex, position);
    }
}


void PositionListModel::removePositions(QModelIndexList selected)
{
    if(positionList){
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int positionIndex = index.row() - numRemoved;
            positionList->removeAt(positionIndex);
            ++numRemoved;
        }
    }
}


void PositionListModel::onPositionAdded(int positionIndex)
{
    if(numPositions() == 0){
        // Remove the empty row first
        beginRemoveRows(QModelIndex(), 0, 0);
        endRemoveRows();
    }
    beginInsertRows(QModelIndex(), positionIndex, positionIndex);
    endInsertRows();
}


void PositionListModel::onPositionRemoved(int positionIndex)
{
    beginRemoveRows(QModelIndex(), positionIndex, positionIndex);
    endRemoveRows();
    if(numPositions() == 0){
        // This is necessary to show the empty row
        beginResetModel();
        endResetModel();
    }
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
    if(flags & MprPosition::PositionUpdate){
        auto modelIndex = index(positionIndex, PositionColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
    if(flags & MprPosition::ObjectReplaced){
        auto modelIndex1 = index(positionIndex, IdColumn, QModelIndex());
        auto modelIndex2 = index(positionIndex, PositionColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex1, modelIndex2, { Qt::EditRole });
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

}


void MprPositionListView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<MprPositionListView>(
        "MprPositionListView", N_("Waypoints"), ViewManager::SINGLE_OPTIONAL);
}


MprPositionListView::MprPositionListView()
{
    impl = new Impl(this);
}


MprPositionListView::Impl::Impl(MprPositionListView* self)
    : QTableView(self),
      self(self),
      targetItemPicker(self)
{
    self->setDefaultLayoutArea(View::RIGHT);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    auto sty = self->style();
    int hs = sty->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    auto hbox = new QHBoxLayout;
    hbox->addSpacing(hs);
    targetLabel.setStyleSheet("font-weight: bold");
    hbox->addWidget(&targetLabel, 0, Qt::AlignVCenter);
    hbox->addStretch();
    addButton.setText(_("Add"));
    addButton.sigClicked().connect([&](){ addPositionIntoCurrentIndex(false); });
    hbox->addWidget(&addButton);
    touchupButton.setText(_("Touch-up"));
    touchupButton.sigClicked().connect([&](){ touchupCurrentPosition(); });
    hbox->addWidget(&touchupButton);
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
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setItemDelegateForColumn(JointSpaceCheckColumn, new CheckItemDelegate(this));
    setEditTriggers(
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    positionListModel = new PositionListModel(this);
    setModel(positionListModel);

    auto hheader = horizontalHeader();
    hheader->setMinimumSectionSize(24);
    hheader->setSectionResizeMode(IdColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(NoteColumn, QHeaderView::Stretch);
    hheader->setSectionResizeMode(PositionColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(JointSpaceCheckColumn, QHeaderView::ResizeToContents);
    verticalHeader()->hide();

    connect(this, &QTableView::pressed,
            [this](const QModelIndex& index){
                if(index.isValid() && QGuiApplication::mouseButtons() == Qt::LeftButton){
                    if(!isSelectionChangedAlreadyCalled){
                        applyPosition(index.row(), false);
                    }
                }
            });

    connect(this, &QTableView::doubleClicked,
            [this](const QModelIndex& index){
                if(index.isValid()){
                    applyPosition(index.row(), true);
                }
            });
    
    vbox->addWidget(this);
    self->setLayout(vbox);

    targetItemPicker.sigTargetItemChanged().connect(
        [&](MprProgramItemBase* item){ setProgramItem(item); });

    bodySyncMode = DirectBodySync;
}


MprPositionListView::~MprPositionListView()
{
    delete impl;
}


void MprPositionListView::onAttachedMenuRequest(MenuManager& menuManager)
{
    auto twoStageCheck = menuManager.addCheckItem(_("Two-stage sync"));
    twoStageCheck->setChecked(impl->bodySyncMode == TwoStageBodySync);
    twoStageCheck->sigToggled().connect(
        [&](bool on){ impl->setBodySyncMode(on ? TwoStageBodySync : DirectBodySync); });

}


void MprPositionListView::Impl::setProgramItem(MprProgramItemBase* item)
{
    programItem = item;

    if(item){
        string caption;
        if(auto bodyItem = item->targetBodyItem()){
            targetLabel.setText(
                format("{0} - {1}",  bodyItem->displayName(), item->displayName()).c_str());
        } else {
            targetLabel.setText(item->displayName().c_str());
        }
        positionList = item->program()->positionList();
        positionListModel->setProgramItem(item);
    } else {
        targetLabel.setText("---");
        positionList = nullptr;
        positionListModel->setProgramItem(nullptr);
    }
    addButton.setEnabled(programItem != nullptr);
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
        position->fetch(programItem->kinematicsKit());
        positionListModel->addPosition(row, position, doInsert);
        resizeColumnToContents(IdColumn);
        resizeColumnToContents(PositionColumn);
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
        applyPosition(indexes.front().row(), false);
    }
}


void MprPositionListView::setBodySyncMode(BodySyncMode mode)
{
    impl->setBodySyncMode(mode);
}


void MprPositionListView::Impl::setBodySyncMode(BodySyncMode mode)
{
    if(mode != bodySyncMode){
        bodySyncMode = mode;
        if(!mode && programItem){
            programItem->clearSuperimposition();
        }
    }
}


bool MprPositionListView::Impl::applyPosition(int positionIndex, bool forceDirectSync)
{
    bool result = false;
    auto position = positionList->positionAt(positionIndex);
    if(bodySyncMode == DirectBodySync || forceDirectSync){
        result = programItem->moveTo(position);
    } else {
        result = programItem->superimposePosition(position);
    }
    return result;
}


void MprPositionListView::Impl::touchupCurrentPosition()
{
    if(positionList){
        if(auto position = positionListModel->positionAt(selectionModel()->currentIndex())){
            programItem->touchupPosition(position);
        }
    }
}


bool MprPositionListView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "current_item");
    auto mode = impl->bodySyncMode;
    if(mode == DirectBodySync){
        archive.write("body_sync_mode", "direct");
    } else if(mode == TwoStageBodySync){
        archive.write("body_sync_mode", "two-stage");
    }
    return true;
}


bool MprPositionListView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "current_item");
    string mode;
    if(archive.read("body_sync_mode", mode)){
        if(mode == "direct"){
            impl->setBodySyncMode(DirectBodySync);
        } else if(mode == "two-stage"){
            impl->setBodySyncMode(TwoStageBodySync);
        }
    }
    return true;
}
