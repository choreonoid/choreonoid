#include "MprPositionListWidget.h"
#include "MprPosition.h"
#include "MprPositionList.h"
#include "MprBodyItemUtil.h"
#include <cnoid/KinematicBodyItemSet>
#include <cnoid/MenuManager>
#include <cnoid/DisplayValueFormat>
#include <cnoid/ConnectionSet>
#include <cnoid/MessageOut>
#include <cnoid/EigenUtil>
#include <cnoid/Format>
#include <cnoid/QtEventUtil>
#include <QHeaderView>
#include <QAbstractTableModel>
#include <QStyledItemDelegate>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QGuiApplication>
#include <QFontMetrics>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

enum ColumnID {
    IdColumn = MprPositionListWidget::IdColumn,
    NoteColumn = MprPositionListWidget::NoteColumn,
    JointSpaceCheckColumn = MprPositionListWidget::JointSpaceCheckColumn,
    MainPositionColumn = MprPositionListWidget::MainPositionColumn,
    NumMinimumColumns = MprPositionListWidget::NumMinimumColumns
};

class PositionListModel : public QAbstractTableModel
{
public:
    MprPositionListWidget* widget;
    KinematicBodyItemSetPtr bodyItemSet;
    ScopedConnectionSet bodyItemSetConnections;
    MprPositionListPtr positionList;
    ScopedConnectionSet positionListConnections;
    int columnCount_;
    vector<int> bodyPartIndices;
    int mainBodyPartIndex;
    QString singlePositionHeaderLabel;
    QStringList positionHeaderLabels;
    QFont monoFont;
    DisplayValueFormat* valueFormat;
    
    PositionListModel(MprPositionListWidget::Impl* widgetImpl);
    void setBodyItemSet(KinematicBodyItemSet* bodyItemSet);    
    void setPositionList(MprPositionList* positionList);
    bool isValid() const;
    int numPositions() const;
    MprPosition* positionAt(const QModelIndex& index) const;
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    bool checkIfJointSpacePosition(MprPosition* position) const;
    QVariant getPositionData(MprPosition* position, int posColumnIndex) const;
    QVariant getSinglePositionData(MprPosition* position) const;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void changePositionType(int positionIndex, MprPosition* position, bool isJointSpace);
    MprPositionPtr getPositionWithChangedPositionType(
        MprPosition* position, BodyKinematicsKit* kinematicsKit, bool isJointSpace);
    void addPosition(int row, MprPosition* position, bool doInsert);
    void removePositions(QModelIndexList selected);
    void onPositionAdded(int positionIndex);
    void onPositionRemoved(int positionIndex);
    void onPositionUpdated(int positionIndex, int flags);
};

class CheckItemDelegate : public QStyledItemDelegate
{
public:
    MprPositionListWidget* widget;
    bool isValid;
    
    CheckItemDelegate(MprPositionListWidget* widget);
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

class MprPositionListWidget::Impl
{
public:
    MprPositionListWidget* self;
    DisplayValueFormat* valueFormat;
    KinematicBodyItemSetPtr bodyItemSet;
    MprPositionListPtr positionList;
    PositionListModel* positionListModel;
    CheckItemDelegate* globalCheckDelegate;
    BodySyncMode bodySyncMode;
    Signal<void(int index, const QPoint& globalPos)> sigContextMenuRequest;
    Signal<void(int index, bool isSelectionChanged)> sigPositionPressed;
    Signal<void(int index)> sigPositionDoubleClicked;
    Signal<void(const QItemSelection& selected, const QItemSelection& deselected)> sigSelectionChanged;
    MenuManager contextMenuManager;
    ConnectionSet operationConnections;
    bool isStandardUserOperationEnabled;
    bool isSelectionChangedAlreadyCalled;

    Impl(MprPositionListWidget* self);
    void setStandardUserOperationEnabled(bool on);
    void addPosition(int row, bool doInsert);    
};

}


PositionListModel::PositionListModel(MprPositionListWidget::Impl* widgetImpl)
    : QAbstractTableModel(widgetImpl->self),
      widget(widgetImpl->self),
      monoFont("Monospace")
{
    columnCount_ = NumMinimumColumns;
    mainBodyPartIndex = -1;
    singlePositionHeaderLabel = _("Position");
    monoFont.setStyleHint(QFont::TypeWriter);
    valueFormat = widgetImpl->valueFormat;
}


void PositionListModel::setBodyItemSet(KinematicBodyItemSet* bodyItemSet)
{
    beginResetModel();

    this->bodyItemSet = bodyItemSet;
    bodyItemSetConnections.disconnect();
    columnCount_ = NumMinimumColumns;
    positionHeaderLabels.clear();

    if(!bodyItemSet){
        bodyPartIndices.clear();
        mainBodyPartIndex = -1;
    } else {
        bodyPartIndices = bodyItemSet->validBodyPartIndices();
        mainBodyPartIndex = bodyItemSet->mainBodyPartIndex();
        if(bodyPartIndices.size() >= 2){
            for(auto index : bodyPartIndices){
                positionHeaderLabels.append(bodyItemSet->bodyPart(index)->body()->name().c_str());
            }
            columnCount_ += positionHeaderLabels.size() - 1;
        }
        bodyItemSetConnections.add(
            bodyItemSet->sigBodySetChanged().connect(
                [this](){
                    if(this->bodyItemSet){
                        setBodyItemSet(this->bodyItemSet);
                    }
                }));
    }

    if(positionHeaderLabels.empty()){
        positionHeaderLabels.append(singlePositionHeaderLabel);
    }

    endResetModel();
}


void PositionListModel::setPositionList(MprPositionList* positionList)
{
    this->positionList = positionList;
    positionListConnections.disconnect();

    if(positionList){
        positionListConnections.add(
            positionList->sigPositionAdded().connect(
                [this](int index){ onPositionAdded(index); }));
        positionListConnections.add(
            positionList->sigPositionRemoved().connect(
                [this](int index, MprPosition*){ onPositionRemoved(index); }));
        positionListConnections.add(
            positionList->sigPositionUpdated().connect(
                [this](int index, int flags){ onPositionUpdated(index, flags); }));
    }
}


bool PositionListModel::isValid() const
{
    return bodyItemSet != nullptr && positionList != nullptr;
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
    return parent.isValid() ? 0 : numPositions();
}


int PositionListModel::columnCount(const QModelIndex& parent) const
{
    return columnCount_;
}
        

QVariant PositionListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case IdColumn:
                return QString(" ID ");
            case NoteColumn:
                return QString(_("Note"));
            case JointSpaceCheckColumn:
                return QString(_("J"));
            default:
                {
                    int posColumnIndex = section - MainPositionColumn;
                    if(posColumnIndex < static_cast<int>(positionHeaderLabels.size())){
                        return positionHeaderLabels[posColumnIndex];
                    }
                    return QVariant();
                }
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
    if(index.isValid() && (index.column() < MainPositionColumn)){
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
            return checkIfJointSpacePosition(position);

        default:
            return getPositionData(position, column - MainPositionColumn);
        }
    } else if(role == Qt::TextAlignmentRole){
        if(column == NoteColumn ||
           column >= MainPositionColumn){
            return static_cast<Qt::Alignment::Int>(Qt::AlignLeft | Qt::AlignVCenter);
        } else {
            return Qt::AlignCenter;
        }
    } else if(role == Qt::FontRole){
        if(column >= MainPositionColumn){
            return monoFont;
        }
    }
    return QVariant();
}


bool PositionListModel::checkIfJointSpacePosition(MprPosition* position) const
{
    if(position->isFK()){
        return true;
    }
    if(auto composite = position->compositePosition()){
        if(auto mainPosition = composite->position(mainBodyPartIndex)){
            return mainPosition->isFK();
        }
    }
    return false;
}


QVariant PositionListModel::getPositionData(MprPosition* position, int posColumnIndex) const
{
    if(posColumnIndex < static_cast<int>(bodyPartIndices.size())){
        int positionIndex = bodyPartIndices[posColumnIndex];
        if(positionIndex >= 0){
            if(!position->isComposite()){
                if(positionIndex == mainBodyPartIndex){
                    return getSinglePositionData(position);
                }
            } else {
                if(auto pi = position->compositePosition()->position(positionIndex)){
                    return getSinglePositionData(pi);
                }
            }
        }
    }
    return QVariant();
}


QVariant PositionListModel::getSinglePositionData(MprPosition* position) const
{
    if(position->isIK()){
        auto ik = position->ikPosition();
        auto p = ik->position().translation();
        auto rpy = degree(ik->rpy());
        auto& baseId = ik->baseFrameId();
        auto& offsetId = ik->offsetFrameId();
        if(baseId.isInt() && offsetId.isInt()){
            if(valueFormat->isMillimeter()){
                return formatC("{0: 8.1f} {1: 8.1f} {2: 8.1f} "
                               "{3: 6.1f} {4: 6.1f} {5: 6.1f} "
                               ": {6:2X} {7:2d} {8:2d}",
                               p.x() * 1000.0, p.y() * 1000.0, p.z() * 1000.0,
                               rpy[0], rpy[1], rpy[2],
                               ik->configuration(), baseId.toInt(), offsetId.toInt()).c_str();
            } else {
                return formatC("{0: 7.3f} {1: 7.3f} {2: 7.3f} "
                               "{3: 6.1f} {4: 6.1f} {5: 6.1f} "
                               ": {6:2X} {7:2d} {8:2d}",
                               p.x(), p.y(), p.z(),
                               rpy[0], rpy[1], rpy[2],
                               ik->configuration(), baseId.toInt(), offsetId.toInt()).c_str();
            }
        } else {
            if(valueFormat->isMillimeter()){
                return formatC("{0: 8.1f} {1: 8.1f} {2: 8.1f} "
                               "{3: 6.1f} {4: 6.1f} {5: 6.1f} : {6:2X}",
                               p.x() * 1000.0, p.y() * 1000.0, p.z() * 1000.0,
                               rpy[0], rpy[1], rpy[2],
                               ik->configuration()).c_str();
            } else {
                return formatC("{0: 7.3f} {1: 7.3f} {2: 7.3f} "
                               "{3: 6.1f} {4: 6.1f} {5: 6.1f} : {6:2X}",
                               p.x(), p.y(), p.z(),
                               rpy[0], rpy[1], rpy[2], ik->configuration()).c_str();
            }
        }
    } else if(position->isFK()){
        auto fk = position->fkPosition();
        string data;
        int n = fk->numJoints();
        int m = n - 1;
        for(int i=0; i < n; ++i){
            auto q = fk->q(i);
            if(fk->checkIfRevoluteJoint(i)){
                data += formatC("{0: 6.1f}", degree(q));
            } else {
                if(valueFormat->isMillimeter()){
                    data += formatC("{0: 9.3f}", q * 1000.0);
                } else {
                    data += formatC("{0: 6.3f}", q);
                }
            }
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
    if(!bodyItemSet){
        return;
    }
    if(!applyPosition(bodyItemSet, position, true, MessageOut::interactive())){
        return;
    }
    if(auto composite = position->compositePosition()){
        bool updated = false;
        for(auto& index : bodyPartIndices){
            auto subPosition = composite->position(index);
            auto kinematicsKit = bodyItemSet->bodyPart(index);
            auto newPosition = getPositionWithChangedPositionType(subPosition, kinematicsKit, isJointSpace);
            if(newPosition){
                composite->setPosition(index, newPosition);
                updated = true;
            }
        }
        if(updated){
            composite->notifyUpdate(MprPosition::PositionUpdate);
        }
    } else {
        auto kinematicsKit = bodyItemSet->mainBodyPart();
        auto newPosition = getPositionWithChangedPositionType(position, kinematicsKit, isJointSpace);
        if(newPosition){
            newPosition->setId(position->id());
            newPosition->setNote(position->note());
            positionList->replace(positionIndex, newPosition);
        }
    } 
}


MprPositionPtr PositionListModel::getPositionWithChangedPositionType
(MprPosition* position, BodyKinematicsKit* kinematicsKit, bool isJointSpace)
{
    MprPositionPtr newPosition;

    if(kinematicsKit->hasJointPath()){ // The body part must be able to have a FK position

        if(position->isIK() && isJointSpace){
            newPosition = new MprFkPosition;
        } else if(position->isFK() && !isJointSpace){
            newPosition = new MprIkPosition;
        }
        if(newPosition){
            if(!newPosition->fetch(kinematicsKit, MessageOut::interactive())){
                newPosition.reset();
            }
        }
    }
    
    return newPosition;
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
    beginInsertRows(QModelIndex(), positionIndex, positionIndex);
    endInsertRows();

    /*
      In Windows, the view's resizeColumnsToContents function must be executed
      to readjust the column size even though the ResizeToContents mode is
      specified with the setSectionResizeMode function in advance.
      \note It may be better to use LazyCaller to execute the functions.
    */
#ifdef Q_OS_WIN32
    widget->resizeColumnToContents(IdColumn);
    for(size_t i=0; i < bodyPartIndices.size(); ++i){
        widget->resizeColumnToContents(MainPositionColumn + i);
    }
#endif
}


void PositionListModel::onPositionRemoved(int positionIndex)
{
    beginRemoveRows(QModelIndex(), positionIndex, positionIndex);
    endRemoveRows();

#ifdef Q_OS_WIN32
    widget->resizeColumnToContents(IdColumn);
    for(size_t i=0; i < bodyPartIndices.size(); ++i){
        widget->resizeColumnToContents(MainPositionColumn + i);
    }
#endif
}


void PositionListModel::onPositionUpdated(int positionIndex, int flags)
{
    if(flags & MprPosition::IdUpdate){
        auto modelIndex = index(positionIndex, IdColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });

#ifdef Q_OS_WIN32
        widget->resizeColumnToContents(IdColumn);
#endif
    }
    if(flags & MprPosition::NoteUpdate){
        auto modelIndex = index(positionIndex, NoteColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
    if(flags & MprPosition::PositionUpdate){
        auto modelIndex1 = index(positionIndex, MainPositionColumn, QModelIndex());
        auto modelIndex2 = index(positionIndex, columnCount_ - 1, QModelIndex());
        Q_EMIT dataChanged(modelIndex1, modelIndex2, { Qt::EditRole });

#ifdef Q_OS_WIN32
        for(size_t i=0; i < bodyPartIndices.size(); ++i){
            widget->resizeColumnToContents(MainPositionColumn + i);
        }
#endif
    }
    if(flags & MprPosition::ObjectReplaced){
        auto modelIndex1 = index(positionIndex, IdColumn, QModelIndex());
        auto modelIndex2 = index(positionIndex, columnCount_ - 1, QModelIndex());
        Q_EMIT dataChanged(modelIndex1, modelIndex2, { Qt::EditRole });

#ifdef Q_OS_WIN32
        widget->resizeColumnToContents(IdColumn);
        for(size_t i=0; i < bodyPartIndices.size(); ++i){
            widget->resizeColumnToContents(MainPositionColumn + i);
        }
#endif
    }
}


CheckItemDelegate::CheckItemDelegate(MprPositionListWidget* widget)
    : QStyledItemDelegate(widget),
      widget(widget)
{

}


void CheckItemDelegate::paint
(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QVariant value = index.data();
    bool isChecked = value.toBool();
    QStyle* style = widget->style();
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
    QStyle* style = widget->style();
    QRect checkBoxRect = style->subElementRect(QStyle::SE_CheckBoxIndicator, &option);
    QSize size = checkBoxRect.size();
    size.setWidth(size.width() * 2);
    return size;
}


MprPositionListWidget::MprPositionListWidget(QWidget* parent)
    : QTableView(parent)
{
    impl = new Impl(this);

    setItemDelegateForColumn(JointSpaceCheckColumn, new CheckItemDelegate(this));
    setModel(impl->positionListModel);

    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setTabKeyNavigation(true);
    setCornerButtonEnabled(true);
    setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    setEditTriggers(
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    auto hheader = horizontalHeader();
    hheader->setMinimumSectionSize(24);
    hheader->setSectionResizeMode(QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(NoteColumn, QHeaderView::Stretch);

    verticalHeader()->hide();

    connect(this, &QTableView::pressed,
            [this](const QModelIndex& index){
                if(index.isValid() && QGuiApplication::mouseButtons() == Qt::LeftButton){
                    impl->sigPositionPressed(index.row(), impl->isSelectionChangedAlreadyCalled);
                }
            });

    connect(this, &QTableView::doubleClicked,
            [this](const QModelIndex& index){
                if(index.isValid()){
                    impl->sigPositionDoubleClicked(index.row());
                }
            });

    impl->bodySyncMode = DirectBodySync;
}


MprPositionListWidget::Impl::Impl(MprPositionListWidget* self)
    : self(self)
{
    valueFormat = DisplayValueFormat::instance();
    positionListModel = new PositionListModel(this);
    isStandardUserOperationEnabled = false;
    isSelectionChangedAlreadyCalled = false;
}


MprPositionListWidget::~MprPositionListWidget()
{
    delete impl;
}


int MprPositionListWidget::getIkPositionColumnWidth() const
{
    QFontMetrics fm(font());
    int width;
    if(impl->valueFormat->isMillimeter()){
        width = fm.horizontalAdvance("+00000.0 +00000.0 +00000.0 +000.0 000.0 000.0 : AB 00 00");
    } else {
        width = fm.horizontalAdvance("+00.000 +00.000 +00.000 +000.0 000.0 000.0 : AB 00 00");
    }
    QStyleOptionViewItem opt;
    opt.initFrom(this);
    int margin = style()->pixelMetric(QStyle::PM_FocusFrameHMargin, &opt, this);
    return width + 2 * margin;
}


void MprPositionListWidget::setSinglePositionHeaderLabel(const std::string& label)
{
    impl->positionListModel->singlePositionHeaderLabel = label.c_str();
}


bool MprPositionListWidget::isStandardUserOperationEnabled() const
{
    return impl->isStandardUserOperationEnabled;
}


void MprPositionListWidget::setStandardUserOperationEnabled(bool on)
{
    impl->setStandardUserOperationEnabled(on);
}


void MprPositionListWidget::Impl::setStandardUserOperationEnabled(bool on)
{
    if(on != isStandardUserOperationEnabled){
        if(!on){
            operationConnections.disconnect();
        } else {
            operationConnections.add(
                sigSelectionChanged.connect(
                    [this](const QItemSelection& selected, const QItemSelection& deselected){
                        auto indexes = selected.indexes();
                        if(!indexes.empty()){
                            self->applyPosition(indexes.front().row(), false);
                        }
                    }));
            operationConnections.add(
                sigPositionPressed.connect(
                    [this](int index, bool isSelectionChanged){
                        if(!isSelectionChanged){
                            self->applyPosition(index, false);
                        }
                    }));
            operationConnections.add(
                sigPositionDoubleClicked.connect(
                    [this](int index){
                        self->applyPosition(index, true);
                    }));
            operationConnections.add(
                sigContextMenuRequest.connect(
                    [this](int index, const QPoint& globalPos){
                        self->showDefaultContextMenu(index, globalPos);
                    }));
        }
        isStandardUserOperationEnabled = on;
    }
}


void MprPositionListWidget::setBodyItemSet(KinematicBodyItemSet* bodyItemSet)
{
    impl->bodyItemSet = bodyItemSet;
    impl->positionListModel->setBodyItemSet(bodyItemSet);
}


void MprPositionListWidget::setPositionList(MprPositionList* positionList)
{
    impl->positionList = positionList;
    impl->positionListModel->setPositionList(positionList);
}


void MprPositionListWidget::addPosition(int row, bool doInsert)
{
    impl->addPosition(row, doInsert);
}


void MprPositionListWidget::Impl::addPosition(int row, bool doInsert)
{
    if(bodyItemSet && positionList){
        MprPositionPtr position;
        if(!bodyItemSet->hasMultiBodyParts()){
            position = new MprIkPosition;
        } else {
            auto composite = new MprCompositePosition;
            for(auto index : bodyItemSet->validBodyPartIndices()){
                auto kinematicsKit = bodyItemSet->bodyPart(index);
                if(kinematicsKit && kinematicsKit->isManipulator()){
                    composite->setPosition(index, new MprIkPosition);
                } else {
                    composite->setPosition(index, new MprFkPosition);
                }
            }
            position = composite;
        }
        if(position->fetch(bodyItemSet, MessageOut::interactive())){
            position->setId(positionList->createNextId());
            positionListModel->addPosition(row, position, doInsert);
        }
    }
}
    

void MprPositionListWidget::addPositionIntoCurrentIndex(bool doInsert)
{
    auto current = selectionModel()->currentIndex();
    int row = current.isValid() ? current.row() : impl->positionListModel->numPositions();
    impl->addPosition(row, doInsert);
}


void MprPositionListWidget::removeSelectedPositions()
{
    if(impl->positionListModel){
        impl->positionListModel->removePositions(selectionModel()->selectedRows());
    }
}


void MprPositionListWidget::keyPressEvent(QKeyEvent* event)
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


void MprPositionListWidget::mousePressEvent(QMouseEvent* event)
{
    impl->isSelectionChangedAlreadyCalled = false;
    
    QTableView::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        if(impl->positionList){
            impl->sigContextMenuRequest(rowAt(event->pos().y()), getGlobalPosition(event));
        }
    }
}


void MprPositionListWidget::showDefaultContextMenu(int row, const QPoint& globalPos)
{
    impl->contextMenuManager.setNewPopupMenu(this);

    impl->contextMenuManager.addItem(_("Add"))
        ->sigTriggered().connect([this, row](){
            impl->addPosition(row >= 0 ? row : 0, false); });

    if(row >= 0){
        impl->contextMenuManager.addItem(_("Remove"))
            ->sigTriggered().connect([this](){
                removeSelectedPositions(); });
    }
    
    impl->contextMenuManager.popupMenu()->popup(globalPos);
}


void MprPositionListWidget::selectionChanged
(const QItemSelection& selected, const QItemSelection& deselected)
{
    impl->isSelectionChangedAlreadyCalled = true;
    
    QTableView::selectionChanged(selected, deselected);

    impl->sigSelectionChanged(selected, deselected);
}


void MprPositionListWidget::setBodySyncMode(BodySyncMode mode)
{
    if(mode != impl->bodySyncMode){
        impl->bodySyncMode = mode;
        if(!mode && impl->bodyItemSet){
            clearSuperimposition(impl->bodyItemSet);
        }
    }
}


MprPositionListWidget::BodySyncMode MprPositionListWidget::bodySyncMode() const
{
    return impl->bodySyncMode;
}


bool MprPositionListWidget::applyPosition(int positionIndex, bool forceDirectSync)
{
    bool result = false;
    if(impl->bodyItemSet && impl->positionList){
        auto position = impl->positionList->positionAt(positionIndex);
        if(impl->bodySyncMode == DirectBodySync || forceDirectSync){
            result = cnoid::applyPosition(impl->bodyItemSet, position, true, MessageOut::interactive());
        } else {
            cnoid::superimposePosition(impl->bodyItemSet, position);
        }
    }
    return result;
}


void MprPositionListWidget::touchupCurrentPosition()
{
    if(impl->bodyItemSet && impl->positionList){
        if(auto position = impl->positionListModel->positionAt(selectionModel()->currentIndex())){
            touchupPosition(impl->bodyItemSet, position, MessageOut::interactive());
        }
    }
}


SignalProxy<void(int index, const QPoint& globalPos)> MprPositionListWidget::sigContextMenuRequest()
{
    return impl->sigContextMenuRequest;
}


SignalProxy<void(int index, bool isSelectionChanged)> MprPositionListWidget::sigPositionPressed()
{
    return impl->sigPositionPressed;
}


SignalProxy<void(int index)> MprPositionListWidget::sigPositionDoubleClicked()
{
    return impl->sigPositionDoubleClicked;
}


SignalProxy<void(const QItemSelection& selected, const QItemSelection& deselected)> MprPositionListWidget::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


