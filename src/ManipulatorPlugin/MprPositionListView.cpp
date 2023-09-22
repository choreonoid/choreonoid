#include "MprPositionListView.h"
#include "MprPositionList.h"
#include "MprPosition.h"
#include "MprProgramItemBase.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/DisplayValueFormat>
#include <cnoid/KinematicBodyItemSet>
#include <cnoid/BodyItemKinematicsKit>
#include <cnoid/Archive>
#include <cnoid/Buttons>
#include <cnoid/ConnectionSet>
#include <cnoid/BodyItem>
#include <cnoid/EigenUtil>
#include <cnoid/MessageOut>
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

MprPositionListView::BodySyncMode defaultBodySyncMode = MprPositionListView::DirectBodySync;

constexpr int IdColumn = 0;
constexpr int NoteColumn = 1;
constexpr int JointSpaceCheckColumn = 2;
constexpr int MainPositionColumn = 3;
constexpr int NumMinimumColumns = 4;

class PositionListModel : public QAbstractTableModel
{
public:
    MprPositionListView::Impl* view;
    MprProgramItemBasePtr programItem;
    int columnCount_;
    vector<int> bodyPartIndices;
    int mainBodyPartIndex;
    QStringList positionHeaderStrings;
    MprPositionListPtr positionList;
    ScopedConnectionSet connections;
    QFont monoFont;
    DisplayValueFormat* valueFormat;
    
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
    columnCount_ = NumMinimumColumns;
    monoFont.setStyleHint(QFont::TypeWriter);
    valueFormat = DisplayValueFormat::instance();
}


void PositionListModel::setProgramItem(MprProgramItemBase* programItem)
{
    beginResetModel();

    /*
      In the Visual C++ debug mode, the following function call may cause a crash when
      setProgramItem is called at the time the position list view is hidden. The crash
      will always occur when the view is visible and the application is finished. The
      crash never occur in the Visual C++ release mode or other environments. Also, if
      the connection to bodyItemSet->sigBodySetChanged() is not held in connections,
      the crash does not occur. The cause of the crash is currently unknown.
    */
    connections.disconnect();
    
    this->programItem = programItem;
    columnCount_ = NumMinimumColumns;
    bodyPartIndices.clear();
    mainBodyPartIndex = -1;
    positionHeaderStrings.clear();

    if(!programItem){
        this->positionList.reset();

    } else {
        if(auto bodyItemSet = programItem->targetBodyItemSet()){
            bodyPartIndices = bodyItemSet->validBodyPartIndices();
            mainBodyPartIndex = bodyItemSet->mainBodyPartIndex();
            if(bodyPartIndices.size() >= 2){
                for(auto index : bodyPartIndices){
                    positionHeaderStrings.append(bodyItemSet->bodyPart(index)->body()->name().c_str());
                }
                columnCount_ += positionHeaderStrings.size() - 1;
            }
            /*
              Adding the following connection to connections causes the crash
              described above. If the connection is held in a single connection
              holder such as ScopedConnection, the result is the same.
            */
            connections.add(
                bodyItemSet->sigBodySetChanged().connect(
                    [this](){
                        if(this->programItem){
                            setProgramItem(this->programItem);
                        }
                    }));
        }
        this->positionList = programItem->program()->positionList();
    }

    if(positionHeaderStrings.empty()){
        positionHeaderStrings.append(_("Position"));
    }

    if(positionList){
        connections.add(
            positionList->sigPositionAdded().connect(
                [this](int index){ onPositionAdded(index); }));
        connections.add(
            positionList->sigPositionRemoved().connect(
                [this](int index, MprPosition*){ onPositionRemoved(index); }));
        connections.add(
            positionList->sigPositionUpdated().connect(
                [this](int index, int flags){ onPositionUpdated(index, flags); }));
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
                return " ID ";
            case NoteColumn:
                return _("Note");
            case JointSpaceCheckColumn:
                return _("J");
            default:
                {
                    int posColumnIndex = section - MainPositionColumn;
                    if(posColumnIndex < static_cast<int>(positionHeaderStrings.size())){
                        return positionHeaderStrings[posColumnIndex];
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
        if(column == NoteColumn || column >= MainPositionColumn){
            return (Qt::AlignLeft + Qt::AlignVCenter);
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
                return format("{0: 9.3f} {1: 9.3f} {2: 9.3f} "
                              "{3: 6.1f} {4: 6.1f} {5: 6.1f} "
                              ": {6:2X} {7:2d} {8:2d}",
                              p.x() * 1000.0, p.y() * 1000.0, p.z() * 1000.0,
                              rpy[0], rpy[1], rpy[2],
                              ik->configuration(), baseId.toInt(), offsetId.toInt()).c_str();
            } else {
                return format("{0: 6.3f} {1: 6.3f} {2: 6.3f} "
                              "{3: 6.1f} {4: 6.1f} {5: 6.1f} "
                              ": {6:2X} {7:2d} {8:2d}",
                              p.x(), p.y(), p.z(),
                              rpy[0], rpy[1], rpy[2],
                              ik->configuration(), baseId.toInt(), offsetId.toInt()).c_str();
            }
        } else {
            if(valueFormat->isMillimeter()){
                return format("{0: 9.3f} {1: 9.3f} {2: 9.3f} "
                              "{3: 6.1f} {4: 6.1f} {5: 6.1f} : {6:2X}",
                              p.x() * 1000.0, p.y() * 1000.0, p.z() * 1000.0,
                              rpy[0], rpy[1], rpy[2],
                              ik->configuration()).c_str();
            } else {
                return format("{0: 6.3f} {1: 6.3f} {2: 6.3f} "
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
                data += format("{0: 6.1f}", degree(q));
            } else {
                if(valueFormat->isMillimeter()){
                    data += format("{0: 9.3f}", q * 1000.0);
                } else {
                    data += format("{0: 6.3f}", q);
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
    auto bodyItemSet = programItem->targetBodyItemSet();
    if(!bodyItemSet){
        return;
    }
    
    if(!view->applyPosition(positionIndex, true)){
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
    view->resizeColumnToContents(IdColumn);
    for(size_t i=0; i < bodyPartIndices.size(); ++i){
        view->resizeColumnToContents(MainPositionColumn + i);
    }
#endif
}


void PositionListModel::onPositionRemoved(int positionIndex)
{
    beginRemoveRows(QModelIndex(), positionIndex, positionIndex);
    endRemoveRows();

#ifdef Q_OS_WIN32
    view->resizeColumnToContents(IdColumn);
    for(size_t i=0; i < bodyPartIndices.size(); ++i){
        view->resizeColumnToContents(MainPositionColumn + i);
    }
#endif
}


void PositionListModel::onPositionUpdated(int positionIndex, int flags)
{
    if(flags & MprPosition::IdUpdate){
        auto modelIndex = index(positionIndex, IdColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });

#ifdef Q_OS_WIN32
        view->resizeColumnToContents(IdColumn);
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
            view->resizeColumnToContents(MainPositionColumn + i);
        }
#endif
    }
    if(flags & MprPosition::ObjectReplaced){
        auto modelIndex1 = index(positionIndex, IdColumn, QModelIndex());
        auto modelIndex2 = index(positionIndex, columnCount_ - 1, QModelIndex());
        Q_EMIT dataChanged(modelIndex1, modelIndex2, { Qt::EditRole });

#ifdef Q_OS_WIN32
        view->resizeColumnToContents(IdColumn);
        for(size_t i=0; i < bodyPartIndices.size(); ++i){
            view->resizeColumnToContents(MainPositionColumn + i);
        }
#endif
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


void MprPositionListView::setDefaultBodySyncMode(BodySyncMode mode)
{
    defaultBodySyncMode = mode;
}


void MprPositionListView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<MprPositionListView>(
        N_("MprPositionListView"), N_("Waypoints"));
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
    self->setDefaultLayoutArea(BottomCenterArea);
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
    addButton.sigClicked().connect([this](){ addPositionIntoCurrentIndex(false); });
    hbox->addWidget(&addButton);
    touchupButton.setText(_("Touch-up"));
    touchupButton.sigClicked().connect([this](){ touchupCurrentPosition(); });
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
    hheader->setSectionResizeMode(QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(NoteColumn, QHeaderView::Stretch);
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
        [this](MprProgramItemBase* item){ setProgramItem(item); });

    bodySyncMode = defaultBodySyncMode;
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
        [this](bool on){ impl->setBodySyncMode(on ? TwoStageBodySync : DirectBodySync); });
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
        if(auto bodyItemSet = programItem->targetBodyItemSet()){
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
        if(positionList){
            showContextMenu(rowAt(event->pos().y()), event->globalPos());
        }
    }
}


void MprPositionListView::Impl::showContextMenu(int row, QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);

    contextMenuManager.addItem(_("Add"))
        ->sigTriggered().connect([=](){ addPosition(row >= 0 ? row : 0, false); });

    if(row >= 0){
        contextMenuManager.addItem(_("Remove"))
            ->sigTriggered().connect([=](){ removeSelectedPositions(); });
    }
    
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
        result = programItem->moveTo(position, MessageOut::interactive());
    } else {
        programItem->superimposePosition(position, MessageOut::interactive());
    }
    return result;
}


void MprPositionListView::Impl::touchupCurrentPosition()
{
    if(positionList){
        if(auto position = positionListModel->positionAt(selectionModel()->currentIndex())){
            programItem->touchupPosition(position, MessageOut::interactive());
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
