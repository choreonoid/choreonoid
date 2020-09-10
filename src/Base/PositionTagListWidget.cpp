#include "PositionTagListWidget.h"
#include "MenuManager.h"
#include "PositionTagListItem.h"
#include <cnoid/PositionTagList>
#include <cnoid/PositionTag>
#include <cnoid/ConnectionSet>
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

constexpr int NumColumns = 2;
constexpr int IndexColumn = 0;
constexpr int CoordColumn = 1;

class TagListModel : public QAbstractTableModel
{
public:
    PositionTagListWidget* widget;
    PositionTagListItemPtr tagListItem;
    ScopedConnectionSet tagListConnections;
    QFont monoFont;
    
    TagListModel(PositionTagListWidget* widget);
    void setTagListItem(PositionTagListItem* tagListItem);
    int numTags() const;
    PositionTag* tagAt(const QModelIndex& index) const;
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    void onTagAdded(int tagIndex);
    void onTagRemoved(int tagIndex);
    void onTagUpdated(int tagIndex);
};

}

namespace cnoid {

class PositionTagListWidget::Impl
{
public:
    PositionTagListItemPtr tagListItem;
    TagListModel* tagListModel;
    bool isSelectionChangedAlreadyCalled;
    MenuManager contextMenuManager;
    Signal<void(int tagIndex)> sigTagPressed;
    Signal<void(int tagIndex)> sigTagDoubleClicked;
    Signal<void(MenuManager& menu)> sigContextMenuRequest;
};

}


TagListModel::TagListModel(PositionTagListWidget* widget)
    : QAbstractTableModel(widget),
      widget(widget),
      monoFont("Monospace")
{
    monoFont.setStyleHint(QFont::TypeWriter);
}


void TagListModel::setTagListItem(PositionTagListItem* tagListItem)
{
    beginResetModel();

    this->tagListItem = tagListItem;

    tagListConnections.disconnect();
    if(tagListItem){
        auto tags = tagListItem->tags();
        tagListConnections.add(
            tags->sigTagAdded().connect(
                [&](int index){ onTagAdded(index); }));
        tagListConnections.add(
            tags->sigTagRemoved().connect(
                [&](int index, PositionTag*){ onTagRemoved(index); }));
        tagListConnections.add(
            tags->sigTagUpdated().connect(
                [&](int index){ onTagUpdated(index); }));
    }
            
    endResetModel();
}


int TagListModel::numTags() const
{
    if(tagListItem){
        return tagListItem->tags()->numTags();
    }
    return 0;
}


PositionTag* TagListModel::tagAt(const QModelIndex& index) const
{
    if(!index.isValid()){
        return nullptr;
    }
    return tagListItem->tags()->tagAt(index.row());
}
        
    
int TagListModel::rowCount(const QModelIndex& parent) const
{
    int n = 0;
    if(!parent.isValid()){
        n = numTags();
    }
    if(n == 0){ // to show an empty row
        n = 1;
    }
    return n;
}


int TagListModel::columnCount(const QModelIndex& parent) const
{
    return NumColumns;
}
        

QVariant TagListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case IndexColumn:
                return " No ";
            case CoordColumn:
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


QModelIndex TagListModel::index(int row, int column, const QModelIndex& parent) const
{
    if(!tagListItem || parent.isValid()){
        return QModelIndex();
    }
    if(row < numTags()){
        return createIndex(row, column);
    }
    return QModelIndex();
}
    

QVariant TagListModel::data(const QModelIndex& index, int role) const
{
    auto tag = tagAt(index);
    if(!tag){
        return QVariant();
    }
    int column = index.column();
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(column){
        case IndexColumn:
            return index.row();

        case CoordColumn: {
            auto p = tag->translation();
            return format("{0: 1.3f} {1: 1.3f} {2: 1.3f}", p.x(), p.y(), p.z()).c_str();
        }

        default:
            break;
        }
    } else if(role == Qt::TextAlignmentRole){
        if(column == CoordColumn){
            return (Qt::AlignLeft + Qt::AlignVCenter);
        } else {
            return Qt::AlignCenter;
        }
    } else if(role == Qt::FontRole){
        if(column == CoordColumn){
            return monoFont;
        }
    }
    return QVariant();
}


void TagListModel::onTagAdded(int tagIndex)
{
    if(numTags() == 0){
        // Remove the empty row first
        beginRemoveRows(QModelIndex(), 0, 0);
        endRemoveRows();
    }
    beginInsertRows(QModelIndex(), tagIndex, tagIndex);
    endInsertRows();

    //resizeColumnToContents(IdColumn);
    //resizeColumnToContents(PositionColumn);
}


void TagListModel::onTagRemoved(int tagIndex)
{
    beginRemoveRows(QModelIndex(), tagIndex, tagIndex);
    endRemoveRows();
    if(numTags() == 0){
        // This is necessary to show the empty row
        beginResetModel();
        endResetModel();
    }
}


void TagListModel::onTagUpdated(int tagIndex)
{
    auto modelIndex = index(tagIndex, CoordColumn, QModelIndex());
    Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
}


PositionTagListWidget::PositionTagListWidget(QWidget* parent)
    : QTableView(parent)
{
    impl = new Impl;
    
    //setFrameShape(QFrame::NoFrame);
    
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setTabKeyNavigation(true);
    setCornerButtonEnabled(true);
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    /*
    setEditTriggers(
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);
    */

    impl->tagListModel = new TagListModel(this);
    setModel(impl->tagListModel);

    auto hheader = horizontalHeader();
    hheader->setMinimumSectionSize(24);
    hheader->setSectionResizeMode(IndexColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(CoordColumn, QHeaderView::Stretch);
    auto vheader = verticalHeader();
    vheader->setSectionResizeMode(QHeaderView::ResizeToContents);
    vheader->hide();

    connect(this, &QTableView::pressed,
            [this](const QModelIndex& index){
                if(index.isValid() && QGuiApplication::mouseButtons() == Qt::LeftButton){
                    if(!impl->isSelectionChangedAlreadyCalled){
                        impl->sigTagPressed(index.row());
                    }
                }
            });

    connect(this, &QTableView::doubleClicked,
            [this](const QModelIndex& index){
                if(index.isValid()){
                    impl->sigTagDoubleClicked(index.row());
                }
            });
}


void PositionTagListWidget::setTagListItem(PositionTagListItem* item)
{
    impl->tagListItem = item;
    impl->tagListModel->setTagListItem(item);
}


int PositionTagListWidget::currentTagIndex() const
{
    auto current = selectionModel()->currentIndex();
    return current.isValid() ? current.row() : impl->tagListModel->numTags();
}


void PositionTagListWidget::removeSelectedTags()
{
    if(impl->tagListItem){
        auto tags = impl->tagListItem->tags();
        auto selected = selectionModel()->selectedRows();
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int tagIndex = index.row() - numRemoved;
            tags->removeAt(tagIndex);
            ++numRemoved;
        }
    }
}


void PositionTagListWidget::keyPressEvent(QKeyEvent* event)
{
    bool processed = true;

    switch(event->key()){
    case Qt::Key_Escape:
        clearSelection();
        break;
    case Qt::Key_Delete:
        removeSelectedTags();
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

       
void PositionTagListWidget::mousePressEvent(QMouseEvent* event)
{
    impl->isSelectionChangedAlreadyCalled = false;
    
    QTableView::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        int row = rowAt(event->pos().y());
        if(row >= 0){
            // Show the context menu
            impl->contextMenuManager.setNewPopupMenu(this);
            impl->sigContextMenuRequest(impl->contextMenuManager);
            impl->contextMenuManager.popupMenu()->popup(event->globalPos());
        }
    }
}


void PositionTagListWidget::selectionChanged(const QItemSelection& selected, const QItemSelection& deselected)
{
    impl->isSelectionChangedAlreadyCalled = true;
    
    QTableView::selectionChanged(selected, deselected);

    auto indexes = selected.indexes();
    //sigSelectionChanged(indexes);
}


SignalProxy<void(int tagIndex)> PositionTagListWidget::sigTagPressed()
{
    return impl->sigTagPressed;
}


SignalProxy<void(int tagIndex)> PositionTagListWidget::sigTagDoubleClicked()
{
    return impl->sigTagDoubleClicked;
}



SignalProxy<void(MenuManager& menu)> PositionTagListWidget::sigContextMenuRequest()
{
    return impl->sigContextMenuRequest;
}

