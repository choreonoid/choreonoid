#include "ManipulatorProgramViewBase.h"
#include "ManipulatorProgramItemBase.h"
#include "ManipulatorProgram.h"
#include "ManipulatorStatements.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/TreeWidget>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <cnoid/BodyItem>
#include <cnoid/Buttons>
#include <QBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QProxyStyle>
#include <QStyledItemDelegate>
#include <QPainter>
#include <QLineEdit>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class StatementItem : public QTreeWidgetItem
{
public:
    ManipulatorStatementPtr statement;
    ManipulatorProgramViewBase::Impl* viewImpl;

    StatementItem(ManipulatorStatement* statement, ManipulatorProgramViewBase::Impl* viewImpl);
    ~StatementItem();
    virtual QVariant data(int column, int role) const override;
    StructuredStatement* parentStatement();
};


class StatementItemDelegate : public QStyledItemDelegate
{
    ManipulatorProgramViewBase::Impl* view;
public:
    StatementItemDelegate(ManipulatorProgramViewBase::Impl* view);
    virtual void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void updateEditorGeometry(
        QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};


class TreeWidgetStyle : public QProxyStyle
{
public:
    TreeWidgetStyle(QStyle* style) : QProxyStyle(style) { }
    void drawPrimitive(PrimitiveElement element, const QStyleOption* option, QPainter* painter, const QWidget* widget = 0) const;
};


class ScopedCounter
{
    int& counter;
public:
    ScopedCounter(int& counter) : counter(counter){
        ++counter;
    }
    ~ScopedCounter(){
        --counter;
    }
};

}

namespace cnoid {

class ManipulatorProgramViewBase::Impl : public TreeWidget
{
public:
    ManipulatorProgramViewBase* self;
    TargetItemPicker<ManipulatorProgramItemBase> targetItemPicker;
    ManipulatorProgramItemBasePtr programItem;
    unordered_map<ManipulatorStatementPtr, StatementItem*> statementItemMap;
    int statementItemOperationCallCounter;
    ScopedConnectionSet programItemConnections;
    ManipulatorStatementPtr currentStatement;
    Signal<void(ManipulatorStatement* statement)> sigCurrentStatementChanged;
    ManipulatorStatementPtr prevCurrentStatement;
    QLabel programNameLabel;
    ToolButton optionMenuButton;
    MenuManager optionMenuManager;
    MenuManager contextMenuManager;
    QHBoxLayout buttonBox[2];

    Impl(ManipulatorProgramViewBase* self);
    ~Impl();
    void setupWidgets();
    void onOptionMenuClicked();
    ScopedCounter scopedCounterOfStatementItemOperationCall();
    bool isDoingStatementItemOperation() const;
    void setProgramItem(ManipulatorProgramItemBase* item);
    void updateStatementTree();
    void addProgramStatementsToTree(ManipulatorProgram* program, QTreeWidgetItem* parentItem);
    void onCurrentTreeWidgetItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void setCurrentStatement(ManipulatorStatement* statement);
    void onTreeWidgetItemClicked(QTreeWidgetItem* item, int /* column */);
    StatementItem* statementItemFromStatement(ManipulatorStatement* statement);
    bool insertStatement(ManipulatorStatement* statement, int insertionType);
    void onStatementAdded(ManipulatorProgram::iterator iter, StructuredStatement* parentStatement);
    void onStatementRemoved(ManipulatorStatement* statement, StructuredStatement* parentStatement);
    void forEachStatementInTreeEditEvent(
        const QModelIndex& parent, int start, int end,
        function<bool(ManipulatorProgram* program, int index, ManipulatorStatement* statement)> func);
    void onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
    void onRowsInserted(const QModelIndex& parent, int start, int end);
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(StatementItem* statementItem, QPoint globalPos);
    void cutStatement(ManipulatorStatement* statementItem);
    void copyStatement(ManipulatorStatement* statementItem);
    void pasteStatement(ManipulatorStatement* statementItem);

    QModelIndex indexFromItem(const QTreeWidgetItem* item, int column = 0) const {
        return TreeWidget::indexFromItem(item, column);
    }
    QTreeWidgetItem* itemFromIndex(const QModelIndex& index) const {
        return TreeWidget::itemFromIndex(index);
    }
};

}


StatementItem::StatementItem(ManipulatorStatement* statement, ManipulatorProgramViewBase::Impl* viewImpl)
    : statement(statement),
      viewImpl(viewImpl)
{
    viewImpl->statementItemMap[statement] = this;
    setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled | Qt::ItemIsEditable);
}


StatementItem::~StatementItem()
{
    viewImpl->statementItemMap.erase(statement);
}


QVariant StatementItem::data(int column, int role) const
{
    if(role == Qt::DisplayRole){
        int span = statement->labelSpan(column);
        if(span == 1){
            return QString(statement->label(column).c_str());
        } else {
            return QVariant();
        }
    } else if(role == Qt::EditRole){
        return QString(statement->label(column).c_str());
    }
    return QTreeWidgetItem::data(column, role);
}


StructuredStatement* StatementItem::parentStatement()
{
    if(auto parentStatementItem = dynamic_cast<StatementItem*>(parent())){
        return dynamic_cast<StructuredStatement*>(parentStatementItem->statement.get());
    }
    return nullptr;
}


StatementItemDelegate::StatementItemDelegate(ManipulatorProgramViewBase::Impl* view)
    : view(view)
{

}


void StatementItemDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    auto item = static_cast<StatementItem*>(view->itemFromIndex(index));
    auto statement = item->statement;
    
    int column = index.column();
    int span = statement->labelSpan(column);
    if(span == 1){
        QStyledItemDelegate::paint(painter, option, index);
    } else if(span > 1){
        auto rect = view->visualRect(index);
        for(int i=1; i < span; ++i){
            auto rect2 = view->visualRect(view->indexFromItem(item, column + i));
            rect = rect.united(rect2);
        }
        // This doesn't work because the HasDecoration bit is always zero
        bool hasDecoration = option.features & QStyleOptionViewItem::HasDecoration;
        if(true /* hasDecoration */){
            int decorationWidth = option.decorationSize.width();
            if(decorationWidth > 0){
                rect.setLeft(rect.left() + decorationWidth);
            }
        }
        painter->save();
        if(option.state & QStyle::State_Selected){
            painter->fillRect(rect, option.palette.highlight());
            painter->setPen(option.palette.highlightedText().color());
        }
        painter->drawText(rect, 0, statement->label(column).c_str());
        painter->restore();
    }
}


QWidget* StatementItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    auto item = static_cast<StatementItem*>(view->itemFromIndex(index));
    auto statement = item->statement;
    int column = index.column();
    int span = statement->labelSpan(column);
    if(span == 0){
        return nullptr;
    } else if(column == 0 && span == 1){
        return nullptr;
    }

   /*
   if(auto comment = dynamic_cast<CommentStatement*>(statement)){
       if(index.column() == 0){
       }
   }
   */

   return QStyledItemDelegate::createEditor(parent, option, index);
}


void StatementItemDelegate::updateEditorGeometry
(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    auto item = static_cast<StatementItem*>(view->itemFromIndex(index));
    auto statement = item->statement;
    int column = index.column();
    int span = statement->labelSpan(column);
    QRect rect;
    if(span == 0){
        
    } else if(span == 1){
        rect = option.rect;
        editor->setGeometry(rect);
    } else {
        rect = view->visualRect(index);
        for(int i=1; i < span; ++i){
            auto rect2 = view->visualRect(view->indexFromItem(item, column + i));
            rect = rect.united(rect2);
        }
        editor->setGeometry(rect);
    }
}


void StatementItemDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    QStyledItemDelegate::setEditorData(editor, index);
}


void StatementItemDelegate::setModelData
(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    auto item = static_cast<StatementItem*>(view->itemFromIndex(index));
    auto statement = item->statement;
    int column = index.column();

    if(auto comment = dynamic_cast<CommentStatement*>(statement.get())){
       if(index.column() == 0){
           if(auto lineEdit = dynamic_cast<QLineEdit*>(editor)){
               comment->setComment(lineEdit->text().toStdString());
           }
       }
   }
}



void TreeWidgetStyle::drawPrimitive
(PrimitiveElement element, const QStyleOption* option, QPainter* painter, const QWidget* widget) const
{
    if(element == QStyle::PE_IndicatorItemViewItemDrop && !option->rect.isNull()){
        QStyleOption opt(*option);
        opt.rect.setLeft(0);
        if(widget){
            opt.rect.setRight(widget->width());
        }
        QProxyStyle::drawPrimitive(element, &opt, painter, widget);
        return;
    }
    QProxyStyle::drawPrimitive(element, option, painter, widget);
}


ManipulatorProgramViewBase::ManipulatorProgramViewBase()
{
    impl = new Impl(this);
}


ManipulatorProgramViewBase::Impl::Impl(ManipulatorProgramViewBase* self)
    : self(self),
      targetItemPicker(self)
{
    setupWidgets();

    statementItemOperationCallCounter = 0;

    targetItemPicker.sigTargetItemChanged().connect(
        [&](ManipulatorProgramItemBase* item){ setProgramItem(item); });
}


ManipulatorProgramViewBase::~ManipulatorProgramViewBase()
{
    delete impl;
}


ManipulatorProgramViewBase::Impl::~Impl()
{

}


void ManipulatorProgramViewBase::Impl::setupWidgets()
{
    self->setDefaultLayoutArea(View::LEFT_TOP);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    auto hbox = new QHBoxLayout;
    hbox->addLayout(&buttonBox[0]);
    hbox->addStretch();
    optionMenuButton.setText(_("*"));
    optionMenuButton.sigClicked().connect([&](){ onOptionMenuClicked(); });
    hbox->addWidget(&optionMenuButton);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addLayout(&buttonBox[1]);
    hbox->addStretch();
    vbox->addLayout(hbox);

    programNameLabel.setFrameStyle(QFrame::Box | QFrame::Sunken);
    vbox->addWidget(&programNameLabel);

    // TreeWidget setup
    setColumnCount(3);
    setFrameShape(QFrame::NoFrame);
    setHeaderHidden(true);
    setRootIsDecorated(true);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setDragDropMode(QAbstractItemView::InternalMove);
    setDropIndicatorShown(true);
    setTabKeyNavigation(true);
    setStyle(new TreeWidgetStyle(style()));
    setItemDelegate(new StatementItemDelegate(this));

    
    auto& rheader = *header();
    rheader.setMinimumSectionSize(0);
    rheader.setStretchLastSection(false);
    rheader.setSectionResizeMode(0, QHeaderView::ResizeToContents);
    rheader.setSectionResizeMode(1, QHeaderView::ResizeToContents);
    rheader.setSectionResizeMode(2, QHeaderView::Stretch);
    
    sigCurrentItemChanged().connect(
        [&](QTreeWidgetItem* current, QTreeWidgetItem* previous){
            onCurrentTreeWidgetItemChanged(current, previous); });

    sigItemClicked().connect(
        [&](QTreeWidgetItem* item, int column){
            onTreeWidgetItemClicked(item, column); });

    sigRowsAboutToBeRemoved().connect(
        [&](const QModelIndex& parent, int start, int end){
            onRowsAboutToBeRemoved(parent, start, end); });
    
    sigRowsInserted().connect(
        [&](const QModelIndex& parent, int start, int end){
            onRowsInserted(parent, start, end);  });
    
    vbox->addWidget(this);
    
    self->setLayout(vbox);

    optionMenuManager.setNewPopupMenu(this);
    optionMenuManager.addItem(_("Refresh"))->sigTriggered().connect(
        [&](){ updateStatementTree(); });
}


void ManipulatorProgramViewBase::addStatementButton(QWidget* button, int row)
{
    impl->buttonBox[row].addWidget(button);
}


void ManipulatorProgramViewBase::onDeactivated()
{
    impl->currentStatement = nullptr;
    impl->prevCurrentStatement = nullptr;
}


void ManipulatorProgramViewBase::Impl::onOptionMenuClicked()
{
    optionMenuManager.popupMenu()->popup(optionMenuButton.mapToGlobal(QPoint(0,0)));
}


ScopedCounter ManipulatorProgramViewBase::Impl::scopedCounterOfStatementItemOperationCall()
{
    return ScopedCounter(statementItemOperationCallCounter);
}


bool ManipulatorProgramViewBase::Impl::isDoingStatementItemOperation() const
{
    return (statementItemOperationCallCounter > 0);
}


void ManipulatorProgramViewBase::Impl::setProgramItem(ManipulatorProgramItemBase* item)
{
    programItemConnections.disconnect();
    programItem = item;
    currentStatement = nullptr;

    bool accepted = self->onCurrentProgramItemChanged(item);
    if(!accepted){
        programItem = nullptr;
    }

    if(!programItem){
        programNameLabel.setStyleSheet("");
        programNameLabel.setText("---");

    } else {
        programItemConnections.add(
            programItem->sigNameChanged().connect(
                [&](const std::string&){ setProgramItem(programItem); }));

        programItemConnections.add(
            programItem->sigStatementAdded().connect(
                [&](ManipulatorProgram::iterator iter, StructuredStatement* parentStatement){
                    onStatementAdded(iter, parentStatement); }));

        programItemConnections.add(
            programItem->sigStatementRemoved().connect(
                [&](ManipulatorStatement* statement, StructuredStatement* parentStatement){
                    onStatementRemoved(statement, parentStatement); }));
        
        programNameLabel.setStyleSheet("font-weight: bold");
        programNameLabel.setText(programItem->name().c_str());

    }

    updateStatementTree();
}


void ManipulatorProgramViewBase::Impl::updateStatementTree()
{
    auto counter = scopedCounterOfStatementItemOperationCall();
    
    clear();
    statementItemMap.clear();
    
    if(programItem){
        addProgramStatementsToTree(programItem->program(), invisibleRootItem());
    }
}


void ManipulatorProgramViewBase::Impl::addProgramStatementsToTree
(ManipulatorProgram* program, QTreeWidgetItem* parentItem)
{
    parentItem->setExpanded(true);
    for(auto& statement : *program){
        auto statementItem = new StatementItem(statement, this);
        parentItem->addChild(statementItem);
        if(auto structured = dynamic_cast<StructuredStatement*>(statement.get())){
            if(auto lowerLevelProgram = structured->getLowerLevelProgram()){
                addProgramStatementsToTree(lowerLevelProgram, statementItem);
            }
        }
    }
}


ManipulatorProgramItemBase* ManipulatorProgramViewBase::currentProgramItem()
{
    return impl->programItem;
}


ManipulatorStatement* ManipulatorProgramViewBase::currentStatement()
{
    return impl->currentStatement;
}


SignalProxy<void(ManipulatorStatement* statement)> ManipulatorProgramViewBase::sigCurrentStatementChanged()
{
    return impl->sigCurrentStatementChanged;
}


void ManipulatorProgramViewBase::Impl::onCurrentTreeWidgetItemChanged
(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    if(auto statementItem = dynamic_cast<StatementItem*>(previous)){
        prevCurrentStatement = statementItem->statement;
    }
    if(auto statementItem = dynamic_cast<StatementItem*>(current)){
        setCurrentStatement(statementItem->statement);
    }
}


void ManipulatorProgramViewBase::Impl::setCurrentStatement(ManipulatorStatement* statement)
{
    currentStatement = statement;
    self->onCurrentStatementChanged(statement);
    sigCurrentStatementChanged(statement);
    self->onCurrentStatementActivated(statement);
}


void ManipulatorProgramViewBase::onCurrentStatementChanged(ManipulatorStatement*)
{

}


void ManipulatorProgramViewBase::Impl::onTreeWidgetItemClicked(QTreeWidgetItem* item, int /* column */)
{
    if(auto statementItem = dynamic_cast<StatementItem*>(item)){
        auto statement = statementItem->statement;
        // If the clicked statement is different from the current one,
        // onCurrentTreeWidgetItemChanged is processed
        if(statement == prevCurrentStatement){
            self->onCurrentStatementActivated(statement);
        }
    }
}


void ManipulatorProgramViewBase::onCurrentStatementActivated(ManipulatorStatement*)
{

}


StatementItem* ManipulatorProgramViewBase::Impl::statementItemFromStatement(ManipulatorStatement* statement)
{
    auto iter = statementItemMap.find(statement);
    if(iter != statementItemMap.end()){
        return iter->second;
    }
    return nullptr;
}


bool ManipulatorProgramViewBase::insertStatement(ManipulatorStatement* statement, int insertionType)
{
    return impl->insertStatement(statement, insertionType);
}


bool ManipulatorProgramViewBase::Impl::insertStatement(ManipulatorStatement* statement, int insertionType)
{
    if(programItem){
        auto program = programItem->program();
        ManipulatorProgram::iterator pos;
        StructuredStatement* parentStatement = nullptr;
        EmptyStatementPtr emptyStatementAtInsertionPosition;
        auto selected = selectedItems();
        if(selected.empty()){
            if(insertionType == BeforeTargetPosition){
                pos = program->begin();
            } else {
                pos = program->end();
            }
        } else {
            auto lastSelectedItem = static_cast<StatementItem*>(selected.back());
            parentStatement = lastSelectedItem->parentStatement();
            if(parentStatement){
                program = parentStatement->getLowerLevelProgram();
            }
            auto index = indexFromItem(lastSelectedItem);
            pos = program->begin() + index.row();

            emptyStatementAtInsertionPosition =
                dynamic_cast<EmptyStatement*>(lastSelectedItem->statement.get());
            if(emptyStatementAtInsertionPosition){
                pos = program->remove(pos);
                programItem->sigStatementRemoved()(emptyStatementAtInsertionPosition, parentStatement);
            }
            if(insertionType == AfterTargetPosition && !emptyStatementAtInsertionPosition){
                ++pos;
            }
        }
        auto inserted = program->insert(pos, statement);
        programItem->sigStatementAdded()(inserted, parentStatement);
        programItem->suggestFileUpdate();
        return true;
    }
    return false;
}


void ManipulatorProgramViewBase::Impl::onStatementAdded
(ManipulatorProgram::iterator iter, StructuredStatement* parentStatement)
{
    auto counter = scopedCounterOfStatementItemOperationCall();

    QTreeWidgetItem* parentItem = invisibleRootItem();
    ManipulatorProgram* program = programItem->program();
    if(parentStatement){
        parentItem = statementItemFromStatement(parentStatement);
        program = parentStatement->getLowerLevelProgram();
    }

    auto statement = *iter;
    auto statementItem = new StatementItem(statement, this);

    bool added = false;
    auto nextIter = ++iter;
    if(nextIter == program->end()){
        parentItem->addChild(statementItem);
        added = true;
    } else {
        if(auto nextItem = statementItemFromStatement(*nextIter)){
            parentItem->insertChild(parentItem->indexOfChild(nextItem), statementItem);
            added = true;
        }
    }

    if(added){
        if(auto structured = dynamic_cast<StructuredStatement*>(statement.get())){
            if(auto lowerLevelProgram = structured->getLowerLevelProgram()){
                addProgramStatementsToTree(lowerLevelProgram, statementItem);
            }
        }
    }
}


void ManipulatorProgramViewBase::Impl::onStatementRemoved
(ManipulatorStatement* statement, StructuredStatement* parentStatement)
{
    auto counter = scopedCounterOfStatementItemOperationCall();
    auto iter = statementItemMap.find(statement);
    if(iter != statementItemMap.end()){
        auto& item = iter->second;
        takeTopLevelItem(indexOfTopLevelItem(item));
        delete item;
    }
}


void ManipulatorProgramViewBase::Impl::forEachStatementInTreeEditEvent
(const QModelIndex& parent, int start, int end,
 function<bool(ManipulatorProgram* program, int index, ManipulatorStatement* statement)> func)
{
    if(!programItem || isDoingStatementItemOperation()){
        return;
    }
    if(auto parentItem = itemFromIndex(parent)){
        return;
    }
    auto root = invisibleRootItem();
    auto program = programItem->program();
    bool updated = false;
    for(int i = start; i <= end; ++i){
        auto item = static_cast<StatementItem*>(root->child(i));
        if(func(program, i, item->statement)){
            updated = true;
        }
    }
    if(updated){
        programItem->suggestFileUpdate();
    }
}
    

void ManipulatorProgramViewBase::Impl::onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    forEachStatementInTreeEditEvent(
        parent, start, end,
        [&](ManipulatorProgram* program, int, ManipulatorStatement* statement){
            return program->remove(statement);
        });
}


void ManipulatorProgramViewBase::Impl::onRowsInserted(const QModelIndex& parent, int start, int end)
{
    forEachStatementInTreeEditEvent(
        parent, start, end,
        [&](ManipulatorProgram* program, int index, ManipulatorStatement* statement){
            program->insert(program->begin() + index, statement);
            return true;
        });
}


void ManipulatorProgramViewBase::Impl::mousePressEvent(QMouseEvent* event)
{
    TreeWidget::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        auto item = dynamic_cast<StatementItem*>(itemAt(event->pos()));
        if(item){
            showContextMenu(item, event->globalPos());
        }
    }
}


void ManipulatorProgramViewBase::Impl::showContextMenu(StatementItem* statementItem, QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);

    auto statement = statementItem->statement;

    contextMenuManager.addItem(_("Cut"))
        ->sigTriggered().connect([=](){ cutStatement(statement); });

    contextMenuManager.addItem(_("Copy"))
        ->sigTriggered().connect([=](){ copyStatement(statement); });

    contextMenuManager.addItem(_("Paste"))
        ->sigTriggered().connect([=](){ pasteStatement(statement); });
    
    contextMenuManager.popupMenu()->popup(globalPos);
}


void ManipulatorProgramViewBase::Impl::cutStatement(ManipulatorStatement* statement)
{
    if(programItem){
        auto program = programItem->program();
        StructuredStatement* parentStatement = nullptr;
        if(auto item = statementItemFromStatement(statement)){
            parentStatement = item->parentStatement();
            if(parentStatement){
                program = parentStatement->getLowerLevelProgram();
            }
        }
        if(program){
            program->remove(statement);
            programItem->sigStatementRemoved()(statement, parentStatement);
            programItem->suggestFileUpdate();
        }
    }
}


void ManipulatorProgramViewBase::Impl::copyStatement(ManipulatorStatement* statementItem)
{

}


void ManipulatorProgramViewBase::Impl::pasteStatement(ManipulatorStatement* statementItem)
{

}


bool ManipulatorProgramViewBase::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "currentProgram");
    return true;
}


bool ManipulatorProgramViewBase::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "currentProgram");
    return true;
}
