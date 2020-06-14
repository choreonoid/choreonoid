#include "BodyElementTreeWidget.h"
#include "BodyItem.h"
#include <cnoid/Link>
#include <cnoid/LinkGroup>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/MenuManager>
#include <cnoid/ItemManager>
#include <QHeaderView>
#include <QBoxLayout>
#include <QEvent>
#include <QApplication>
#include <set>
#include <map>
#include <iostream>
#include <cassert>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
const bool TRACE_FUNCTIONS = false;
}

namespace cnoid {

class BodyElementTreeWidget::Impl
{
public:
    Impl(BodyElementTreeWidget* self);
    ~Impl();

    BodyElementTreeWidget* self;

    struct ColumnInfo {
        ColumnDataFunction dataFunction;
        ColumnSetDataFunction setDataFunction;
        ColumnWidgetFunction widgetFunction;
    };
    vector<ColumnInfo> columnInfos;

    QTreeWidgetItem* headerItem;
    int nameColumn;
    int idColumn;
    int rowIndexCounter;
    int itemWidgetWidthAdjustment;
    vector<BodyElementTreeItem*> customRows;
    bool isNameColumnMarginEnabled;
    int listingMode;
    Menu popupMenu; // This must be defined before popupMenuManager
    MenuManager popupMenuManager;

    Signal<void(bool isInitialCreation)> sigUpdateRequest;
    Signal<void(BodyElementTreeItem* item, int column)> sigItemChanged;
    Signal<void(int linkIndex)> sigCurrentLinkChanged;
    Signal<void()> sigLinkSelectionChanged;

    int defaultExpansionLevel;
    bool isCacheEnabled;

    class BodyItemInfo {
    public:
        bool isRestoringTreeStateNeeded;

        LinkGroupPtr linkGroup;

        vector<bool> selection;
        vector<int> selectedLinkIndices;
        Signal<void()> sigSelectionChanged;

        bool needTreeExpansionUpdate;
        vector<bool> linkExpansions;
        set<string> expandedParts;
                
        Connection detachedFromRootConnection;

        BodyItemInfo() {
            isRestoringTreeStateNeeded = true;
        }
        ~BodyItemInfo() {
            detachedFromRootConnection.disconnect();
        }
        void setNumLinks(int n, bool doInitialize) {
            if(doInitialize){
                selection.clear();
                linkExpansions.clear();
                expandedParts.clear();
            }
            selection.resize(n, false);
            linkExpansions.resize(n, true);
        }
    };
    typedef shared_ptr<BodyItemInfo> BodyItemInfoPtr;

    typedef map<BodyItemPtr, BodyItemInfoPtr> BodyItemInfoMap;
    BodyItemInfoMap bodyItemInfoCache;

    vector<BodyElementTreeItem*> linkIndexToItemMap;

    BodyItemPtr currentBodyItem;
    BodyItemInfoPtr currentBodyItemInfo;

    Signal<void()> dummySigSelectionChanged; // never emitted
    vector<int> emptyLinkIndices;
    vector<bool> emptySelection;

    void initialize();
    void enableCache(bool on);
    BodyItemInfoPtr getBodyItemInfo(BodyItem* bodyItem);
    void onBodyItemDisconnectedFromRoot(BodyItem* bodyItem);
    void clearTreeItems();
    void setCurrentBodyItem(BodyItem* bodyItem, bool forceTreeUpdate);
    void restoreTreeState();
    void restoreSubTreeState(QTreeWidgetItem* item);
    void restoreTreeStateSub(QTreeWidgetItem* parentItem);
    void addChild(BodyElementTreeItem* parentItem, BodyElementTreeItem* item);
    void addChild(BodyElementTreeItem* item);
    void setLinkTree(Link* link, bool onlyJoints);
    void setLinkTreeSub(Link* link, Link* parentLink, BodyElementTreeItem* parentItem, bool onlyJoints);
    void setLinkList(Body* body);
    void setJointList(Body* body);
    void setDeviceList(Body* body);
    void setLinkGroupTree(Body* body, LinkGroupPtr linkGroup);
    void setLinkGroupTreeSub(BodyElementTreeItem* parentItem, LinkGroupPtr linkGroup, Body* body);
    void addCustomRows();
    void onListingModeChanged(int index);
    void onSelectionChanged();
    void onItemChanged(QTreeWidgetItem* item, int column);
    Signal<void()>& sigSelectionChangedOf(BodyItem* bodyItem);
    int selectedLinkIndex(BodyItem* bodyItem) const;
    const vector<int>& selectedLinkIndices(BodyItem* bodyItem);
    const vector<bool>& linkSelection(BodyItem* bodyItem);
    void onCustomContextMenuRequested(const QPoint& pos);
    void setExpansionState(const BodyElementTreeItem* item, bool on);
    void onItemExpanded(QTreeWidgetItem* treeWidgetItem);
    void onItemCollapsed(QTreeWidgetItem* treeWidgetItem);
    bool makeSingleSelection(BodyItem* bodyItem, int linkIndex);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}

namespace {

QVariant idData(const BodyElementTreeItem* item, int role)
{
    const Link* link = item->link();
    if(role == Qt::DisplayRole && link){
        if(item->listingMode() == BodyElementTreeWidget::JointList){
            if(link->jointId() >= 0){
                return link->jointId();
            } else {
                return "";
            }
        } else {
            return link->index();
        }
    } else if(role == Qt::TextAlignmentRole){
        return Qt::AlignHCenter;
    }
    return QVariant();
}

QVariant nameData(const BodyElementTreeItem* item, int role)
{
    if(role == Qt::DisplayRole){
        return item->nameText();
    }
    return QVariant();
}

}


BodyElementTreeItem::BodyElementTreeItem
(const std::string& name, BodyElementTreeWidget::Impl* treeImpl)
    : rowIndex_(-1),
      name_(name),
      link_(nullptr),
      device_(nullptr),
      treeImpl(treeImpl),
      isLinkGroup_(false)
{
    if(treeImpl->isNameColumnMarginEnabled){
        nameText_ = QString(" %1 ").arg(name_.c_str());
    } else {
        nameText_ = name_.c_str();
    }
}


BodyElementTreeItem::BodyElementTreeItem(Link* link, BodyElementTreeWidget::Impl* treeImpl)
    : BodyElementTreeItem(link->name(), treeImpl)
{
    link_ = link;
    treeImpl->linkIndexToItemMap[link->index()] = this;
}


BodyElementTreeItem::BodyElementTreeItem(Device* device, BodyElementTreeWidget::Impl* treeImpl)
    : BodyElementTreeItem(device->name().empty() ? device->typeName() : device->name(), treeImpl)
{
    device_ = device;
}


BodyElementTreeItem::BodyElementTreeItem(LinkGroup* linkGroup, BodyElementTreeWidget::Impl* treeImpl)
    : BodyElementTreeItem(linkGroup->name(), treeImpl)
{
    if(treeImpl->isNameColumnMarginEnabled){
        nameText_ = QString(" %1 ").arg(name_.c_str());
    } else {
        nameText_ = name_.c_str();
    }
    isLinkGroup_ = true;
}


QVariant BodyElementTreeItem::data(int column, int role) const
{
    QVariant value;
    auto tree = static_cast<BodyElementTreeWidget*>(treeWidget());
    if(auto& func = tree->impl->columnInfos[column].dataFunction){
        value = func(this, role);
    }
    if(value.isValid()){
        return value;
    }
    return QTreeWidgetItem::data(column, role);
}


void BodyElementTreeItem::setData(int column, int role, const QVariant& value)
{
    auto tree = static_cast<BodyElementTreeWidget*>(treeWidget());
    auto& func = tree->impl->columnInfos[column].setDataFunction;
    if(func){
        func(this, role, value);
    }
    return QTreeWidgetItem::setData(column, role, value);
}


int BodyElementTreeItem::listingMode() const
{
    return treeImpl->listingMode;
}


BodyElementTreeWidget::BodyElementTreeWidget(QWidget* parent)
    : TreeWidget(parent)
{
    impl = new Impl(this);
    impl->initialize();
}


BodyElementTreeWidget::Impl::Impl(BodyElementTreeWidget* self)
    : self(self),
      popupMenuManager(&popupMenu)
{

}


void BodyElementTreeWidget::Impl::initialize()
{
    rowIndexCounter = 0;
    defaultExpansionLevel = std::numeric_limits<int>::max();
    isCacheEnabled = false;
    itemWidgetWidthAdjustment = 0;
    isNameColumnMarginEnabled = false;
    
    headerItem = new QTreeWidgetItem;
    auto header = self->header();
    header->setMinimumSectionSize(0);
    header->setStretchLastSection(false);

    self->sigSectionResized().connect(
        [&](int, int, int){ self->updateGeometry(); });

    self->setHeaderItem(headerItem);
    self->setSelectionMode(QAbstractItemView::ExtendedSelection);
    self->setIndentation(12);
    self->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    self->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    self->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    nameColumn = self->addColumn(_("Link"));
    header->setSectionResizeMode(nameColumn, QHeaderView::Stretch);
    self->setColumnDataFunction(nameColumn, &nameData);

    idColumn = self->addColumn(_(" No "));
    self->setColumnDataFunction(idColumn, &idData);
    header->setSectionResizeMode(idColumn, QHeaderView::ResizeToContents);
    
    headerItem->setTextAlignment(idColumn, Qt::AlignCenter);
    self->moveVisualColumnIndex(idColumn, 0);

    QObject::connect(self, &QTreeWidget::itemChanged,
                     [&](QTreeWidgetItem* item, int column){ onItemChanged(item, column); });

    QObject::connect(self, &QTreeWidget::itemExpanded,
                     [&](QTreeWidgetItem* item){ onItemExpanded(item); });

    QObject::connect(self, &QTreeWidget::itemCollapsed,
                     [&](QTreeWidgetItem* item){ onItemCollapsed(item); });

    QObject::connect(self, &QTreeWidget::itemSelectionChanged,
                     [&](){ onSelectionChanged(); });

    listingMode = LinkList;
}


BodyElementTreeWidget::~BodyElementTreeWidget()
{
    delete impl;
}


BodyElementTreeWidget::Impl::~Impl()
{
    for(size_t i=0; i < customRows.size(); ++i){
        delete customRows[i];
    }
}


void BodyElementTreeWidget::setDefaultExpansionLevel(int level)
{
    impl->defaultExpansionLevel = level;
}


void BodyElementTreeWidget::enableCache(bool on)
{
    impl->enableCache(on);
}


void BodyElementTreeWidget::Impl::enableCache(bool on)
{
    isCacheEnabled = on;

    if(!isCacheEnabled){
        bodyItemInfoCache.clear();
    }
}


void BodyElementTreeWidget::setListingMode(int mode)
{
    if(mode != impl->listingMode){
        impl->listingMode = mode;
        char* idLabel;
        if(mode == JointList){
            idLabel = _(" ID ");
        } else {
            idLabel = _(" No ");
        }
        impl->headerItem->setText(impl->idColumn, idLabel);
        if(impl->currentBodyItem){
            impl->setCurrentBodyItem(impl->currentBodyItem, true);
        }
    }
}


int BodyElementTreeWidget::listingMode() const
{
    return impl->listingMode;
}


int BodyElementTreeWidget::nameColumn() const
{
    return impl->nameColumn;
}


int BodyElementTreeWidget::idColumn() const
{
    return impl->idColumn;
}


int BodyElementTreeWidget::setNumColumns(int n)
{
    setColumnCount(n);
    impl->columnInfos.resize(n);
    return 0;
}


int BodyElementTreeWidget::addColumn()
{
    int column = impl->columnInfos.size();
    impl->columnInfos.push_back(Impl::ColumnInfo());
    setColumnCount(impl->columnInfos.size());
    impl->headerItem->setText(column, QString());
    header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
    return column;
}


int BodyElementTreeWidget::addColumn(const QString& headerText)
{
    int column = addColumn();
    impl->headerItem->setText(column, headerText);
    header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
    return column;
}


void BodyElementTreeWidget::setColumnStretchResizeMode(int column)
{
    header()->setSectionResizeMode(column, QHeaderView::Stretch);
}


void BodyElementTreeWidget::setColumnInteractiveResizeMode(int column)
{
    header()->setSectionResizeMode(column, QHeaderView::Interactive);
}


void BodyElementTreeWidget::setColumnResizeToContentsMode(int column)
{
    header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
}

                    
void BodyElementTreeWidget::moveVisualColumnIndex(int column, int visualIndex)
{
    header()->moveSection(header()->visualIndex(column), visualIndex);
}


void BodyElementTreeWidget::setColumnDataFunction(int column, ColumnDataFunction func)
{
    impl->columnInfos[column].dataFunction = func;
}


void BodyElementTreeWidget::setColumnSetDataFunction(int column, ColumnSetDataFunction func)
{
    impl->columnInfos[column].setDataFunction = func;
}


void BodyElementTreeWidget::setColumnWidgetFunction(int column, ColumnWidgetFunction func)
{
    impl->columnInfos[column].widgetFunction = func;
}


void BodyElementTreeWidget::changeEvent(QEvent* event)
{
    QTreeWidget::changeEvent(event);
    if(event->type() == QEvent::StyleChange){
        //impl->setCurrentBodyItem(impl->currentBodyItem, true);
    }
}


void BodyElementTreeWidget::setNameColumnMarginEnabled(bool on)
{
    impl->isNameColumnMarginEnabled = on;
}


void BodyElementTreeWidget::setAlignedItemWidget
(BodyElementTreeItem* item, int column, QWidget* widget, Qt::Alignment alignment)
{
    auto box = new QHBoxLayout;
    box->setContentsMargins(0, 0, 0, 0);
    if(impl->itemWidgetWidthAdjustment > 0){
        widget->setMinimumWidth(widget->sizeHint().width() + impl->itemWidgetWidthAdjustment);
    }
    box->addWidget(widget, 0, alignment);
    auto base = new QWidget;
    base->setLayout(box);
    setItemWidget(item, column, base);
}


QWidget* BodyElementTreeWidget::alignedItemWidget(BodyElementTreeItem* item, int column)
{
    QWidget* base = itemWidget(item, column);
    if(base){
        QLayout* layout = base->layout();
        if(layout){
            QLayoutItem* layoutItem = layout->itemAt(0);
            if(layoutItem){
                QWidget* widget = layoutItem->widget();
                if(widget){
                    return widget;
                }
            }
        }
    }
    return nullptr;
}


void BodyElementTreeWidget::addCustomRow(BodyElementTreeItem* treeItem)
{
    impl->customRows.push_back(treeItem);
}


int BodyElementTreeWidget::numBodyElementTreeItems()
{
    return impl->rowIndexCounter;
}


SignalProxy<void(bool isInitialCreation)> BodyElementTreeWidget::sigUpdateRequest()
{
    return impl->sigUpdateRequest;
}


BodyElementTreeWidget::Impl::BodyItemInfoPtr BodyElementTreeWidget::Impl::getBodyItemInfo(BodyItem* bodyItem)
{
    BodyItemInfoPtr info;
    bool isInfoForNewBody = false;

    if(bodyItem){
        if(bodyItem == currentBodyItem){
            info = currentBodyItemInfo;
        } else if(isCacheEnabled){
            auto p = bodyItemInfoCache.find(bodyItem);
            if(p != bodyItemInfoCache.end()){
                info = p->second;
            } else if(isCacheEnabled){
                auto originalItem = ItemManager::findOriginalItemForReloadedItem(bodyItem);
                auto q = bodyItemInfoCache.find(dynamic_cast<BodyItem*>(originalItem));
                if(q != bodyItemInfoCache.end()){
                    info = q->second;
                    isInfoForNewBody = true;
                    bodyItemInfoCache.erase(q);
                }
            }
        }

        if(!info && bodyItem->findRootItem()){
            if(!isCacheEnabled){
                bodyItemInfoCache.clear();
            }
            info = std::make_shared<BodyItemInfo>();
            isInfoForNewBody = true;
        }

        if(isInfoForNewBody){
            info->linkGroup = LinkGroup::create(*bodyItem->body());
            info->detachedFromRootConnection = bodyItem->sigDisconnectedFromRoot().connect(
                [this, bodyItem](){ onBodyItemDisconnectedFromRoot(bodyItem); });
            bodyItemInfoCache[bodyItem] = info;
        }

        if(info){
            info->setNumLinks(bodyItem->body()->numLinks(), false);
        }
    }

    return info;
}


void BodyElementTreeWidget::Impl::onBodyItemDisconnectedFromRoot(BodyItem* bodyItem)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyElementTreeWidgetImpl::onBodyItemDisconnectedFromRoot(" << bodyItem->name() << ")" << endl;
    }
    
    if(currentBodyItem == bodyItem){
        setCurrentBodyItem(0, false);
    }

    BodyItemInfoMap::iterator p = bodyItemInfoCache.find(bodyItem);
    if(p != bodyItemInfoCache.end()){
        p->second->detachedFromRootConnection.disconnect();
        bodyItemInfoCache.erase(p);
    }
}

void BodyElementTreeWidget::setBodyItem(BodyItem* bodyItem)
{
    impl->setCurrentBodyItem(bodyItem, false);
}


BodyItem* BodyElementTreeWidget::bodyItem()
{
    return impl->currentBodyItem;
}


BodyElementTreeItem* BodyElementTreeWidget::itemOfLink(int linkIndex)
{
    if(linkIndex < (int)impl->linkIndexToItemMap.size()){
        return impl->linkIndexToItemMap[linkIndex];
    }
    return nullptr;
}


void BodyElementTreeWidget::Impl::clearTreeItems()
{
    // Take custom row items before calling clear() to prevent the items from being deleted.
    for(size_t i=0; i < customRows.size(); ++i){
        BodyElementTreeItem* item = customRows[i];
        if(item->treeWidget()){
            self->takeTopLevelItem(self->indexOfTopLevelItem(item));
        }
    }
    self->clear();
}


void BodyElementTreeWidget::Impl::setCurrentBodyItem(BodyItem* bodyItem, bool forceTreeUpdate)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyElementTreeWidgetImpl::setCurrentBodyItem(" << (bodyItem ? bodyItem->name() : string("0"))
        << ", " << forceTreeUpdate << ")" << endl;
    }

    bool currentChanged = bodyItem != currentBodyItem;
    
    if(forceTreeUpdate || currentChanged){

        self->blockSignals(true);

        clearTreeItems();
        rowIndexCounter = 0;
        linkIndexToItemMap.clear();

        if(QApplication::style()->objectName() == "cleanlooks"){
            itemWidgetWidthAdjustment = 2;
        } else {
            itemWidgetWidthAdjustment = 0;
        }
        
        self->blockSignals(false);

        currentBodyItemInfo = getBodyItemInfo(bodyItem);
        currentBodyItem = bodyItem;

        if(bodyItem){

            auto body = bodyItem->body();
            linkIndexToItemMap.resize(body->numLinks(), 0);

            switch(listingMode){
            case BodyElementTreeWidget::Tree:
                self->setRootIsDecorated(true);
                setLinkTree(body->rootLink(), false);
                headerItem->setText(nameColumn, _(" Link / Device "));
                break;
            case BodyElementTreeWidget::LinkList:
                self->setRootIsDecorated(false);
                setLinkList(body);
                headerItem->setText(nameColumn, _(" Link "));
                break;
            case BodyElementTreeWidget::JointList:
                self->setRootIsDecorated(false);
                setJointList(body);
                headerItem->setText(nameColumn, _(" Joint "));
                break;
            case BodyElementTreeWidget::DeviceList:
                self->setRootIsDecorated(false);
                setDeviceList(body);
                headerItem->setText(nameColumn, _(" Device "));
                break;
            case BodyElementTreeWidget::PartTree:
                self->setRootIsDecorated(true);
                setLinkGroupTree(bodyItem->body(), currentBodyItemInfo->linkGroup);
                headerItem->setText(nameColumn, _(" Part / Link "));
                break;
            default:
                break;
            }

            addCustomRows();

            restoreTreeState();
        }

        sigUpdateRequest(true);
    }
}


void BodyElementTreeWidget::Impl::restoreTreeState()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyElementTreeWidget::Impl::restoreTreeState()" << endl;
    }
    
    if(currentBodyItemInfo){
        restoreSubTreeState(self->invisibleRootItem());
        currentBodyItemInfo->isRestoringTreeStateNeeded = false;
    }
}


void BodyElementTreeWidget::Impl::restoreSubTreeState(QTreeWidgetItem* item)
{
    self->blockSignals(true);
    restoreTreeStateSub(item);
    self->blockSignals(false);
}


void BodyElementTreeWidget::Impl::restoreTreeStateSub(QTreeWidgetItem* parentItem)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyElementTreeWidget::Impl::restoreTreeStateSub()" << endl;
    }

    int n = parentItem->childCount();
    for(int i=0; i < n; ++i){
        BodyElementTreeItem* item = dynamic_cast<BodyElementTreeItem*>(parentItem->child(i));
        if(item){
            const Link* link = item->link();
            if(link){
                item->setSelected(currentBodyItemInfo->selection[link->index()]);
            }
        
            if(item->childCount() > 0){ // Tree
            
                bool expanded = item->isExpanded();
        
                if(listingMode == BodyElementTreeWidget::PartTree){
                    if(!link){
                        const std::set<string>& parts = currentBodyItemInfo->expandedParts;                        
                        expanded = (parts.find(item->name()) != parts.end());
                        item->setExpanded(expanded);
                    }
                } else if(link){
                    expanded = currentBodyItemInfo->linkExpansions[link->index()];
                    item->setExpanded(expanded);
                }
                if(expanded){
                    restoreTreeStateSub(item);
                }
            }
        }
    }
}


void BodyElementTreeWidget::Impl::addChild(BodyElementTreeItem* parentItem, BodyElementTreeItem* item)
{
    if(parentItem){
        parentItem->addChild(item);
    } else {
        self->addTopLevelItem(item);
    }
    item->rowIndex_ = rowIndexCounter++;

    for(size_t col=0; col < columnInfos.size(); ++col){
        BodyElementTreeWidget::ColumnWidgetFunction& func = columnInfos[col].widgetFunction;
        if(func){
            QWidget* widget = func(item);
            if(widget){
                self->setItemWidget(item, col, widget);
            }
        }
    }
}


void BodyElementTreeWidget::Impl::addChild(BodyElementTreeItem* item)
{
    addChild(0, item);
}


void BodyElementTreeWidget::Impl::setLinkTree(Link* link, bool onlyJoints)
{
    self->blockSignals(true);
    setLinkTreeSub(link, 0, 0, onlyJoints);
    self->blockSignals(false);
}


void BodyElementTreeWidget::Impl::setLinkTreeSub
(Link* link, Link* parentLink, BodyElementTreeItem* parentItem, bool onlyJoints)
{
    BodyElementTreeItem* item = nullptr;

    if(onlyJoints && link->jointId() < 0){
        item = parentItem;
    } else {
        item = new BodyElementTreeItem(link, this);
        addChild(parentItem, item);
        item->setExpanded(true);
    }

    if(link->child()){
        for(Link* child = link->child(); child; child = child->sibling()){
            setLinkTreeSub(child, link, item, onlyJoints);
        }
    }
}


void BodyElementTreeWidget::Impl::setLinkList(Body* body)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyElementTreeWidget::Impl::setLinkList(" << body->name() << ")" << endl;
    }
    
    for(int i=0; i < body->numLinks(); ++i){
        addChild(new BodyElementTreeItem(body->link(i), this));
    }
}


void BodyElementTreeWidget::Impl::setJointList(Body* body)
{
    for(auto& joint : body->joints()){
        if(joint->isValid()){
            addChild(new BodyElementTreeItem(joint, this));
        }
    }
}


void BodyElementTreeWidget::Impl::setDeviceList(Body* body)
{
    for(auto& device : body->devices()){
        addChild(new BodyElementTreeItem(device, this));
    }
}


void BodyElementTreeWidget::Impl::setLinkGroupTree(Body* body, LinkGroupPtr linkGroup)
{
    if(linkGroup){
        self->blockSignals(true);
        setLinkGroupTreeSub(0, linkGroup, body);
        self->blockSignals(false);
    }
}


void BodyElementTreeWidget::Impl::setLinkGroupTreeSub
(BodyElementTreeItem* parentItem, LinkGroupPtr linkGroup, Body* body)
{
    BodyElementTreeItem* item = new BodyElementTreeItem(linkGroup.get(), this);
    addChild(parentItem, item);

    int numChildGroups = 0;
    int n = linkGroup->numElements();
    for(int i=0; i < n; ++i){
        if(linkGroup->isSubGroup(i)){
            setLinkGroupTreeSub(item, linkGroup->subGroup(i), body);
            ++numChildGroups;
        } else if(linkGroup->isLinkIndex(i)){
            Link* link = body->link(linkGroup->linkIndex(i));
            if(link){
                addChild(item, new BodyElementTreeItem(link, this));
            }
        }
    }

    item->setExpanded(numChildGroups > 0);
}


void BodyElementTreeWidget::Impl::addCustomRows()
{
    for(size_t i=0; i < customRows.size(); ++i){
        addChild(customRows[i]);
    }
}


void BodyElementTreeWidget::Impl::onSelectionChanged()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyElementTreeWidgetImpl::onSelectionChanged()" << endl;
    }

    if(currentBodyItem){
        auto& selection = currentBodyItemInfo->selection;
        std::fill(selection.begin(), selection.end(), false);
        QList<QTreeWidgetItem*> selected = self->selectedItems();
        for(int i=0; i < selected.size(); ++i){
            BodyElementTreeItem* item = dynamic_cast<BodyElementTreeItem*>(selected[i]);
            if(item && item->link()){
                selection[item->link()->index()] = true;
            }
        }
        currentBodyItemInfo->sigSelectionChanged();
        sigLinkSelectionChanged();
    }
}


SignalProxy<void(BodyElementTreeItem* item, int column)> BodyElementTreeWidget::sigItemChanged()
{
    return impl->sigItemChanged;
}


void BodyElementTreeWidget::Impl::onItemChanged(QTreeWidgetItem* item, int column)
{
    BodyElementTreeItem* linkTreeItem = dynamic_cast<BodyElementTreeItem*>(item);
    if(linkTreeItem){
        sigItemChanged(linkTreeItem, column);
    }
}


SignalProxy<void(int linkIndex)> BodyElementTreeWidget::sigCurrentLinkChanged()
{
    return impl->sigCurrentLinkChanged;
}


int BodyElementTreeWidget::currentLinkIndex() const
{
    return impl->selectedLinkIndex(impl->currentBodyItem);
}


bool BodyElementTreeWidget::setCurrentLink(int linkIndex)
{
    return impl->makeSingleSelection(impl->currentBodyItem, linkIndex);
}


SignalProxy<void()> BodyElementTreeWidget::sigLinkSelectionChanged()
{
    return impl->sigLinkSelectionChanged;
}


const std::vector<int>& BodyElementTreeWidget::selectedLinkIndices() const
{
    return impl->selectedLinkIndices(impl->currentBodyItem);
}


const std::vector<bool>& BodyElementTreeWidget::linkSelection() const
{
    return impl->linkSelection(impl->currentBodyItem);
}


SignalProxy<void()> BodyElementTreeWidget::sigSelectionChanged(BodyItem* bodyItem)
{
    return impl->sigSelectionChangedOf(bodyItem);
}


Signal<void()>& BodyElementTreeWidget::Impl::sigSelectionChangedOf(BodyItem* bodyItem)
{
    BodyItemInfoPtr info = getBodyItemInfo(bodyItem);
    if(info){
        return info->sigSelectionChanged;
    }
    return dummySigSelectionChanged;
}


int BodyElementTreeWidget::selectedLinkIndex(BodyItem* bodyItem) const
{
    return const_cast<Impl*>(impl)->selectedLinkIndex(bodyItem);
}


int BodyElementTreeWidget::Impl::selectedLinkIndex(BodyItem* bodyItem) const
{
    BodyItemInfoPtr info = const_cast<Impl*>(this)->getBodyItemInfo(bodyItem);
    if(info){
        const auto& selection = info->selection;
        for(size_t i=0; i < selection.size(); ++i){
            if(selection[i]){
                return i;
            }
        }
    }
    return -1;
}


const std::vector<int>& BodyElementTreeWidget::selectedLinkIndices(BodyItem* bodyItem)
{
    return impl->selectedLinkIndices(bodyItem);
}


const vector<int>& BodyElementTreeWidget::Impl::selectedLinkIndices(BodyItem* bodyItem)
{
    BodyItemInfoPtr info = getBodyItemInfo(bodyItem);
    if(info){
        info->selectedLinkIndices.clear();
        const auto& selection = info->selection;
        for(size_t i=0; i < selection.size(); ++i){
            if(selection[i]){
                info->selectedLinkIndices.push_back(i);
            }
        }
        return info->selectedLinkIndices;
    }
    return emptyLinkIndices;
}


const std::vector<bool>& BodyElementTreeWidget::linkSelection(BodyItem* bodyItem)
{
    return impl->linkSelection(bodyItem);
}


const vector<bool>& BodyElementTreeWidget::Impl::linkSelection(BodyItem* bodyItem)
{
    BodyItemInfoPtr info = getBodyItemInfo(bodyItem);
    if(info){
        return info->selection;
    }
    return emptySelection;
}


void BodyElementTreeWidget::Impl::onCustomContextMenuRequested(const QPoint& pos)
{
    popupMenu.popup(pos);
}


void BodyElementTreeWidget::Impl::setExpansionState(const BodyElementTreeItem* item, bool on)
{
    if(listingMode == Tree){
        if(item->link()){
            currentBodyItemInfo->linkExpansions[item->link()->index()] = on;
        }

    } else if(listingMode == PartTree){
        if(on){
            currentBodyItemInfo->expandedParts.insert(item->name());
        } else {
            currentBodyItemInfo->expandedParts.erase(item->name());
        }
    }
}


void BodyElementTreeWidget::Impl::onItemExpanded(QTreeWidgetItem* treeWidgetItem)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyElementTreeWidget::Impl::onItemExpanded()" << endl;
    }
    BodyElementTreeItem* item = dynamic_cast<BodyElementTreeItem*>(treeWidgetItem);
    if(item){
        setExpansionState(item, true);
        restoreSubTreeState(item);
    }
}


void BodyElementTreeWidget::Impl::onItemCollapsed(QTreeWidgetItem* treeWidgetItem)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyElementTreeWidget::Impl::onItemCollapsed()" << endl;
    }
    BodyElementTreeItem* item = dynamic_cast<BodyElementTreeItem*>(treeWidgetItem);
    if(item){
        setExpansionState(item, false);
    }
}


bool BodyElementTreeWidget::Impl::makeSingleSelection(BodyItem* bodyItem, int linkIndex)
{
    BodyItemInfoPtr info = getBodyItemInfo(bodyItem);

    if(!info){
        return false;
    }

    auto& selection = info->selection;
    if(static_cast<size_t>(linkIndex) < selection.size()){
        if(!selection[linkIndex] || std::count(selection.begin(), selection.end(), true) > 1){
            std::fill(selection.begin(), selection.end(), false);
            selection[linkIndex] = true;
            
            if(bodyItem == currentBodyItem){
                restoreTreeState();

                BodyElementTreeItem* item = linkIndexToItemMap[linkIndex];
                if(item){
                    self->scrollToItem(item);
                }
                
                currentBodyItemInfo->sigSelectionChanged();
                sigLinkSelectionChanged();
            } else {
                info->sigSelectionChanged();
            }
        }
    }
    return true;
}


MenuManager& BodyElementTreeWidget::popupMenuManager()
{
    return impl->popupMenuManager;
}


bool BodyElementTreeWidget::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool BodyElementTreeWidget::Impl::storeState(Archive& archive)
{
    if(!isCacheEnabled || !bodyItemInfoCache.empty()){
        return true;
    }

    ListingPtr bodyItemNodes = new Listing;

    for(auto& kv : bodyItemInfoCache){

        BodyItem* bodyItem = kv.first;
        ValueNodePtr id = archive.getItemId(bodyItem);
        if(!id){ // The item is not in the item tree
            continue;
        }
        BodyItemInfo& info = *kv.second;
        MappingPtr bodyItemNode = new Mapping;
        bool isEmpty = true;

        bodyItemNode->insert("id", id);
            
        const auto& indices = selectedLinkIndices(bodyItem);
        if(!indices.empty()){
            Listing& selected = *bodyItemNode->createFlowStyleListing("selected_links");
            for(auto& index : indices){
                selected.append(index, 20, indices.size());
            }
            isEmpty = false;
        }

        auto exps = info.linkExpansions;
        int n = exps.size();
        int m = n - std::count(exps.begin(), exps.end(), true);
        if(m > 0){
            Listing& nonExpanded = *bodyItemNode->createFlowStyleListing("unexpanded_links");
            for(int i=0; i < n; ++i){
                if(!exps[i]){
                    nonExpanded.append(i, 20, m);
                }
            }
            isEmpty = false;
        }

        n = info.expandedParts.size();
        if(n > 0){
            Listing& expanded = *bodyItemNode->createFlowStyleListing("expanded_parts");
            for(auto& part : info.expandedParts){
                expanded.append(part, 10, n, DOUBLE_QUOTED);
            }
            isEmpty = false;
        }
        
        if(!isEmpty){
            bodyItemNodes->append(bodyItemNode);
        }
    }

    if(!bodyItemNodes->empty()){
        archive.insert("body_items", bodyItemNodes);
    }

    return true;
}


bool BodyElementTreeWidget::restoreState(const Archive& archive)
{
    if(impl->isCacheEnabled){
        archive.addPostProcess([&](){ impl->restoreState(archive); });
    }
    return true;
}


bool BodyElementTreeWidget::Impl::restoreState(const Archive& archive)
{
    const Listing& nodes = *archive.findListing("body_items");
    if(nodes.isValid() && !nodes.empty()){
        for(int i=0; i < nodes.size(); ++i){
            const Mapping& node = *nodes[i].toMapping();
            ValueNode* id = node.find("id");
            if(id->isValid()){
                BodyItem* bodyItem = archive.findItem<BodyItem>(id);
                if(bodyItem){
                    int numLinks = bodyItem->body()->numLinks();
                    BodyItemInfoPtr info = getBodyItemInfo(bodyItem);
                    const Listing& selected = *node.findListing("selected_links");
                    if(selected.isValid()){
                        info->setNumLinks(numLinks, true);
                        for(int i=0; i < selected.size(); ++i){
                            int index = selected[i].toInt();
                            if(index < numLinks){
                                info->selection[index] = true;
                            }
                        }
                    }
                    const Listing& expanded = *node.findListing("expanded_parts");
                    for(int i=0; i < expanded.size(); ++i){
                        info->expandedParts.insert(expanded[i]);
                    }
                }
            }
        }
    }

    //restoreTreeState();
    
    return true;
}
