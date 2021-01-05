#include "LinkDeviceTreeWidget.h"
#include "BodyItem.h"
#include <cnoid/LinkGroup>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/MenuManager>
#include <cnoid/ItemManager>
#include <QHeaderView>
#include <QBoxLayout>
#include <QEvent>
#include <QApplication>
#include <QFontMetrics>
#include <map>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyItemInfo
{
public:
    LinkGroupPtr linkGroup;
    vector<bool> selection;
    vector<int> selectedLinkIndices;
    Signal<void()> sigSelectionChanged;
    vector<bool> linkExpansions;
    set<string> expandedParts;
    bool isSelectedLinkIndicesValid;
    bool isRestoringTreeStateNeeded;
    bool needTreeExpansionUpdate;
    ScopedConnection bodyItemConnection;
    
    BodyItemInfo() {
        isSelectedLinkIndicesValid = false;
        isRestoringTreeStateNeeded = true;
    }
    void setNumLinks(int n, bool doInitialize) {
        if(doInitialize){
            selection.clear();
            linkExpansions.clear();
            expandedParts.clear();
        }
        selection.resize(n, false);
        linkExpansions.resize(n, true);
        isSelectedLinkIndicesValid = false;
    }
};
typedef shared_ptr<BodyItemInfo> BodyItemInfoPtr;

}

namespace cnoid {

class LinkDeviceTreeWidget::Impl
{
public:
    Impl(LinkDeviceTreeWidget* self);
    ~Impl();

    LinkDeviceTreeWidget* self;

    int listingMode;
    int numberColumnMode;
    bool isLinkItemVisible;
    bool isJointItemVisible;
    bool isDeviceItemVisible;
    bool isCacheEnabled;
    std::function<bool(Link* link)> visibleLinkPredicate;

    struct ColumnInfo {
        ColumnDataGetter dataGetter;
        ColumnDataSetter dataSetter;
        ColumnWidgetFunction widgetFunction;
    };
    vector<ColumnInfo> columnInfos;

    QTreeWidgetItem* headerItem;
    int nameColumn;
    int numberColumn;
    int rowIndexCounter;
    int itemWidgetWidthAdjustment;
    vector<LinkDeviceTreeItem*> customRows;
    bool isNameColumnMarginEnabled;
    Menu popupMenu; // This must be defined before popupMenuManager
    MenuManager popupMenuManager;

    Signal<void(bool isInitialCreation)> sigUpdateRequest;
    Signal<void(LinkDeviceTreeItem* item, int column)> sigTreeItemChanged;
    Signal<void()> sigLinkSelectionChanged;

    int defaultExpansionLevel;

    typedef map<BodyItemPtr, BodyItemInfoPtr> BodyItemInfoMap;
    BodyItemInfoMap bodyItemInfoMap;

    vector<LinkDeviceTreeItem*> linkIndexToItemMap;

    BodyItemPtr currentBodyItem;
    BodyItemInfoPtr currentBodyItemInfo;

    vector<int> emptyLinkIndices;

    void initialize();
    void setCacheEnabled(bool on);
    void onNumberSectionClicked();
    void setCurrentBodyItem(BodyItem* bodyItem, bool forceTreeUpdate);
    void updateTreeItems();
    void clearTreeItems();
    BodyItemInfoPtr getOrCreateBodyItemInfo(BodyItem* bodyItem);
    void onBodyItemDisconnectedFromRoot(BodyItem* bodyItem);
    void addTreeItem(QTreeWidgetItem* item, QTreeWidgetItem* parentItem);
    void addLinkDeviceTreeItem(LinkDeviceTreeItem* item, QTreeWidgetItem* parentItem = nullptr);
    void createLinkDeviceList(Body* body);
    void createDeviceList(Body* body);
    void createLinkDeviceTree(Body* body);
    void createLinkDeviceTreeSub(
        Link* link, LinkDeviceTreeItem* parentItem, map<Link*, vector<Device*>>& devices);
    void createGroupedLinkTree(Body* body, LinkGroupPtr linkGroup);
    void createGroupedLinkTreeSub(Body* body, LinkGroupPtr linkGroup, LinkDeviceTreeItem* parentItem);
    void updateCustomRows();
    void restoreTreeState();
    void restoreSubTreeState(QTreeWidgetItem* item);
    void restoreSubTreeStateIter(QTreeWidgetItem* parentItem);
    void onSelectionChanged();
    void onItemChanged(QTreeWidgetItem* item, int column);
    Signal<void()>& sigSelectionChangedOf(BodyItem* bodyItem);
    int selectedLinkIndex(BodyItem* bodyItem);
    const vector<int>& selectedLinkIndices(BodyItem* bodyItem);
    const vector<bool>& linkSelection(BodyItem* bodyItem);
    void onCustomContextMenuRequested(const QPoint& pos);
    void setExpansionState(const LinkDeviceTreeItem* item, bool on);
    void onItemExpanded(QTreeWidgetItem* treeWidgetItem);
    void onItemCollapsed(QTreeWidgetItem* treeWidgetItem);
    bool makeSingleSelection(BodyItem* bodyItem, int linkIndex);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}

namespace {

QVariant nameData(const LinkDeviceTreeItem* item, int role)
{
    if(role == Qt::DisplayRole){
        return item->nameText();
    }
    return QVariant();
}

QVariant numberData(const LinkDeviceTreeItem* item, int role)
{
    if(role == Qt::DisplayRole){
        if(auto link = item->link()){
            if(item->numberColumnMode() == LinkDeviceTreeWidget::Index){
                return link->index();
            } else {
                int id = link->jointId();
                if(id >= 0){
                    return id;
                }
            }
        } else if(auto device = item->device()){
            if(item->numberColumnMode() == LinkDeviceTreeWidget::Index){
                return device->index();
            } else {
                int id = device->id();
                if(id >= 0){
                    return id;
                }
            }
        }
    } else if(role == Qt::TextAlignmentRole){
        return Qt::AlignHCenter;
    }
    return QVariant();
}

}


LinkDeviceTreeItem::LinkDeviceTreeItem
(const std::string& name, LinkDeviceTreeWidget::Impl* treeImpl)
    : rowIndex_(-1),
      name_(name),
      link_(nullptr),
      device_(nullptr),
      treeImpl(treeImpl),
      isLinkGroup_(false)
{
    if(treeImpl && treeImpl->isNameColumnMarginEnabled){
        nameText_ = QString(" %1 ").arg(name_.c_str());
    } else {
        nameText_ = name_.c_str();
    }
}


LinkDeviceTreeItem::LinkDeviceTreeItem(Link* link, LinkDeviceTreeWidget::Impl* treeImpl)
    : LinkDeviceTreeItem(
        treeImpl->isJointItemVisible ? link->jointName() : link->name(), treeImpl)
{
    link_ = link;
    treeImpl->linkIndexToItemMap[link->index()] = this;
}


LinkDeviceTreeItem::LinkDeviceTreeItem(Device* device, LinkDeviceTreeWidget::Impl* treeImpl)
    : LinkDeviceTreeItem(device->name().empty() ? device->typeName() : device->name(), treeImpl)
{
    device_ = device;
}


LinkDeviceTreeItem::LinkDeviceTreeItem(LinkGroup* linkGroup, LinkDeviceTreeWidget::Impl* treeImpl)
    : LinkDeviceTreeItem(linkGroup->name(), treeImpl)
{
    if(treeImpl->isNameColumnMarginEnabled){
        nameText_ = QString(" %1 ").arg(name_.c_str());
    } else {
        nameText_ = name_.c_str();
    }
    isLinkGroup_ = true;
}


int LinkDeviceTreeItem::numberColumnMode() const
{
    return treeImpl ? treeImpl->numberColumnMode : LinkDeviceTreeWidget::Index;
}


QVariant LinkDeviceTreeItem::data(int column, int role) const
{
    QVariant value;
    auto tree = static_cast<LinkDeviceTreeWidget*>(treeWidget());
    if(auto& func = tree->impl->columnInfos[column].dataGetter){
        value = func(this, role);
    }
    if(value.isValid()){
        return value;
    }
    return QTreeWidgetItem::data(column, role);
}


void LinkDeviceTreeItem::setData(int column, int role, const QVariant& value)
{
    auto tree = static_cast<LinkDeviceTreeWidget*>(treeWidget());
    auto& func = tree->impl->columnInfos[column].dataSetter;
    if(func){
        func(this, role, value);
    }
    return QTreeWidgetItem::setData(column, role, value);
}


LinkDeviceTreeWidget::LinkDeviceTreeWidget(QWidget* parent)
    : TreeWidget(parent)
{
    impl = new Impl(this);
    impl->initialize();
}


LinkDeviceTreeWidget::Impl::Impl(LinkDeviceTreeWidget* self)
    : self(self),
      popupMenuManager(&popupMenu)
{

}


void LinkDeviceTreeWidget::Impl::initialize()
{
    headerItem = new QTreeWidgetItem;
    auto header = self->header();
    QFontMetrics metrics(self->font());
    header->setMinimumSectionSize(metrics.averageCharWidth() * 4);
    header->setStretchLastSection(false);
    header->setSectionsClickable(true);

    self->sigSectionResized().connect(
        [&](int, int, int){ self->updateGeometry(); });

    self->setHeaderItem(headerItem);
    self->setSelectionMode(QAbstractItemView::ExtendedSelection);
    self->setIndentation(12);
    self->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    self->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    self->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    nameColumn = self->addColumn();
    header->setSectionResizeMode(nameColumn, QHeaderView::Stretch);
    self->setColumnDataGetter(nameColumn, &nameData);

    numberColumn = self->addColumn();
    self->setColumnDataGetter(numberColumn, &numberData);
    header->setSectionResizeMode(numberColumn, QHeaderView::ResizeToContents);
    headerItem->setTextAlignment(numberColumn, Qt::AlignCenter);
    QObject::connect(header, &QHeaderView::sectionClicked,
                     [&](int logicalIndex){
                         if(logicalIndex == 1){ onNumberSectionClicked(); }
                     });

    self->moveVisualColumnIndex(numberColumn, 0);

    QObject::connect(self, &QTreeWidget::itemChanged,
                     [&](QTreeWidgetItem* item, int column){ onItemChanged(item, column); });

    QObject::connect(self, &QTreeWidget::itemExpanded,
                     [&](QTreeWidgetItem* item){ onItemExpanded(item); });

    QObject::connect(self, &QTreeWidget::itemCollapsed,
                     [&](QTreeWidgetItem* item){ onItemCollapsed(item); });

    QObject::connect(self, &QTreeWidget::itemSelectionChanged,
                     [&](){ onSelectionChanged(); });

    listingMode = List;
    self->setNumberColumnMode(Index);
    isLinkItemVisible = true;
    isJointItemVisible = false;
    isDeviceItemVisible = false;
    isCacheEnabled = false;
    
    rowIndexCounter = 0;
    itemWidgetWidthAdjustment = 0;
    isNameColumnMarginEnabled = true;
    defaultExpansionLevel = std::numeric_limits<int>::max();
}


LinkDeviceTreeWidget::~LinkDeviceTreeWidget()
{
    delete impl;
}


LinkDeviceTreeWidget::Impl::~Impl()
{
    for(size_t i=0; i < customRows.size(); ++i){
        delete customRows[i];
    }
}


void LinkDeviceTreeWidget::setListingMode(int mode)
{
    impl->listingMode = mode;
}


int LinkDeviceTreeWidget::listingMode() const
{
    return impl->listingMode;
}


void LinkDeviceTreeWidget::setNumberColumnMode(int mode)
{
    impl->numberColumnMode = mode;
    if(mode == Index){
        impl->headerItem->setText(impl->numberColumn, _("No"));
    } else {
        impl->headerItem->setText(impl->numberColumn, _("ID"));
    }
}


int LinkDeviceTreeWidget::numberColumnMode() const
{
    return impl->numberColumnMode;
}


void LinkDeviceTreeWidget::setLinkItemVisible(bool on)
{
    impl->isLinkItemVisible = on;
    setNumberColumnMode(Index);
}


bool LinkDeviceTreeWidget::isLinkItemVisible() const
{
    return impl->isLinkItemVisible;
}


void LinkDeviceTreeWidget::setVisibleLinkPredicate(std::function<bool(Link* link)> pred)
{
    impl->visibleLinkPredicate = pred;
}


void LinkDeviceTreeWidget::setJointItemVisible(bool on)
{
    impl->isJointItemVisible = on;
    setNumberColumnMode(Identifier);
}


bool LinkDeviceTreeWidget::isJointItemVisible() const
{
    return impl->isJointItemVisible;
}


void LinkDeviceTreeWidget::setDeviceItemVisible(bool on)
{
    impl->isDeviceItemVisible = on;
}


bool LinkDeviceTreeWidget::isDeviceItemVisible() const
{
    return impl->isDeviceItemVisible;
}


void LinkDeviceTreeWidget::setDefaultExpansionLevel(int level)
{
    impl->defaultExpansionLevel = level;
}


void LinkDeviceTreeWidget::setCacheEnabled(bool on)
{
    impl->setCacheEnabled(on);
}


void LinkDeviceTreeWidget::Impl::setCacheEnabled(bool on)
{
    isCacheEnabled = on;

    if(!isCacheEnabled){
        bodyItemInfoMap.clear();
    }
}


bool LinkDeviceTreeWidget::isCacheEnabled() const
{
    return impl->isCacheEnabled;
}


int LinkDeviceTreeWidget::nameColumn() const
{
    return impl->nameColumn;
}


int LinkDeviceTreeWidget::numberColumn() const
{
    return impl->numberColumn;
}


int LinkDeviceTreeWidget::setNumColumns(int n)
{
    setColumnCount(n);
    impl->columnInfos.resize(n);
    return 0;
}


int LinkDeviceTreeWidget::addColumn()
{
    return addColumn(QString());
}


int LinkDeviceTreeWidget::addColumn(const QString& headerText)
{
    int column = impl->columnInfos.size();
    impl->columnInfos.push_back(Impl::ColumnInfo());
    setColumnCount(impl->columnInfos.size());
    impl->headerItem->setText(column, headerText);
    header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
    return column;
}


void LinkDeviceTreeWidget::setColumnStretchResizeMode(int column)
{
    header()->setSectionResizeMode(column, QHeaderView::Stretch);
}


void LinkDeviceTreeWidget::setColumnInteractiveResizeMode(int column)
{
    header()->setSectionResizeMode(column, QHeaderView::Interactive);
}


void LinkDeviceTreeWidget::setColumnResizeToContentsMode(int column)
{
    header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
}

                    
void LinkDeviceTreeWidget::setNameColumnMarginEnabled(bool on)
{
    impl->isNameColumnMarginEnabled = on;
}


void LinkDeviceTreeWidget::moveVisualColumnIndex(int column, int visualIndex)
{
    header()->moveSection(header()->visualIndex(column), visualIndex);
}


void LinkDeviceTreeWidget::setColumnDataGetter(int column, ColumnDataGetter func)
{
    impl->columnInfos[column].dataGetter = func;
}


void LinkDeviceTreeWidget::setColumnDataSetter(int column, ColumnDataSetter func)
{
    impl->columnInfos[column].dataSetter = func;
}


void LinkDeviceTreeWidget::setColumnWidgetFunction(int column, ColumnWidgetFunction func)
{
    impl->columnInfos[column].widgetFunction = func;
}


void LinkDeviceTreeWidget::setAlignedItemWidget
(LinkDeviceTreeItem* item, int column, QWidget* widget, Qt::Alignment alignment)
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


QWidget* LinkDeviceTreeWidget::alignedItemWidget(LinkDeviceTreeItem* item, int column)
{
    if(auto base = itemWidget(item, column)){
        if(auto layout = base->layout()){
            if(auto layoutItem = layout->itemAt(0)){
                if(auto widget = layoutItem->widget()){
                    return widget;
                }
            }
        }
    }
    return nullptr;
}


int LinkDeviceTreeWidget::numLinkDeviceTreeItems()
{
    return impl->rowIndexCounter;
}


SignalProxy<void(bool isInitialCreation)> LinkDeviceTreeWidget::sigUpdateRequest()
{
    return impl->sigUpdateRequest;
}


void LinkDeviceTreeWidget::Impl::onNumberSectionClicked()
{
    if(numberColumnMode == Index){
        self->setNumberColumnMode(Identifier);
    } else {
        self->setNumberColumnMode(Index);
    }
    updateTreeItems();
}


void LinkDeviceTreeWidget::setBodyItem(BodyItem* bodyItem, bool forceTreeUpdate)
{
    impl->setCurrentBodyItem(bodyItem, forceTreeUpdate);
}


BodyItem* LinkDeviceTreeWidget::bodyItem()
{
    return impl->currentBodyItem;
}


void LinkDeviceTreeWidget::Impl::setCurrentBodyItem(BodyItem* bodyItem, bool forceTreeUpdate)
{
    if(bodyItem != currentBodyItem || forceTreeUpdate){
        currentBodyItem = bodyItem;
        currentBodyItemInfo.reset();
        updateTreeItems();
    }
}


void LinkDeviceTreeWidget::updateTreeItems()
{
    return impl->updateTreeItems();
}


void LinkDeviceTreeWidget::Impl::updateTreeItems()
{
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

    if(currentBodyItem){
        currentBodyItemInfo = getOrCreateBodyItemInfo(currentBodyItem);
        
        auto body = currentBodyItem->body();
        linkIndexToItemMap.resize(body->numLinks(), 0);

        if(listingMode == GroupedTree){
            self->setRootIsDecorated(true);
            headerItem->setText(nameColumn, _(" Part / Link "));
            createGroupedLinkTree(body, currentBodyItemInfo->linkGroup);
        } else {
            self->setRootIsDecorated(false);
            if(isLinkItemVisible){
                if(isDeviceItemVisible){
                    headerItem->setText(nameColumn, _(" Link / Device "));
                } else {
                    headerItem->setText(nameColumn, _(" Link "));
                }
                if(listingMode == List){
                    createLinkDeviceList(body);
                }
            } else if(isJointItemVisible){
                headerItem->setText(nameColumn, _(" Joint "));
                if(listingMode == List){
                    createLinkDeviceList(body);
                }

            } else if(isDeviceItemVisible){
                headerItem->setText(nameColumn, _(" Device "));
                if(listingMode == List){
                    createDeviceList(body);
                }
            }
            if(listingMode == Tree){
                if(isLinkItemVisible || isJointItemVisible){
                    createLinkDeviceTree(body);
                }
            }
        }
        
        updateCustomRows();

        if(isCacheEnabled){
            restoreTreeState();
        }
    }
    
    sigUpdateRequest(true);
}    


void LinkDeviceTreeWidget::Impl::clearTreeItems()
{
    // Take custom row items before calling clear() to prevent the items from being deleted.
    for(size_t i=0; i < customRows.size(); ++i){
        auto item = customRows[i];
        if(item->treeWidget()){
            self->takeTopLevelItem(self->indexOfTopLevelItem(item));
        }
    }
    self->clear();
}


BodyItemInfoPtr LinkDeviceTreeWidget::Impl::getOrCreateBodyItemInfo(BodyItem* bodyItem)
{
    BodyItemInfoPtr info;
    bool isNewBodyInfo = false;

    if(bodyItem){
        if(isCacheEnabled){
            auto p = bodyItemInfoMap.find(bodyItem);
            if(p != bodyItemInfoMap.end()){
                info = p->second;
            } else {
                if(auto originalItem = bodyItem->findOriginalItem()){
                    auto q = bodyItemInfoMap.find(dynamic_cast<BodyItem*>(originalItem));
                    if(q != bodyItemInfoMap.end()){
                        info = q->second;
                        isNewBodyInfo = true;
                        bodyItemInfoMap.erase(q);
                    }
                }
            }
        }
        if(!info && bodyItem->findRootItem()){
            info = std::make_shared<BodyItemInfo>();
            isNewBodyInfo = true;
        }
        if(isNewBodyInfo){
            info->bodyItemConnection = bodyItem->sigDisconnectedFromRoot().connect(
                [this, bodyItem](){ onBodyItemDisconnectedFromRoot(bodyItem); });
            if(isCacheEnabled){
                bodyItemInfoMap[bodyItem] = info;
            }
        }
        if(info){
            info->setNumLinks(bodyItem->body()->numLinks(), false);

            if(listingMode == GroupedTree && !info->linkGroup){
                info->linkGroup = LinkGroup::create(*bodyItem->body());
            }
        }
    }

    return info;
}


LinkDeviceTreeItem* LinkDeviceTreeWidget::itemOfLink(int linkIndex)
{
    if(linkIndex < (int)impl->linkIndexToItemMap.size()){
        return impl->linkIndexToItemMap[linkIndex];
    }
    return nullptr;
}


void LinkDeviceTreeWidget::Impl::onBodyItemDisconnectedFromRoot(BodyItem* bodyItem)
{
    if(currentBodyItem == bodyItem){
        setCurrentBodyItem(nullptr, false);
    }
    if(isCacheEnabled){
        bodyItemInfoMap.erase(bodyItem);
    }
}


void LinkDeviceTreeWidget::Impl::addTreeItem(QTreeWidgetItem* item, QTreeWidgetItem* parentItem)
{
    parentItem->addChild(item);
    rowIndexCounter++;
}


void LinkDeviceTreeWidget::Impl::addLinkDeviceTreeItem(LinkDeviceTreeItem* item, QTreeWidgetItem* parentItem)
{
    if(parentItem){
        parentItem->addChild(item);
    } else {
        self->addTopLevelItem(item);
    }
    item->rowIndex_ = rowIndexCounter++;

    for(size_t col=0; col < columnInfos.size(); ++col){
        LinkDeviceTreeWidget::ColumnWidgetFunction& func = columnInfos[col].widgetFunction;
        if(func){
            QWidget* widget = func(item);
            if(widget){
                self->setItemWidget(item, col, widget);
            }
        }
    }
}


void LinkDeviceTreeWidget::Impl::createLinkDeviceList(Body* body)
{
    map<const Link*, vector<Device*>> devices;
    if(isDeviceItemVisible){
        for(auto& device : body->devices()){
            devices[device->link()].push_back(device);
        }
    }
    auto links = body->links();
    if(numberColumnMode == Identifier || isJointItemVisible){
        std::stable_sort(links.begin(), links.end(),
                  [&](Link* l0, Link* l1){
                      if(l0->jointId() < 0 || l1->jointId() < 0){
                          return false;
                      }
                      return l0->jointId() < l1->jointId();
                  });
    }
        
    for(auto& link : links){
        if(visibleLinkPredicate && !visibleLinkPredicate(link)){
            continue;
        }
        if(isJointItemVisible && link->jointId() < 0){
            continue;
        }
        auto linkItem = new LinkDeviceTreeItem(link, this);
        addLinkDeviceTreeItem(linkItem);

        auto it = devices.find(linkItem->link());
        if(it != devices.end()){
            for(auto& device : it->second){
                auto deviceItem = new LinkDeviceTreeItem(device, this);
                addLinkDeviceTreeItem(deviceItem, linkItem);
            }
        }
    }

    if(!devices.empty()){
        self->setRootIsDecorated(true);
    }
}


void LinkDeviceTreeWidget::Impl::createDeviceList(Body* body)
{
    for(auto& device : body->devices()){
        addLinkDeviceTreeItem(new LinkDeviceTreeItem(device, this));
    }
}


void LinkDeviceTreeWidget::Impl::createLinkDeviceTree(Body* body)
{
    self->blockSignals(true);

    map<Link*, vector<Device*>> devices;
    if(isDeviceItemVisible){
        for(auto& device : body->devices()){
            devices[device->link()].push_back(device);
        }
    }
    
    self->setRootIsDecorated(false);
    createLinkDeviceTreeSub(body->rootLink(), nullptr, devices);
    
    self->blockSignals(false);
}


void LinkDeviceTreeWidget::Impl::createLinkDeviceTreeSub
(Link* link, LinkDeviceTreeItem* parentItem, map<Link*, vector<Device*>>& devices)
{
    if(visibleLinkPredicate && !visibleLinkPredicate(link)){
        return;
    }

    LinkDeviceTreeItem* item = nullptr;
    
    if(!isJointItemVisible || link->jointId() >= 0){

        item = new LinkDeviceTreeItem(link, this);
        addLinkDeviceTreeItem(item, parentItem);
        item->setExpanded(true);

        if(isDeviceItemVisible){
            auto it = devices.find(link);
            if(it != devices.end()){
                auto deviceGroup = new QTreeWidgetItem({ _("Device") });
                addTreeItem(deviceGroup, item);
                deviceGroup->setExpanded(true);
                for(auto& device : it->second){
                    auto deviceItem = new LinkDeviceTreeItem(device, this);
                    addLinkDeviceTreeItem(deviceItem, deviceGroup);
                }
            }
        }
    }
    
    if(link->child()){
        if(!item){
            item = parentItem;
        }
        for(Link* child = link->child(); child; child = child->sibling()){
            createLinkDeviceTreeSub(child, item, devices);
        }
    }
}


void LinkDeviceTreeWidget::Impl::createGroupedLinkTree(Body* body, LinkGroupPtr linkGroup)
{
    if(linkGroup){
        self->blockSignals(true);
        createGroupedLinkTreeSub(body, linkGroup, nullptr);
        self->blockSignals(false);
    }
}


void LinkDeviceTreeWidget::Impl::createGroupedLinkTreeSub
(Body* body, LinkGroupPtr linkGroup, LinkDeviceTreeItem* parentItem)
{
    auto item = new LinkDeviceTreeItem(linkGroup.get(), this);
    addLinkDeviceTreeItem(item, parentItem);

    int numChildGroups = 0;
    int n = linkGroup->numElements();
    for(int i=0; i < n; ++i){
        if(linkGroup->isSubGroup(i)){
            createGroupedLinkTreeSub(body, linkGroup->subGroup(i), item);
            ++numChildGroups;
        } else if(linkGroup->isLinkIndex(i)){
            auto link = body->link(linkGroup->linkIndex(i));
            if(link){
                addLinkDeviceTreeItem(new LinkDeviceTreeItem(link, this), item);
            }
        }
    }

    item->setExpanded(numChildGroups > 0);
}


void LinkDeviceTreeWidget::addCustomRow(LinkDeviceTreeItem* treeItem)
{
    impl->customRows.push_back(treeItem);
}


void LinkDeviceTreeWidget::Impl::updateCustomRows()
{
    for(size_t i=0; i < customRows.size(); ++i){
        addLinkDeviceTreeItem(customRows[i]);
    }
}


void LinkDeviceTreeWidget::Impl::restoreTreeState()
{
    if(currentBodyItemInfo){
        restoreSubTreeState(self->invisibleRootItem());
        currentBodyItemInfo->isRestoringTreeStateNeeded = false;
    }
}


void LinkDeviceTreeWidget::Impl::restoreSubTreeState(QTreeWidgetItem* item)
{
    self->blockSignals(true);
    restoreSubTreeStateIter(item);
    self->blockSignals(false);
}


void LinkDeviceTreeWidget::Impl::restoreSubTreeStateIter(QTreeWidgetItem* parentItem)
{
    int n = parentItem->childCount();
    for(int i=0; i < n; ++i){
        if(auto item = dynamic_cast<LinkDeviceTreeItem*>(parentItem->child(i))){
            auto link = item->link();
            if(link){
                item->setSelected(currentBodyItemInfo->selection[link->index()]);
            }
            if(item->childCount() > 0){ // Tree
                bool expanded = item->isExpanded();
                if(listingMode == GroupedTree){
                    if(!link){
                        auto& parts = currentBodyItemInfo->expandedParts;                        
                        expanded = (parts.find(item->name()) != parts.end());
                        item->setExpanded(expanded);
                    }
                } else if(link){
                    expanded = currentBodyItemInfo->linkExpansions[link->index()];
                    item->setExpanded(expanded);
                }
                if(expanded){
                    restoreSubTreeStateIter(item);
                }
            }
        }
    }
}


void LinkDeviceTreeWidget::Impl::onSelectionChanged()
{
    if(currentBodyItemInfo){
        auto& selection = currentBodyItemInfo->selection;
        std::fill(selection.begin(), selection.end(), false);
        QList<QTreeWidgetItem*> selected = self->selectedItems();
        for(int i=0; i < selected.size(); ++i){
            auto item = dynamic_cast<LinkDeviceTreeItem*>(selected[i]);
            if(item && item->link()){
                selection[item->link()->index()] = true;
            }
        }
        currentBodyItemInfo->isSelectedLinkIndicesValid = false;
        if(isCacheEnabled){
            currentBodyItemInfo->sigSelectionChanged();
        }
        sigLinkSelectionChanged();
    }
}


SignalProxy<void(LinkDeviceTreeItem* item, int column)> LinkDeviceTreeWidget::sigTreeItemChanged()
{
    return impl->sigTreeItemChanged;
}


void LinkDeviceTreeWidget::Impl::onItemChanged(QTreeWidgetItem* item, int column)
{
    LinkDeviceTreeItem* linkTreeItem = dynamic_cast<LinkDeviceTreeItem*>(item);
    if(linkTreeItem){
        sigTreeItemChanged(linkTreeItem, column);
    }
}


SignalProxy<void()> LinkDeviceTreeWidget::sigLinkSelectionChanged()
{
    return impl->sigLinkSelectionChanged;
}


const std::vector<bool>& LinkDeviceTreeWidget::linkSelection() const
{
    return impl->linkSelection(impl->currentBodyItem);
}


const std::vector<int>& LinkDeviceTreeWidget::selectedLinkIndices() const
{
    return impl->selectedLinkIndices(impl->currentBodyItem);
}


SignalProxy<void()> LinkDeviceTreeWidget::sigSelectionChanged(BodyItem* bodyItem)
{
    return impl->sigSelectionChangedOf(bodyItem);
}


Signal<void()>& LinkDeviceTreeWidget::Impl::sigSelectionChangedOf(BodyItem* bodyItem)
{
    if(isCacheEnabled && bodyItem){
        return getOrCreateBodyItemInfo(bodyItem)->sigSelectionChanged;
    } else {
        static Signal<void()> dummySigSelectionChanged;
        return dummySigSelectionChanged;
    }
}


const std::vector<bool>& LinkDeviceTreeWidget::linkSelection(BodyItem* bodyItem)
{
    return impl->linkSelection(bodyItem);
}


const vector<bool>& LinkDeviceTreeWidget::Impl::linkSelection(BodyItem* bodyItem)
{
    if(auto info = getOrCreateBodyItemInfo(bodyItem)){
        return info->selection;
    } else {
        static vector<bool> emptySelection;
        return emptySelection;
    }
}


const std::vector<int>& LinkDeviceTreeWidget::selectedLinkIndices(BodyItem* bodyItem)
{
    return impl->selectedLinkIndices(bodyItem);
}


const vector<int>& LinkDeviceTreeWidget::Impl::selectedLinkIndices(BodyItem* bodyItem)
{
    if(auto info = getOrCreateBodyItemInfo(bodyItem)){
        if(info->isSelectedLinkIndicesValid){
            return info->selectedLinkIndices;
        }
        auto& selected = info->selectedLinkIndices;
        selected.clear();
        const auto& selection = info->selection;
        for(size_t i=0; i < selection.size(); ++i){
            if(selection[i]){
                selected.push_back(i);
            }
        }
        info->isSelectedLinkIndicesValid = true;
        return selected;
    }
    return emptyLinkIndices;
}


void LinkDeviceTreeWidget::setLinkSelection(BodyItem* bodyItem, const std::vector<bool>& selection)
{
    if(auto info = impl->getOrCreateBodyItemInfo(bodyItem)){
        info->selection = selection;
        info->selection.resize(bodyItem->body()->numLinks(), false);
        info->isSelectedLinkIndicesValid = false;
        if(bodyItem == impl->currentBodyItem){
            impl->restoreTreeState();
        }
    }
}


void LinkDeviceTreeWidget::Impl::onCustomContextMenuRequested(const QPoint& pos)
{
    popupMenu.popup(pos);
}


void LinkDeviceTreeWidget::Impl::setExpansionState(const LinkDeviceTreeItem* item, bool on)
{
    if(listingMode == Tree){
        if(item->link()){
            currentBodyItemInfo->linkExpansions[item->link()->index()] = on;
        }

    } else if(listingMode == GroupedTree){
        if(on){
            currentBodyItemInfo->expandedParts.insert(item->name());
        } else {
            currentBodyItemInfo->expandedParts.erase(item->name());
        }
    }
}


void LinkDeviceTreeWidget::Impl::onItemExpanded(QTreeWidgetItem* treeWidgetItem)
{
    if(auto item = dynamic_cast<LinkDeviceTreeItem*>(treeWidgetItem)){
        setExpansionState(item, true);
        restoreSubTreeState(item);
    }
}


void LinkDeviceTreeWidget::Impl::onItemCollapsed(QTreeWidgetItem* treeWidgetItem)
{
    if(auto item = dynamic_cast<LinkDeviceTreeItem*>(treeWidgetItem)){
        setExpansionState(item, false);
    }
}


MenuManager& LinkDeviceTreeWidget::popupMenuManager()
{
    return impl->popupMenuManager;
}


void LinkDeviceTreeWidget::changeEvent(QEvent* event)
{
    QTreeWidget::changeEvent(event);
    if(event->type() == QEvent::StyleChange){
        //impl->setCurrentBodyItem(impl->currentBodyItem, true);
    }
}


bool LinkDeviceTreeWidget::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool LinkDeviceTreeWidget::Impl::storeState(Archive& archive)
{
    if(!isCacheEnabled || bodyItemInfoMap.empty()){
        return true;
    }

    ListingPtr bodyItemNodes = new Listing;

    for(auto& kv : bodyItemInfoMap){

        BodyItem* bodyItem = kv.first;
        ValueNodePtr id = archive.getItemId(bodyItem);
        if(!id){ // The item is not in the item tree
            continue;
        }
        BodyItemInfo& info = *kv.second;
        MappingPtr bodyItemNode = new Mapping;
        bool isEmpty = true;

        bodyItemNode->insert("id", id);
            
        auto& indices = selectedLinkIndices(bodyItem);
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
            Listing& unexpanded = *bodyItemNode->createFlowStyleListing("unexpanded_links");
            for(int i=0; i < n; ++i){
                if(!exps[i]){
                    unexpanded.append(i, 20, m);
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


bool LinkDeviceTreeWidget::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool LinkDeviceTreeWidget::Impl::restoreState(const Archive& archive)
{
    if(!isCacheEnabled){
        return true;
    }
    const Listing& nodes = *archive.findListing("body_items");
    if(nodes.isValid() && !nodes.empty()){
        for(int i=0; i < nodes.size(); ++i){
            const Mapping& node = *nodes[i].toMapping();
            ValueNode* id = node.find("id");
            if(id->isValid()){
                BodyItem* bodyItem = archive.findItem<BodyItem>(id);
                if(bodyItem){
                    int numLinks = bodyItem->body()->numLinks();
                    BodyItemInfoPtr info = getOrCreateBodyItemInfo(bodyItem);
                    if(auto& selected = *node.findListing("selected_links")){
                        info->setNumLinks(numLinks, true);
                        for(int i=0; i < selected.size(); ++i){
                            int index = selected[i].toInt();
                            if(index < numLinks){
                                info->selection[index] = true;
                            }
                        }
                        info->isSelectedLinkIndicesValid = false;
                    }
                    if(auto& unexpanded = *node.findListing("unexpanded_links")){
                        for(auto& node : unexpanded){
                            int index = node->toInt();
                            if(index < numLinks){
                                info->linkExpansions[index] = false;
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
    return true;
}
