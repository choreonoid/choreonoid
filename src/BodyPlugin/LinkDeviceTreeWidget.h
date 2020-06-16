#ifndef CNOID_BODY_PLUGIN_LINK_DEVICE_TREE_WIDGET_H
#define CNOID_BODY_PLUGIN_LINK_DEVICE_TREE_WIDGET_H

#include <cnoid/TreeWidget>
#include <cnoid/Signal>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class LinkDeviceTreeItem;
class BodyItem;
class Link;
class Device;
class LinkGroup;
class MenuManager;
class Archive;

class CNOID_EXPORT LinkDeviceTreeWidget : public TreeWidget
{
public:
    LinkDeviceTreeWidget(QWidget* parent = 0);
    virtual ~LinkDeviceTreeWidget();

    enum ListingMode { List, Tree, GroupedTree };
    void setListingMode(int mode);
    int listingMode() const;
    enum NumberColumnMode { Index, Identifier };
    void setNumberColumnMode(int mode);
    int numberColumnMode() const;
    void setLinkItemVisible(bool on);
    bool isLinkItemVisible() const;
    void setDeviceItemVisible(bool on);
    bool isDeviceItemVisible() const;
    void setSortByIdEnabled(bool on);
    bool isSortByIdEnabled() const;
    void setDefaultExpansionLevel(int level);
    void setCacheEnabled(bool on);
    bool isCacheEnabled() const;

    int nameColumn() const;
    int numberColumn() const;
    int setNumColumns(int n);
    int addColumn();
    int addColumn(const QString& headerText);
    void setColumnStretchResizeMode(int column);
    void setColumnInteractiveResizeMode(int column);
    void setColumnResizeToContentsMode(int column);
    void setNameColumnMarginEnabled(bool on);
    void moveVisualColumnIndex(int column, int visualIndex);

    typedef std::function<QVariant(const LinkDeviceTreeItem* item, int role)> ColumnDataGetter;
    typedef std::function<void(const LinkDeviceTreeItem* item, int role, const QVariant& value)> ColumnDataSetter;
    typedef std::function<QWidget*(const LinkDeviceTreeItem* item)> ColumnWidgetFunction;

    void setColumnDataGetter(int column, ColumnDataGetter getter);
    void setColumnDataSetter(int column, ColumnDataSetter func);
    void setColumnWidgetFunction(int column, ColumnWidgetFunction func);

    void setAlignedItemWidget(
        LinkDeviceTreeItem* item, int column, QWidget* widget, Qt::Alignment alignment = Qt::AlignCenter);
    QWidget* alignedItemWidget(LinkDeviceTreeItem* item, int column);

    void setBodyItem(BodyItem* bodyItem);
    BodyItem* bodyItem();
    void updateTreeItems();
    void addCustomRow(LinkDeviceTreeItem* treeItem);
    SignalProxy<void(bool isInitialCreation)> sigUpdateRequest();
    LinkDeviceTreeItem* itemOfLink(int linkIndex);
    int numLinkDeviceTreeItems();
            
    SignalProxy<void(LinkDeviceTreeItem* item, int column)> sigItemChanged();
                    
    SignalProxy<void(int linkIndex)> sigCurrentLinkChanged();
    int currentLinkIndex() const;
    bool setCurrentLink(int linkIndex);
    SignalProxy<void()> sigLinkSelectionChanged();
    const std::vector<int>& selectedLinkIndices() const;
    const std::vector<bool>& linkSelection() const;

    /// This signal is available after calling 'enableCache(true)'.
    SignalProxy<void()> sigSelectionChanged(BodyItem* bodyItem);
    /// This function is available after calling 'enableCache(true)'.
    int selectedLinkIndex(BodyItem* bodyItem) const;
    /// This function is available after calling 'enableCache(true)'.
    const std::vector<int>& selectedLinkIndices(BodyItem* bodyItem);
    /// This function is available after calling 'enableCache(true)'.
    const std::vector<bool>& linkSelection(BodyItem* bodyItem);

    MenuManager& popupMenuManager();

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

protected:
    virtual void changeEvent(QEvent* event) override;

private:
    class Impl;        
    Impl* impl;

    friend class LinkDeviceTreeItem;
};

class CNOID_EXPORT LinkDeviceTreeItem : public QTreeWidgetItem
{
public:
    int rowIndex() const { return rowIndex_; }
    const std::string& name() const { return name_; }
    const QString& nameText() const { return nameText_; }
    const Link* link() const { return link_; }
    const Device* device() const { return device_; }
    bool isLinkGroup() const { return isLinkGroup_; }
    int numberColumnMode() const;

    virtual QVariant data(int column, int role) const override;
    virtual void setData(int column, int role, const QVariant& value) override;

private:
    int rowIndex_;
    std::string name_;
    QString nameText_;
    Link* link_;
    Device* device_;
    LinkDeviceTreeWidget::Impl* treeImpl;
    bool isLinkGroup_;

    LinkDeviceTreeItem(const std::string& name, LinkDeviceTreeWidget::Impl* treeImpl);
    LinkDeviceTreeItem(Link* link, LinkDeviceTreeWidget::Impl* treeImpl);
    LinkDeviceTreeItem(Device* device, LinkDeviceTreeWidget::Impl* treeImpl);
    LinkDeviceTreeItem(LinkGroup* linkGroup, LinkDeviceTreeWidget::Impl* treeImpl);

    friend class LinkDeviceTreeWidget;
    friend class LinkDeviceTreeWidget::Impl;
};
    
}

#endif
