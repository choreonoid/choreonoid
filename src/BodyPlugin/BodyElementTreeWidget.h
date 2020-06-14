#ifndef CNOID_BODY_PLUGIN_BODY_ELEMENT_TREE_WIDGET_H
#define CNOID_BODY_PLUGIN_BODY_ELEMENT_TREE_WIDGET_H

#include <cnoid/TreeWidget>
#include <cnoid/Signal>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class BodyElementTreeItem;
class BodyItem;
class Link;
class Device;
class LinkGroup;
class MenuManager;
class Archive;

class CNOID_EXPORT BodyElementTreeWidget : public TreeWidget
{
public:
    BodyElementTreeWidget(QWidget* parent = 0);
    virtual ~BodyElementTreeWidget();

    void setDefaultExpansionLevel(int level);
    void enableCache(bool on);

    enum ListingMode {
        Tree, LinkList, JointList, DeviceList, PartTree
    };

    void setListingMode(int mode);
    int listingMode() const;

    SignalProxy<void(bool isInitialCreation)> sigUpdateRequest();
            
    void setBodyItem(BodyItem* bodyItem);
    BodyItem* bodyItem();

    typedef std::function<QVariant(const BodyElementTreeItem* item, int role)> ColumnDataFunction;
    typedef std::function<void(const BodyElementTreeItem* item, int role, const QVariant& value)> ColumnSetDataFunction;
    typedef std::function<QWidget*(const BodyElementTreeItem* item)> ColumnWidgetFunction;

    int nameColumn() const;
    int idColumn() const;

    int setNumColumns(int n);
    int addColumn();
    int addColumn(const QString& headerText);
    void setColumnStretchResizeMode(int column);
    void setColumnInteractiveResizeMode(int column);
    void setColumnResizeToContentsMode(int column);
        
    void setColumnDataFunction(int column, ColumnDataFunction func);
    void setColumnSetDataFunction(int column, ColumnSetDataFunction func);
    void setColumnWidgetFunction(int column, ColumnWidgetFunction func);
    void moveVisualColumnIndex(int column, int visualIndex);

    void setNameColumnMarginEnabled(bool on);

    void setAlignedItemWidget(BodyElementTreeItem* item, int column, QWidget* widget, Qt::Alignment alignment = Qt::AlignCenter);
    QWidget* alignedItemWidget(BodyElementTreeItem* item, int column);

    void addCustomRow(BodyElementTreeItem* treeItem);
        
    BodyElementTreeItem* itemOfLink(int linkIndex);

    int numBodyElementTreeItems();

    SignalProxy<void(BodyElementTreeItem* item, int column)> sigItemChanged();
                    
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

    friend class BodyElementTreeItem;
};

class CNOID_EXPORT BodyElementTreeItem : public QTreeWidgetItem
{
public:
    int rowIndex() const { return rowIndex_; }
        
    const std::string& name() const { return name_; }
    const QString& nameText() const { return nameText_; }
    const Link* link() const { return link_; }
    Link* link() { return link_; }
    bool isLinkGroup() const { return isLinkGroup_; }
    int listingMode() const;

    virtual QVariant data(int column, int role) const override;
    virtual void setData(int column, int role, const QVariant& value) override;

private:
    int rowIndex_;
    std::string name_;
    QString nameText_;
    Link* link_;
    Device* device_;
    BodyElementTreeWidget::Impl* treeImpl;
    bool isLinkGroup_;

    BodyElementTreeItem(const std::string& name, BodyElementTreeWidget::Impl* treeImpl);
    BodyElementTreeItem(Link* link, BodyElementTreeWidget::Impl* treeImpl);
    BodyElementTreeItem(Device* device, BodyElementTreeWidget::Impl* treeImpl);
    BodyElementTreeItem(LinkGroup* linkGroup, BodyElementTreeWidget::Impl* treeImpl);

    friend class BodyElementTreeWidget;
    friend class BodyElementTreeWidget::Impl;
};
    
}

#endif
