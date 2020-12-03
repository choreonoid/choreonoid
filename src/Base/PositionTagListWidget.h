#ifndef CNOID_BASE_POSITION_TAG_LIST_WIDGET_H
#define CNOID_BASE_POSITION_TAG_LIST_WIDGET_H

#include <cnoid/Signal>
#include <QTableView>
#include "exportdecl.h"

namespace cnoid {

class PositionTagGroupItem;
class MenuManager;

class CNOID_EXPORT PositionTagListWidget : public QTableView
{
public:
    PositionTagListWidget(QWidget* parent = nullptr);
    void setTagGroupItem(PositionTagGroupItem* item);
    int currentTagIndex() const;
    void setCurrentTagIndex(int tagIndex);
    const std::vector<int>& selectedTagIndices() const;
    void removeSelectedTags();

    SignalProxy<void(const std::vector<int>& selected)> sigTagSelectionChanged();
    SignalProxy<void(int tagIndex)> sigTagPressed();
    SignalProxy<void(int tagIndex)> sigTagDoubleClicked();
    SignalProxy<void(MenuManager& menu)> sigContextMenuRequest();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) override;
    virtual void dropEvent(QDropEvent *event) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
