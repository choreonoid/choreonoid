#ifndef CNOID_BASE_POSITION_TAG_LIST_WIDGET_H
#define CNOID_BASE_POSITION_TAG_LIST_WIDGET_H

#include <cnoid/Signal>
#include <QTableView>
#include "exportdecl.h"

namespace cnoid {

class PositionTagListItem;
class MenuManager;

class CNOID_EXPORT PositionTagListWidget : public QTableView
{
public:
    PositionTagListWidget(QWidget* parent = nullptr);
    void setTagListItem(PositionTagListItem* item);
    int currentTagIndex() const;
    void removeSelectedTags();

    SignalProxy<void(int tagIndex)> sigTagPressed();
    SignalProxy<void(int tagIndex)> sigTagDoubleClicked();
    SignalProxy<void(MenuManager& menu)> sigContextMenuRequest();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
