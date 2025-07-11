#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_WIDGET_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_WIDGET_H

#include <cnoid/Signal>
#include <QTableView>
#include "exportdecl.h"

namespace cnoid {

class KinematicBodyItemSet;
class MprPositionList;

class CNOID_EXPORT MprPositionListWidget : public QTableView
{
public:
    MprPositionListWidget(QWidget* parent = nullptr);
    ~MprPositionListWidget();
    
    enum BodySyncMode { NoBodySync, DirectBodySync, TwoStageBodySync };
    void setBodySyncMode(BodySyncMode mode);
    BodySyncMode bodySyncMode() const;

    void setBodyItemSet(KinematicBodyItemSet* bodyItemSet);
    void setPositionList(MprPositionList* positionList);
    int currentPositionIndex() const;
    void setCurrentPositionIndex(int index);
    void addPosition(int row, bool doInsert);
    void addPositionIntoCurrentIndex(bool doInsert);
    void removeSelectedPositions();
    void showDefaultContextMenu(int index, const QPoint& globalPos);
    bool applyPosition(int positionIndex, bool forceDirectSync);
    void touchupCurrentPosition();

    SignalProxy<void(int index, const QPoint& globalPos)> sigContextMenuRequest();
    SignalProxy<void(int index, bool isSelectionChanged)> sigPositionPressed();
    SignalProxy<void(int index)> sigPositionDoubleClicked();
    SignalProxy<void(const QItemSelection& selected, const QItemSelection& deselected)> sigSelectionChanged();

    enum ColumnID {
        IdColumn = 0,
        NoteColumn = 1,
        JointSpaceCheckColumn = 2,
        MainPositionColumn,
        NumMinimumColumns = 4
    };

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


