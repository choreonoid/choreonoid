/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_TREE_VIEW_H_INCLUDED
#define CNOID_GUIBASE_TREE_VIEW_H_INCLUDED

#include "ItemSelectionModel.h"
#include <cnoid/SignalProxy>
#include <QTreeView>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TreeView : public QTreeView
{
    Q_OBJECT

        public:
    TreeView(QWidget* parent = 0);

    virtual void setModel(QAbstractItemModel* model);

    ItemSelectionModel* itemSelectionModel() const;
        
    inline SignalProxy< boost::signal<void(const QModelIndex& index)> > sigCollapsed() {
        return sigCollapsed_;
    }
    inline SignalProxy< boost::signal<void(const QModelIndex& index)> > sigExpanded() {
        return sigExpanded_;
    }
    inline SignalProxy< boost::signal<void(const QModelIndex& index)> > sigActivated() {
        return sigActivated_;
    }
    inline SignalProxy< boost::signal<void(const QModelIndex& index)> > sigClicked() {
        return sigClicked_;
    }
    inline SignalProxy< boost::signal<void(const QModelIndex& index)> > sigDoubleClicked() {
        return sigDoubleClicked_;
    }
    inline SignalProxy< boost::signal<void(const QModelIndex& index)> > sigEntered() {
        return sigEntered_;
    }
    inline SignalProxy< boost::signal<void(const QModelIndex& index)> > sigPressed() {
        return sigPressed_;
    }
    inline SignalProxy< boost::signal<void()> > sigViewportEntered() {
        return sigViewportEntered_;
    }

private Q_SLOTS:
    void onCollapsed(const QModelIndex& index);
    void onExpanded(const QModelIndex& index);
    void onActivated(const QModelIndex& index);
    void onClicked(const QModelIndex& index);
    void onDoubleClicked(const QModelIndex& index);
    void onEntered(const QModelIndex& index);
    void onPressed(const QModelIndex& index);
    void onViewportEntered(void);

private:
    boost::signal<void(const QModelIndex& index)> sigCollapsed_;
    boost::signal<void(const QModelIndex& index)> sigExpanded_;
    boost::signal<void(const QModelIndex& index)> sigActivated_;
    boost::signal<void(const QModelIndex& index)> sigClicked_;
    boost::signal<void(const QModelIndex& index)> sigDoubleClicked_;
    boost::signal<void(const QModelIndex& index)> sigEntered_;
    boost::signal<void(const QModelIndex& index)> sigPressed_;
    boost::signal<void()> sigViewportEntered_;
};
}

#endif

