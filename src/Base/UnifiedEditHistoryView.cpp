#include "UnifiedEditHistoryView.h"
#include "UnifiedEditHistory.h"
#include "EditRecord.h"
#include "TreeWidget.h"
#include "ViewManager.h"
#include <cnoid/ConnectionSet>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class UnifiedEditHistoryView::Impl : public TreeWidget
{
public:
    UnifiedEditHistory* history;
    ScopedConnectionSet historyConnections;
    int currentPosition;
    QFont normalFont;
    QFont boldFont;

    Impl();
    void onActivated();
    void onDeactivated();
    void updateHistory();
    void setCurrentPosition(int position);
};

}


void UnifiedEditHistoryView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<UnifiedEditHistoryView>(
        "UnifiedEditHistoryView", N_("Edit History"), ViewManager::SINGLE_OPTIONAL);
}


UnifiedEditHistoryView::UnifiedEditHistoryView()
{
    setDefaultLayoutArea(View::LEFT_BOTTOM);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    
    impl = new Impl;

    vbox->addWidget(impl);
    setLayout(vbox);
}


UnifiedEditHistoryView::Impl::Impl()
{
    history = UnifiedEditHistory::instance();
    currentPosition = 0;

    setHeaderHidden(true);
    setFrameShape(QFrame::NoFrame);
    setColumnCount(1);
    setSelectionMode(NoSelection);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

    boldFont.setBold(true);
}


UnifiedEditHistoryView::~UnifiedEditHistoryView()
{
    delete impl;
}


void UnifiedEditHistoryView::onActivated()
{
    impl->onActivated();
}


void UnifiedEditHistoryView::Impl::onActivated()
{
    historyConnections.add(
        history->sigHistoryUpdated().connect(
            [&](){ updateHistory(); }));

    historyConnections.add(
        history->sigCurrentPositionChanged().connect(
            [&](int position){ setCurrentPosition(position); }));

    updateHistory();
}


void UnifiedEditHistoryView::onDeactivated()
{
    impl->onDeactivated();
}


void UnifiedEditHistoryView::Impl::onDeactivated()
{
    historyConnections.disconnect();
}


void UnifiedEditHistoryView::Impl::updateHistory()
{
    clear();

    const int n = history->numRecords();
    for(int i=0; i < n; ++i){
        auto record = history->record(i);
        auto treeItem = new QTreeWidgetItem(this, QStringList(record->label().c_str()));
        addTopLevelItem(treeItem);
        if(auto group = dynamic_cast<EditRecordGroup*>(record)){
            const int m = group->numRecords();
            for(int j=0; j < m; ++j){
                auto record = group->record(j);
                new QTreeWidgetItem(treeItem, QStringList(record->label().c_str()));
            }
        }
    }

    setCurrentPosition(history->currentPosition());
}


void UnifiedEditHistoryView::Impl::setCurrentPosition(int position)
{
    if(position != currentPosition){
        if(currentPosition < topLevelItemCount()){
            topLevelItem(currentPosition)->setFont(0, normalFont);
        }
    }
    if(position < topLevelItemCount()){
        topLevelItem(position)->setFont(0, boldFont);
    }
    currentPosition = position;
}
