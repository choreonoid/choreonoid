#include "UnifiedEditHistory.h"
#include "EditRecord.h"
#include "ExtensionManager.h"
#include "ProjectManager.h"
#include "Action.h"
#include "MessageView.h"
#include "LazyCaller.h"
#include <fmt/format.h>
#include <deque>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

UnifiedEditHistory* instance_;

}

namespace cnoid {

class UnifiedEditHistory::Impl
{
public:
    deque<EditRecordPtr> records;
    int currentPosition;
    size_t maxHistorySize;
    EditRecordGroupPtr currentGroup;
    bool isProjectBeingLoaded;
    vector<EditRecordPtr> newRecordBuffer;
    LazyCaller flushNewRecordBufferLater;
    Signal<void()> sigHistoryUpdated;
    Signal<void(int position)> sigCurrentPositionChanged;
    MessageView* mv;

    Impl(ExtensionManager* ext);
    void clear();
    void removeRecordsAfter(int index);
    void addRecord(EditRecord* record);
    void flushNewRecordBuffer();
    void expandHistoryFromLatestToCurrentUndoPosition();
    bool undo();
    bool redo();    
    void cancelRedo(int position);
};

}


void UnifiedEditHistory::initializeClass(ExtensionManager* ext)
{
    static UnifiedEditHistory history(ext);
    instance_ = &history;
}


UnifiedEditHistory* UnifiedEditHistory::instance()
{
    return instance_;
}


UnifiedEditHistory::UnifiedEditHistory(ExtensionManager* ext)
{
    impl = new Impl(ext);
}


UnifiedEditHistory::Impl::Impl(ExtensionManager* ext)
    : flushNewRecordBufferLater([this](){ flushNewRecordBuffer(); }, LazyCaller::LowPriority)
{
    currentPosition = 0;
    maxHistorySize = 100;

    mv = MessageView::instance();

    auto pm = ProjectManager::instance();
    pm->sigProjectAboutToBeLoaded().connect(
        [&](int recursiveLevel){
            if(recursiveLevel == 0){
                clear();
                isProjectBeingLoaded = true;
            }
        });
    pm->sigProjectLoaded().connect(
        [&](int recursiveLevel){
            if(recursiveLevel == 0){
                isProjectBeingLoaded = false;
            }
        });
    isProjectBeingLoaded = false;
}


UnifiedEditHistory::~UnifiedEditHistory()
{
    delete impl;
}


int UnifiedEditHistory::numRecords() const
{
    return impl->records.size();
}

    
EditRecord* UnifiedEditHistory::record(int index)
{
    return impl->records[index];
}


void UnifiedEditHistory::clear()
{
    impl->clear();
}


void UnifiedEditHistory::Impl::clear()
{
    removeRecordsAfter(0);
}


void UnifiedEditHistory::Impl::removeRecordsAfter(int index)
{
    auto size0 = static_cast<int>(records.size());
    records.resize(index == 0 ? 0 : (index - 1));
    auto size1 = static_cast<int>(records.size());
    if(size1 != size0){
        bool positionChanged = false;
        if(currentPosition > size1){
            currentPosition = size1;
            positionChanged = true;
        }
        sigHistoryUpdated();
        if(positionChanged){
            sigCurrentPositionChanged(currentPosition);
        }
    }
}


void UnifiedEditHistory::addRecord(EditRecordPtr record)
{
    if(!impl->isProjectBeingLoaded){
        impl->addRecord(record);
    }
}


void UnifiedEditHistory::Impl::addRecord(EditRecord* record)
{
    if(currentGroup){
        currentGroup->addRecord(record);
    } else {
        newRecordBuffer.push_back(record);
        flushNewRecordBufferLater();
    }
}


void UnifiedEditHistory::beginEditGroup(const std::string& label)
{
    if(!impl->isProjectBeingLoaded){
        impl->currentGroup = new EditRecordGroup(label);
    }
}


void UnifiedEditHistory::endEditGroup()
{
    if(!impl->isProjectBeingLoaded && impl->currentGroup){
        EditRecordGroupPtr group = impl->currentGroup;
        impl->currentGroup.reset();
        if(!group->empty()){
            addRecord(group);
        }
    }
}


void UnifiedEditHistory::Impl::flushNewRecordBuffer()
{
    if(newRecordBuffer.empty()){
        return;
    }

    if(MessageView::isFlushing()){
        flushNewRecordBufferLater();
        return;
    }

    EditRecordPtr newRecord;
    if(newRecordBuffer.size() == 1){
        newRecord = newRecordBuffer.front();
    } else {
        auto group = new EditRecordGroup(newRecordBuffer.front()->label());
        for(auto& record : newRecordBuffer){
            group->addRecord(record);
        }
        newRecord = group;
    }
    if(currentPosition >= 1){
        expandHistoryFromLatestToCurrentUndoPosition();
    }
    records.push_front(newRecord);
    
    while(records.size() > maxHistorySize &&
          currentPosition < static_cast<int>(records.size())){
        records.pop_back();
    }
    newRecordBuffer.clear();
    sigHistoryUpdated();
}


void UnifiedEditHistory::Impl::expandHistoryFromLatestToCurrentUndoPosition()
{
    vector<EditRecordPtr> undoRecords;
    for(size_t i=0; i < currentPosition; ++i){
        undoRecords.push_back(records[i]->getFlipped());
    }
    for(auto& record : undoRecords){
        records.push_front(record);
    }
    currentPosition = 0;
}


int UnifiedEditHistory::currentPosition() const
{
    return impl->currentPosition;
}


bool UnifiedEditHistory::isUndoable() const
{
    return (impl->currentPosition < impl->records.size());
}


bool UnifiedEditHistory::isRedoable() const
{
    return (impl->currentPosition >= 1);
}


bool UnifiedEditHistory::undo()
{
    return impl->undo();
}


bool UnifiedEditHistory::Impl::undo()
{
    bool done = false;

    if(currentPosition < static_cast<int>(records.size())){
        auto record = records[currentPosition];
        if(record->applyUndo()){
            mv->notify(format(_("Undo: {0}."), record->label()));
            ++currentPosition;
            sigCurrentPositionChanged(currentPosition);
            done = true;
        } else {
            record->applyRedo();
            clear();
            mv->notify(_("Undo failed and the edit history has been clered."), MessageView::Error);
        }
    }

    return done;
}


bool UnifiedEditHistory::redo()
{
    return impl->redo();
}


bool UnifiedEditHistory::Impl::redo()
{
    bool done = false;

    if(currentPosition >= 1){
        auto record = records[currentPosition - 1];
        if(record->applyRedo()){
            mv->notify(format(_("Redo: {0}."), record->label()));
            --currentPosition;
            sigCurrentPositionChanged(currentPosition);
            done = true;
        } else {
            record->applyUndo();
            clear();
            mv->notify(_("Redo failed and the edit history has been clered."), MessageView::Error);
        }
    }

    return done;
}
            

SignalProxy<void()> UnifiedEditHistory::sigHistoryUpdated()
{
    return impl->sigHistoryUpdated;
}


SignalProxy<void(int position)> UnifiedEditHistory::sigCurrentPositionChanged()
{
    return impl->sigCurrentPositionChanged;
}

