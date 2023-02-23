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

class RecordBlockerImpl : public UnifiedEditHistory::RecordBlocker
{
public:
    UnifiedEditHistory::Impl* history;
    function<bool(EditRecord* record)> predicate;
    vector<EditRecordPtr> records;
    string groupName;

    RecordBlockerImpl(UnifiedEditHistory::Impl* history, const std::function<bool(EditRecord* record)>& predicate)
        : history(history),
          predicate(predicate) { }

    virtual ~RecordBlockerImpl();
    virtual void setBlockedRecordGroupName(const std::string& name) override;
    virtual void addBlockedRecordsToHistory() override;
    virtual void finishBlocking() override;
};

typedef ref_ptr<RecordBlockerImpl> RecordBlockerImplPtr;

}

namespace cnoid {

class UnifiedEditHistory::Impl
{
public:
    deque<EditRecordPtr> records;
    int currentPosition;
    size_t maxHistorySize;
    EditRecordGroupPtr currentGroup;
    bool isDisabled;
    vector<EditRecordPtr> newRecordBuffer;
    LazyCaller flushNewRecordBufferLater;
    Signal<void()> sigHistoryUpdated;
    Signal<void(int position)> sigCurrentPositionChanged;
    MessageView* mv;

    // Smart pointers are not used to avoid cyclic references.
    // Each blocker is externally referenced using a smart pointer and the corresponding element
    // in the following vector is automatically removed by the destructor of the blocker.
    vector<RecordBlockerImpl*> blockers;

    Impl(ExtensionManager* ext);
    ~Impl();
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
    maxHistorySize = 10;

    mv = MessageView::instance();

    auto pm = ProjectManager::instance();

    pm->sigProjectCleared().connect([&](){ clear(); });
    
    pm->sigProjectAboutToBeLoaded().connect(
        [&](int recursiveLevel){
            if(recursiveLevel == 0){
                isDisabled = true;
            }
        });
    pm->sigProjectLoaded().connect(
        [&](int recursiveLevel){
            if(recursiveLevel == 0){
                isDisabled = false;
            }
        });
    
    isDisabled = false;
}


UnifiedEditHistory::~UnifiedEditHistory()
{
    delete impl;
}


UnifiedEditHistory::Impl::~Impl()
{
    for(auto& blocker : blockers){
        blocker->history = nullptr;
    }
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
    newRecordBuffer.clear();
    removeRecordsAfter(0);
}


void UnifiedEditHistory::terminateRecording()
{
    impl->isDisabled = true;
    impl->clear();
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
    if(!impl->isDisabled){
        bool isBlocked = false;
        for(auto& blocker : impl->blockers){
            if(blocker->predicate(record)){
                blocker->records.push_back(record);
                isBlocked = true;
                break;
            }
        }
        if(!isBlocked){
            impl->addRecord(record);
        }
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


void UnifiedEditHistory::flushNewRecordBuffer()
{
    impl->flushNewRecordBuffer();
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


void UnifiedEditHistory::beginEditGroup(const std::string& label, bool isValidForSingleRecord)
{
    if(!impl->isDisabled){
        impl->currentGroup = new EditRecordGroup(label, isValidForSingleRecord);
    }
}


void UnifiedEditHistory::endEditGroup()
{
    if(!impl->isDisabled && impl->currentGroup){
        EditRecordGroupPtr group = impl->currentGroup;
        impl->currentGroup.reset();
        if(!group->empty()){
            if(group->numRecords() == 1 && !group->isValidForSingleRecord()){
                impl->addRecord(group->record(0));
            } else {
                impl->addRecord(group);
            }
        }
    }
}


UnifiedEditHistory::RecordBlockerHandle UnifiedEditHistory::blockRecording(std::function<bool(EditRecord* record)> predicate)
{
    RecordBlockerImplPtr blocker = new RecordBlockerImpl(impl, predicate);
    impl->blockers.push_back(blocker);
    return blocker;
}


RecordBlockerImpl::~RecordBlockerImpl()
{
    finishBlocking();
}


void RecordBlockerImpl::setBlockedRecordGroupName(const std::string& name)
{
    groupName = name;
}


void RecordBlockerImpl::addBlockedRecordsToHistory()
{
    if(!records.empty()){
        if(records.size() == 1 && groupName.empty()){
            history->addRecord(records.front());
        } else {
            EditRecordGroupPtr group = new EditRecordGroup(groupName);
            for(auto& record : records){
                group->addRecord(record);
            }
            history->addRecord(group);
        }
        records.clear();
    }
}


void RecordBlockerImpl::finishBlocking()
{
    if(history){
        auto& blockers = history->blockers;
        auto it = blockers.begin();
        while(it != blockers.end()){
            if(*it == this){
                blockers.erase(it);
                break;
            }
            ++it;
        }
    }
    history = nullptr;
    records.clear();
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


