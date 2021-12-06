#ifndef CNOID_BASE_UNIFIED_EDIT_HISTORY_H
#define CNOID_BASE_UNIFIED_EDIT_HISTORY_H

#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class EditRecord;
typedef ref_ptr<EditRecord> EditRecordPtr;

class CNOID_EXPORT UnifiedEditHistory
{
public:
    static void initializeClass(ExtensionManager* ext);
    static UnifiedEditHistory* instance();

    int numRecords() const;

    //! \note Index 0 is the latest record
    EditRecord* record(int index);

    void addRecord(EditRecordPtr record);
    void beginEditGroup(const std::string& label, bool isValidForSingleRecord = true);
    void endEditGroup();

    class RecordBlocker : public Referenced
    {
    public:
        virtual void setBlockedRecordGroupName(const std::string& name) = 0;
        virtual void addBlockedRecordsToHistory() = 0;
        /**
           \note This function is called in the destructor, so the handle's reset function
           can also be used to finish blocking if a single handle variable is being used.
        */
        virtual void finishBlocking() = 0;
    };
    typedef ref_ptr<RecordBlocker> RecordBlockerHandle;

    RecordBlockerHandle blockRecording(std::function<bool(EditRecord* record)> predicate);

    int currentPosition() const;
    bool isUndoable() const;
    bool isRedoable() const;
    bool undo();
    bool redo();
    void clear();

    SignalProxy<void()> sigHistoryUpdated();
    SignalProxy<void(int position)> sigCurrentPositionChanged();

    class Impl;

private:
    UnifiedEditHistory(ExtensionManager* ext);
    ~UnifiedEditHistory();
    
    Impl* impl;
};

}

#endif
