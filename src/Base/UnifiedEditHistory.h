#ifndef CNOID_BASE_UNIFIED_EDIT_HISTORY_H
#define CNOID_BASE_UNIFIED_EDIT_HISTORY_H

#include <cnoid/Signal>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class EditRecord;

class CNOID_EXPORT UnifiedEditHistory
{
public:
    static void initializeClass(ExtensionManager* ext);
    static UnifiedEditHistory* instance();

    int numRecords() const;

    //! \note Index 0 is the latest record
    EditRecord* record(int index);

    void addRecord(EditRecord* record);
    void beginRecordGroup(const std::string& label, bool isManualOperation);
    void endRecordCroup();

    int currentPosition() const;
    bool undo();
    bool redo();

    void clear();

    SignalProxy<void()> sigHistoryUpdated();
    SignalProxy<void(int position)> sigCurrentPositionChanged();

private:
    UnifiedEditHistory(ExtensionManager* ext);
    ~UnifiedEditHistory();
    
    class Impl;
    Impl* impl;
};

}

#endif
