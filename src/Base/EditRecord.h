#ifndef CNOID_BASE_EDIT_RECORD_H
#define CNOID_BASE_EDIT_RECORD_H

#include <cnoid/Referenced>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT EditRecord : public Referenced
{
public:
    virtual EditRecord* clone() const = 0;
    virtual std::string label() const = 0;

    bool applyUndo() { return !isReverse_ ? undo() : redo(); }
    bool applyRedo() { return !isReverse_ ? redo() : undo(); }

    void flip(){ isReverse_ = !isReverse_; }
    bool isReverse() const { return isReverse_; }
    EditRecord* getFlipped() const;

protected:
    EditRecord();
    EditRecord(const EditRecord& org);

    virtual bool undo() = 0;
    virtual bool redo() = 0;

private:
    bool isReverse_;
};

typedef ref_ptr<EditRecord> EditRecordPtr;


class CNOID_EXPORT EditRecordGroup : public EditRecord
{
public:
    EditRecordGroup(const std::string& label);

    virtual EditRecord* clone() const override;
    virtual std::string label() const override;

    void addRecord(EditRecord* record){
        group_.push_back(record);
    }
    bool empty() const { return group_.empty(); }
    int numRecords() const { return group_.size(); }
    EditRecord* record(int index){ return group_[index]; }

    virtual bool undo() override;
    virtual bool redo() override;

protected:
    EditRecordGroup(const EditRecordGroup& org);

private:
    std::vector<EditRecordPtr> group_;
    std::string label_;
};

typedef ref_ptr<EditRecordGroup> EditRecordGroupPtr;

}

#endif
