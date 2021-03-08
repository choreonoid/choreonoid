#include "EditRecord.h"

using namespace cnoid;


EditRecord::EditRecord()
    : isReverse_(false)
{

}


EditRecord::EditRecord(const EditRecord& org)
    : isReverse_(org.isReverse_)
{

}


EditRecord* EditRecord::getFlipped() const
{
    auto flipped = clone();
    flipped->flip();
    return flipped;
}


EditRecordGroup::EditRecordGroup(const std::string& label)
    : label_(label)
{

}


EditRecordGroup::EditRecordGroup(const EditRecordGroup& org)
    : EditRecord(org),
      label_(org.label_)
{
    group_.reserve(org.group_.size());
    for(auto& record : org.group_){
        group_.push_back(record->clone());
    }
}


EditRecord* EditRecordGroup::clone() const
{
    return new EditRecordGroup(*this);
}


std::string EditRecordGroup::label() const
{
    return label_;
}


bool EditRecordGroup::undo()
{
    bool done = false;
    if(!group_.empty()){
        done = true;
        const int n = static_cast<int>(group_.size());
        for(int i = n - 1; i >= 0; --i){
            if(!group_[i]->applyUndo()){
                for(int j = i + 1; j < n; ++j){
                    group_[j]->applyRedo();
                }
                done = false;
                break;
            }
        }
    }
    return done;
}


bool EditRecordGroup::redo()
{
    bool done = false;
    if(!group_.empty()){
        done = true;
        const int n = static_cast<int>(group_.size());
        for(int i = 0; i < n; ++i){
            if(!group_[i]->applyRedo()){
                for(int j = i - 1; j >= 0; --j){
                    group_[j]->applyUndo();
                }
                done = false;
                break;
            }
        }
    }
    return done;
}
